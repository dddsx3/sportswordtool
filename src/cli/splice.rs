//! 支线实验：真人路段拼接（track-splice）。
//!
//! 把真人跑按打卡点切成路段（起点->P1, P1->P2, ..., P5->终点），段内数据
//! 100% 来自真机；按服务端点位的顺序拼装成整条跑步，只需处理拼接点。
//! 距离预算：起点/终点腿长度散布大，用它们调总距；中间对优先直连。

use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize)]
pub struct Leg {
    #[serde(rename = "from")]
    pub from: String,
    #[serde(rename = "to")]
    pub to: String,
    #[serde(rename = "length_m")]
    pub length_m: f64,
    #[serde(rename = "dur_s")]
    pub dur_s: i64,
    pub pts: Vec<crate::track::model::GenPoint>,
}

#[derive(Deserialize)]
pub struct LegDb {
    pub legs: Vec<Leg>,
}

#[derive(Deserialize)]
pub struct SplicePoints {
    pub points: Vec<SplicePoint>,
}

#[derive(Deserialize)]
pub struct SplicePoint {
    #[serde(rename = "pointName")]
    pub point_name: String,
    pub lat: f64,
    pub lon: f64,
}

pub fn cmd_track_splice(rest: &[&str]) -> i32 {
    let flags = super::parse_flags(rest);
    let dist = super::get(&flags, "dist")
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(2100.0);
    let seed = super::get(&flags, "seed")
        .and_then(|v| v.parse::<u64>().ok())
        .unwrap_or(1);
    let rrid = super::get(&flags, "rrid")
        .and_then(|v| v.parse::<i64>().ok())
        .unwrap_or(1320000000);
    let start_ms = super::get(&flags, "start-ms")
        .and_then(|v| v.parse::<i64>().ok())
        .unwrap_or(1793000000000);
    let uid = super::get(&flags, "uid")
        .and_then(|v| v.parse::<i64>().ok())
        .unwrap_or(15264041);
    let out = super::get(&flags, "out").unwrap_or("splice.json");
    let legs_path = super::get(&flags, "legs").unwrap_or("legs.json");
    let points_file = super::get(&flags, "points").unwrap_or("points_cache.json");
    let dur = super::get(&flags, "dur")
        .and_then(|v| v.parse::<i64>().ok())
        .unwrap_or(1200);

    let db: LegDb = match std::fs::read_to_string(legs_path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
    {
        Ok(d) => d,
        Err(e) => {
            eprintln!("读取路段库失败: {e}");
            return 1;
        }
    };
    let pf: SplicePoints = match std::fs::read_to_string(points_file)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
    {
        Ok(d) => d,
        Err(e) => {
            eprintln!("读取点位失败: {e}");
            return 1;
        }
    };
    let order: Vec<String> = pf.points.iter().map(|p| p.point_name.clone()).collect();

    // —— 路段索引：只保留可用段（≥60m，剔除切点噪声）——
    // 反向腿：真人往返段天然合理；反向点集重算 bdD（航向 +180° 语义由
    // 重算折线航向给出）。
    let mut db = db;
    let orig_count = db.legs.len();
    for i in 0..orig_count {
        let mut pts = db.legs[i].pts.clone();
        if pts.len() < 4 {
            continue;
        }
        // 时间镜像：反向腿的新时刻 = t_last - 原时刻 + t_first（保持单调）
        let orig_times: Vec<i64> = pts.iter().map(|p| p.totalTime).collect();
        let orig_vtimes: Vec<i64> = pts.iter().map(|p| p.validTime).collect();
        pts.reverse();
        let (t_first, t_last) = (orig_times[orig_times.len() - 1], orig_times[0]);
        let span = t_last - t_first;
        for (k, p) in pts.iter_mut().enumerate() {
            p.totalTime = t_first + (span - (orig_times[orig_times.len() - 1 - k] - orig_times[0]));
            // validTime 同步镜像（保持 validTime ≤ totalTime）
            p.validTime = p.totalTime.min(
                orig_vtimes[orig_times.len() - 1 - k]
                    .max(p.totalTime - (orig_vtimes[orig_times.len() - 1 - k] - t_first)),
            );
        }
        for k in 0..pts.len().saturating_sub(1) {
            let (a, b) = (&pts[k], &pts[k + 1]);
            let brg =
                ((b.gLat - a.gLat).atan2(b.gLng - a.gLng).to_degrees() + 90.0).rem_euclid(360.0);
            pts[k].bdD = (brg * 10.0).round() / 10.0;
        }
        if pts.len() >= 2 {
            let n = pts.len();
            let last_bd = pts[n - 2].bdD;
            pts[n - 1].bdD = last_bd;
        }
        db.legs.push(Leg {
            from: db.legs[i].to.clone(),
            to: db.legs[i].from.clone(),
            length_m: db.legs[i].length_m,
            dur_s: db.legs[i].dur_s,
            pts,
        });
    }
    let mut pair: std::collections::HashMap<(String, String), Vec<usize>> = Default::default();
    for (i, l) in db.legs.iter().enumerate() {
        if l.length_m >= 60.0 && l.pts.len() >= 4 && l.dur_s >= 15 {
            pair.entry((l.from.clone(), l.to.clone()))
                .or_default()
                .push(i);
        }
    }
    eprintln!(
        "[splice] legs {} (含反向 {}), pairs {}",
        db.legs.len(),
        db.legs.len() - orig_count,
        pair.len()
    );
    let pick_longest = |pair: &std::collections::HashMap<(String, String), Vec<usize>>,
                        a: &str,
                        b: &str|
     -> Option<usize> {
        pair.get(&(a.to_string(), b.to_string())).and_then(|v| {
            v.iter()
                .copied()
                .max_by_key(|&i| (db.legs[i].length_m * 100.0) as i64)
        })
    };

    // —— 链装配：起点 -> cp1..cpN -> 终点，直连优先，缺则两跳（禁自环）——
    let mut chain: Vec<(String, String)> = Vec::new();
    let mut seq: Vec<String> = vec!["起点".into()];
    seq.extend(order.iter().cloned());
    seq.push("终点".into());
    let mut used_total = 0.0f64;
    let mut visited: std::collections::HashSet<String> = std::collections::HashSet::new();
    visited.insert("起点".into());
    let pair_share = dist / seq.len() as f64;
    for w in seq.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        // 候选: 直连 / 两跳 / 三跳, 评分 = |长度-段预算| + 重访惩罚 + 跳数惩罚
        let mut cands: Vec<(f64, Vec<(String, String)>)> = Vec::new();
        if let Some(i) = pick_longest(&pair, a, b) {
            let l = db.legs[i].length_m;
            let revisit = if a != b { 0.0 } else { 5000.0 };
            cands.push((
                (l - pair_share).abs() + revisit,
                vec![(a.clone(), b.clone())],
            ));
        }
        for ((x, y), _) in pair.iter() {
            if x != a || y == a || y == b || x == b {
                continue;
            }
            let (Some(c1), Some(c2)) = (pick_longest(&pair, x, y), pick_longest(&pair, y, b))
            else {
                continue;
            };
            let total = db.legs[c1].length_m + db.legs[c2].length_m;
            let mut revisit = 0.0;
            if visited.contains(y) {
                revisit += 4000.0;
            }
            if db.legs[c1].to == *x {
                revisit += 4000.0; // 自环
            }
            cands.push((
                (total - pair_share).abs() + revisit + 250.0,
                vec![(a.clone(), y.clone()), (y.clone(), b.clone())],
            ));
            for ((y2, z), _) in pair.iter() {
                if y2 != y || z == a || z == b || z == x || z == y {
                    continue;
                }
                let (Some(c2b), Some(c3)) =
                    (pick_longest(&pair, y, y2), pick_longest(&pair, y2, b))
                else {
                    continue;
                };
                let mid2 = db.legs[c2b].to.clone();
                if mid2 == *y || mid2 == *x {
                    continue;
                }
                let total3 = db.legs[c1].length_m + db.legs[c2b].length_m + db.legs[c3].length_m;
                let mut revisit3 = 0.0;
                if visited.contains(y) {
                    revisit3 += 4000.0;
                }
                if visited.contains(y2) {
                    revisit3 += 4000.0;
                }
                cands.push((
                    (total3 - pair_share).abs() + revisit3 + 500.0,
                    vec![
                        (a.clone(), y.clone()),
                        (y.clone(), y2.clone()),
                        (y2.clone(), b.clone()),
                    ],
                ));
            }
        }
        cands.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap_or(std::cmp::Ordering::Equal));
        match cands.first() {
            Some((score, legs_sel)) => {
                for (a2, b2) in legs_sel {
                    chain.push((a2.clone(), b2.clone()));
                    used_total += pick_longest(&pair, a2, b2)
                        .map(|i| db.legs[i].length_m)
                        .unwrap_or(0.0);
                    visited.insert(b2.clone());
                }
                visited.insert(a.clone());
            }
            None => {
                eprintln!("路段覆盖不足: {a} -> {b}");
                return 1;
            }
        }
    }
    // 拼接自然长度受点位覆盖制约（部分有序对从未被真人连续跑过），
    // 允许 1.5-6km 的合理区间；配速仍由各腿自然节奏决定（~9'45"/km）
    if !(1500.0..=6000.0).contains(&used_total) {
        eprintln!("拼装总距 {used_total:.0}m 超出合理区间");
        return 1;
    }

    for (a, b) in &chain {
        if let Some(i) = pick_longest(&pair, a, b) {
            let l = &db.legs[i];
            let f = l.pts.first().unwrap();
            let t = l.pts.last().unwrap();
            eprintln!(
                "[leg] {}->{} len={:.0} 首({:.5},{:.5}) 尾({:.5},{:.5})",
                a, b, l.length_m, f.gLat, f.gLng, t.gLat, t.gLng
            );
        }
    }
    // —— 拼接：取段、重计时、去重拼接点 ——
    let mut locs: Vec<crate::track::model::GenPoint> = Vec::new();
    let mut t_off = 0i64;
    for (a, b) in &chain {
        let Some(i) = pick_longest(&pair, a, b) else {
            continue;
        };
        let leg = &db.legs[i];
        let base_t = leg.pts.first().map(|p| p.totalTime).unwrap_or(0);
        for (k, p) in leg.pts.iter().enumerate() {
            let mut q = p.clone();
            q.totalTime = t_off + (p.totalTime - base_t);
            q.validTime = t_off + (p.validTime - base_t);
            q.id = 0; // 之后统一重排
                      // 拼接点去重：与上一实点距离 <3m 时跳过（保留后段节奏）
            if let Some(last) = locs.last() {
                let d = ((q.gLat - last.gLat) * 111_000.0).hypot((q.gLng - last.gLng) * 85_000.0);
                if d < 3.0 && k == 0 {
                    continue;
                }
            }
            locs.push(q);
        }
        t_off = locs.last().map(|p| p.totalTime).unwrap_or(t_off);
    }
    if locs.len() < 30 {
        eprintln!("拼接后点数过少: {}", locs.len());
        return 1;
    }

    // —— 步数合成：真机点不带逐点 steps（计数器语义），按个人步频基准
    //    随速度形状累计（真人同速 132-174 spm）——
    {
        let mut rng_c = crate::track::geom::Rng::new(seed.wrapping_add(0xCA_D4_00_01));
        let personal = rng_c.uniform(132.0, 174.0);
        let mut steps = 0i64;
        let mut prev_t = locs.first().map(|p| p.totalTime).unwrap_or(0);
        let mut prev_pt = locs.first().map(|p| (p.gLat, p.gLng));
        let avg_v = 1.7f64;
        for p in locs.iter_mut() {
            let dt = (p.totalTime - prev_t).max(1) as f64;
            let (pla, plo) = prev_pt.unwrap_or((p.gLat, p.gLng));
            let d = ((p.gLat - pla) * 111_000.0).hypot((p.gLng - plo) * 85_000.0);
            if p.totalTime > prev_t {
                let v = d / dt;
                let shape = (0.75 + 0.5 * (v / avg_v)).clamp(0.65, 1.35);
                steps += (personal * shape / 60.0 * dt) as i64;
            }
            p.steps = steps;
            prev_t = p.totalTime;
            prev_pt = Some((p.gLat, p.gLng));
        }
    }
    // —— 协议化：哨兵/收尾/重计时 ——
    let mut rng = crate::track::geom::Rng::new(seed.wrapping_add(0x5E_11_CE));
    crate::track::postfix::apply_post_fixes(&mut locs, &mut rng, start_ms);
    // 点 id 重排（设备计数器语义由调用方注入，这里按序）
    for (i, p) in locs.iter_mut().enumerate() {
        p.id = i as i64 + 1;
    }

    let last_t = locs.last().unwrap().totalTime;
    let mut track = crate::track::model::Track {
        totalTime: last_t,
        totalDistance: dist,
        validDistance: dist,
        validTime: last_t,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: locs.last().unwrap().steps,
        locations: locs,
        speedPerTenSec: Vec::new(),
        stepsPerTenSec: Vec::new(),
        segments: Vec::new(),
        id_base: 10_000 + (seed as i64 * 137) % 6_000,
        window_id_base: 4_000 + (seed as i64 * 311) % 3_000,
        window_t0_ms: 0,
        five_point_id_base: 340 + (seed as i64 * 53) % 160,
    };
    for (i, p) in track.locations.iter_mut().enumerate() {
        p.id = track.id_base + i as i64;
    }
    // 设备语义重derive + 统计重建
    let mut rng2 = crate::track::geom::Rng::new(seed.wrapping_add(0xDE_71_CE_5A_55_E5));
    crate::track::stats::rederive_from_coordinates(&mut track, &mut rng2);
    crate::track::stats::rebuild(&mut track);
    {
        // 定位首个违约点
        let locs = &track.locations;
        for (i, p) in locs.iter().enumerate() {
            if p.totalDis < 0.0
                || p.validDis > p.totalDis
                || p.validTime > p.totalTime
                || p.steps < 0
            {
                eprintln!(
                    "[dbg] 点 {i} 非法: td={} vd={} tt={} vt={} steps={}",
                    p.totalDis, p.validDis, p.totalTime, p.validTime, p.steps
                );
                break;
            }
            if i > 0 {
                let a = &locs[i - 1];
                if p.totalTime < a.totalTime
                    || p.totalDis < a.totalDis
                    || p.steps < a.steps
                    || p.validDis < a.validDis
                    || p.validTime < a.validTime
                {
                    eprintln!("[dbg] 点 {i} 倒退: td {}->{} tt {}->{} steps {}->{} vt {}->{} vd {}->{} type {}",
                        a.totalDis, p.totalDis, a.totalTime, p.totalTime, a.steps, p.steps, a.validTime, p.validTime, a.validDis, p.validDis, p.ptype);
                    break;
                }
            }
        }
    }
    if let Err(e) = track.validate_consistency() {
        eprintln!("拼接轨迹一致性校验失败: {e}");
        return 1;
    }

    // 打卡点通过性
    // 拼接轨迹为 GCJ（真人载荷原生坐标系）；点位 BD 坐标转 GCJ 后校验，
    // 五点载荷与真人同形：lat/lon(BD) + glat/glon(GCJ, 6 位小数)。
    let mut five: Vec<serde_json::Value> = Vec::new();
    for (i, p) in pf.points.iter().enumerate() {
        let (glat, glon) = crate::track::wire::bd09_to_gcj02(p.lat, p.lon);
        five.push(serde_json::json!({
            "flag": start_ms,
            "glat": (glat * 1e6).round() / 1e6,
            "glon": (glon * 1e6).round() / 1e6,
            "id": track.five_point_id_base + i as i64,
            "isFixed": 0,
            "isPass": true,
            "lat": p.lat,
            "lon": p.lon,
            "pointName": p.point_name,
            "position": i,
            "state": 0,
        }));
    }
    for (i, p) in five.iter().enumerate() {
        let glat = p["glat"].as_f64().unwrap();
        let glon = p["glon"].as_f64().unwrap();
        let min_m = track
            .locations
            .iter()
            .map(|q| ((q.gLat - glat) * 111_000.0).hypot((q.gLng - glon) * 85_000.0))
            .fold(f64::INFINITY, f64::min);
        if min_m > 40.0 {
            eprintln!(
                "[dbg] 打卡点 {} 最近 {:.1}m 未覆盖 (chain: {:?})",
                p["pointName"].as_str().unwrap_or("?"),
                min_m,
                chain
                    .iter()
                    .map(|(a, b)| format!("{a}->{b}"))
                    .collect::<Vec<_>>()
            );
            let _ = i;
            return 1;
        }
    }

    let five_json = serde_json::json!({
        "useZip": false,
        "fivePointJson": Value::Array(five.clone()).to_string(),
        "runAreaId": -1,
        "geoFencesJson": "[]",
        "freedomShowFence": false,
    })
    .to_string();
    let uuid = uuid::Uuid::new_v4().to_string().to_uppercase();
    let laps_bias = ((uuid.bytes().map(|b| b as u64).sum::<u64>() % 220) as f64) / 10.0 - 11.0;
    let obj = crate::track::wire::build_obs_object_with_laps_bias(
        &track, rrid, &uuid, uid, &five_json, laps_bias,
    );
    let decoded = super::cmds::decode_gz_fields(&obj);
    match serde_json::to_string_pretty(&decoded) {
        Ok(s) => {
            if let Err(e) = std::fs::write(out, s) {
                eprintln!("写入 {out} 失败: {e}");
                return 1;
            }
        }
        Err(e) => {
            eprintln!("序列化失败: {e}");
            return 1;
        }
    }
    println!(
        "已保存 {out}（chain={} 段, 总距 {:.0}m, 时长 {}s, 点数 {}）",
        chain.len(),
        track.totalDistance,
        track.totalTime,
        track.locations.len()
    );
    0
}
