//! OBS 对象组装。
//!
//! 10 键对象，每值 gzip+base64；BD→GCJ 单次转换；28 键协议点集；
//! 10s 窗（speed/step_freq）；每 1000m 一圈；五点 fixed_point_json。
#![allow(non_snake_case)]

use flate2::write::GzEncoder;
use flate2::Compression;
use serde_json::{json, Value};
use std::io::Write;

use super::geom::round_to;
use super::model::{GenPoint, Track};

const X_PI: f64 = std::f64::consts::PI * 3000.0 / 180.0;

/// 百度 BD-09 → 高德 GCJ-02。
pub fn bd09_to_gcj02(bd_lat: f64, bd_lng: f64) -> (f64, f64) {
    let x = bd_lng - 0.0065;
    let y = bd_lat - 0.006;
    let z = (x * x + y * y).sqrt() - 0.00002 * (y * X_PI).sin();
    let theta = y.atan2(x) - 0.000003 * (x * X_PI).cos();
    (z * theta.sin(), z * theta.cos())
}

/// gzip + base64。
pub fn gz(data: &[u8]) -> String {
    let mut enc = GzEncoder::new(Vec::new(), Compression::default());
    enc.write_all(data).expect("gzip 写入内存失败");
    let out = enc.finish().expect("gzip 收尾失败");
    super::super::crypto::envelope::b64_encode(&out)
}

fn gz_json(v: &Value) -> String {
    gz(v.to_string().as_bytes())
}

fn gz_str(v: &str) -> String {
    gz(v.as_bytes())
}

/// 28 键协议点集（gen 点 → OBS 点；gLat/gLng 由 BD 转 GCJ）。
///
/// 字段口径全部对齐 NEUQ 真人实测（33 条 / 8081 点）：
/// - 无 `steps` 键（真人不逐点上报累计步数）；
/// - `queueNum` 为逐点递增计数器（真人 98% 非零）；
/// - `gLat/gLng/radius/totalDis` 全精度浮点（真人坐标 14 位小数、radius 完整 double）；
/// - `avgSpeed/speed/bdA/bdS` 保留 2 位、`bdD` 1 位（与真人一致）。
pub fn conv_point(p: &GenPoint, start_ms: i64, idx: usize) -> Value {
    let (glat, glng) = bd09_to_gcj02(p.gLat, p.gLng);
    json!({
        "avgSpeed": round_to(p.avgSpeed, 2),
        "bdA": round_to(p.bdA, 2),
        "bdD": round_to(p.bdD, 1),
        "bdG": p.bdG,
        "bdS": round_to(p.bdS, 2),
        "coorType": "gcj02",
        "count": p.count,
        "dtr": 0.0,
        "flag": start_ms,
        "gLat": glat,
        "gLng": glng,
        "gainTime": p.gainTime,
        "id": p.id,
        "lat": -1.0,
        "lng": -1.0,
        "locType": p.locType,
        "locationId": "",
        // 真机点级 queueNum = 该点所属 10s 窗口的序号（前导 0 由首窗
        // 覆盖的采样点数自然决定，真人 5-6 个）
        "queueNum": 0,
        // 真机 radius 为 7 位小数量级（如 3.7900925）；生成侧已在设备基准处
        // 量化到 1e-7，直接输出即为同形短小数
        "radius": p.radius,
        "speed": round_to(p.speed, 2),
        "state": p.state,
        "stepDistance": 0.0,
        "totalDis": p.totalDis,
        "totalTime": p.totalTime,
        "type": p.ptype,
        "validDis": p.validDis,
        "validTime": p.validTime,
    })
}

/// Metadata from the same live response; no made-up region identifiers.
#[derive(Clone, Debug)]
pub struct RunAreaMeta {
    pub run_area_id: i64,
    pub geo_fences: Vec<Value>,
    pub freedom_show_fence: bool,
}
impl Default for RunAreaMeta {
    fn default() -> Self {
        Self {
            run_area_id: -1,
            geo_fences: Vec::new(),
            freedom_show_fence: false,
        }
    }
}

/// Point checks use final, non-exception samples in BD-09, with the existing 40m tolerance.
pub fn passes_point(track: &Track, point: (f64, f64)) -> bool {
    track.locations.iter().filter(|p| p.ptype != -1).any(|p| {
        let dy = (p.gLat - point.0) * super::geom::MET_PER_DEG_LAT;
        let dx = (p.gLng - point.1) * 111_320.0 * point.0.to_radians().cos();
        dx * dx + dy * dy <= 40.0 * 40.0
    })
}

pub fn five_point_payload(points: &[Value], track: &Track) -> Vec<Value> {
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let bd = crate::api::fence::point_xy(p);
            let (lat, lon) = bd.unwrap_or((0.0, 0.0));
            let (glat, glon) = bd09_to_gcj02(lat, lon);
            json!({
                // 真人五点坐标取 6 位小数（App 序列化形态）
                "flag": track.startTime,
                "glat": round_to(glat, 6),
                "glon": round_to(glon, 6),
                // 点位 id 为真机 App 内部点表计数器（NEUQ 实测 346..350 连号）
                "id": track.five_point_id_base + i as i64,
                "isFixed": p.get("isFixed").cloned().unwrap_or_else(|| json!(0)),
                "isPass": bd.map(|xy| passes_point(track, xy)).unwrap_or(false),
                "lat": lat, "lon": lon,
                "pointName": p["pointName"].as_str().unwrap_or(""),
                // 真人 position 为 0..n-1 顺序号
                "position": i,
                "state": 0,
            })
        })
        .collect()
}

/// Construct once and reuse exactly for submit and OBS.
pub fn five_point_wrapper(points: &[Value], track: &Track, area: &RunAreaMeta) -> String {
    // 真机 fp 键为字母序（盲查：fivePointJson, freedomShowFence, geoFencesJson,
    // runAreaId, useZip）——序列化顺序对齐 App 端
    json!({
        "fivePointJson": Value::Array(five_point_payload(points, track)).to_string(),
        "freedomShowFence": area.freedom_show_fence,
        "geoFencesJson": Value::Array(area.geo_fences.clone()).to_string(),
        "runAreaId": area.run_area_id,
        "useZip": false,
    })
    .to_string()
}

/// 校验提交用五点轨迹仍是同一次点位请求产生的有效数据。
pub fn validate_five_point_wrapper(wrapper: &str) -> Result<(), String> {
    let outer: Value =
        serde_json::from_str(wrapper).map_err(|e| format!("五点轨迹 JSON 无效: {e}"))?;
    let raw = outer
        .get("fivePointJson")
        .and_then(Value::as_str)
        .ok_or("五点轨迹缺少 fivePointJson")?;
    let points: Vec<Value> =
        serde_json::from_str(raw).map_err(|e| format!("五点轨迹数组无效: {e}"))?;
    if points.is_empty() {
        return Err("五点轨迹不能为空".into());
    }
    for point in points {
        let lat = point.get("lat").and_then(Value::as_f64).unwrap_or(0.0);
        let lon = point.get("lon").and_then(Value::as_f64).unwrap_or(0.0);
        let glat = point.get("glat").and_then(Value::as_f64).unwrap_or(0.0);
        let glon = point.get("glon").and_then(Value::as_f64).unwrap_or(0.0);
        if (lat == 0.0 && lon == 0.0) || (glat == 0.0 && glon == 0.0) {
            return Err("五点轨迹包含缺失坐标".into());
        }
        crate::location::Coordinate::new(lat, lon, 0.0)?;
        crate::location::Coordinate::new(glat, glon, 0.0)?;
    }
    Ok(())
}

#[cfg(test)]
mod validation_tests {
    use super::*;
    #[test]
    fn rejects_empty_or_malformed_five_point_payload() {
        assert!(validate_five_point_wrapper("{}").is_err());
        assert!(validate_five_point_wrapper(r#"{"fivePointJson":"[]"}"#).is_err());
    }

    #[test]
    fn laps_are_rebuilt_from_overridden_altitude() {
        let points = vec![(38.901678, 121.540241), (38.902564, 121.541233)];
        let mut track = crate::track::generator::build(
            1200.0,
            600,
            7,
            (38.9, 121.54),
            1_700_000_000_000,
            &points,
        );
        crate::track::altitude::override_bd_a(&mut track, 36.75).unwrap();
        let laps = build_laps(&track, track.startTime);
        assert!(!laps.is_empty());
        // 平移保留 AR(1) 噪声：海拔围绕 36.75 波动、爬升非零
        assert!(laps
            .iter()
            .all(|lap| (lap["endAltAbs"].as_f64().unwrap() - 36.75).abs() < 15.0));
        assert!(laps
            .iter()
            .any(|lap| lap["elevationGain"].as_f64().unwrap() > 0.0));
    }
}

/// 10s 时间窗（真人形态：窗口由 10s 基准 + 毫秒抖动构成，9.3-12s 不等，
/// queueNum 从 0 单调递增，id 为设备窗计数器；真人 minDiff/maxDiff/avgDiff
/// 与生成一致同为 0/1000/0 占位）。
pub(crate) fn build_windows(track: &Track, _rrid: i64) -> (Vec<Value>, Vec<Value>) {
    let start_ms = track.startTime;
    let mut sp = Vec::new();
    let mut stf = Vec::new();
    let mut lo_ms = track.window_t0_ms;
    let mut cum_d = 0.0f64;
    let mut cum_s = 0.0f64;
    // 真机 3/33 文件的窗口带 state∈{1,2}（定位状态透传）
    let mut rng_state = super::geom::Rng::new(start_ms as u64 ^ 0x57_1E_57_1E);
    let win_state = if rng_state.random() < 0.09 {
        rng_state.weighted(&[(0, 6), (1, 2), (2, 1)])
    } else {
        0
    };
    for (k, a) in track.speedPerTenSec.iter().enumerate() {
        let b = &track.stepsPerTenSec[k];
        let hi_ms = lo_ms + a.time;
        cum_d += a.value;
        cum_s += b.value;
        let id = track.window_id_base + k as i64;
        // 真机 speed/step 的 id 共享窗计数器；queueNum 各自独立 0..N-1
        // （盲查实测：speed.qn 与 steps.qn 均为 0,1,2,... 步长 1）
        sp.push(json!({
            "beginTime": start_ms + lo_ms,
            "distance": a.value,
            "endTime": start_ms + hi_ms,
            "flag": start_ms,
            "id": id,
            "queueNum": k as i64,
            "state": 0,
        }));
        // 步数取累计差，保证窗口求和与总步数一致
        let steps_num = cum_s.round() - (cum_s - b.value).round();
        stf.push(json!({
            "avgDiff": 0.0,
            "beginTime": start_ms + lo_ms,
            "endTime": start_ms + hi_ms,
            "flag": start_ms,
            "id": id,
            "maxDiff": 0.0,
            "minDiff": 1000.0,
            "queueNum": k as i64,
            "state": 0,
            "stepsNum": steps_num as i64,
        }));
        lo_ms = hi_ms;
    }
    (sp, stf)
}

/// segment_json：真人与 10s 窗一一对应（边界取整秒），avgSpeed 单位 min/km
/// 全精度，avgStep 为窗口步/分，id 沿用设备窗计数器，speedStandard=900。
fn build_segments(track: &Track) -> Vec<Value> {
    // 真人 segment id 与 speed id 是两条独立自增序列（盲查：偏移 -17~+7
    // 随条变化）——用独立派生基准，避免同池重合
    let seg_id_base = track.window_id_base.wrapping_add(
        (super::geom::Rng::new(track.startTime as u64 ^ 0x5E_61_D7A).uniform(-40.0, 40.0)) as i64,
    );
    // 段 state 131072 罕见变体（真机 2/33 文件）
    let seg_state = if super::geom::Rng::new(track.startTime as u64 ^ 0x5E6_57_1E).random() < 0.06 {
        131072
    } else {
        0
    };
    track
        .segments
        .iter()
        .enumerate()
        .map(|(k, s)| {
            json!({
                "avgSpeed": s.avgSpeed,
                "avgStep": s.avgStep,
                "distance": s.distance,
                "endTime": s.endTime,
                "flag": track.startTime,
                "id": seg_id_base + k as i64,
                "intervalStepModel": 0,
                "originAvgStep": s.avgStep,
                "speedStandard": 900,
                "startTime": s.startTime,
                "state": seg_state,
            })
        })
        .collect()
}

/// 圈（每 1000m 一圈，末圈 isFullLap=false）。
/// NEUQ 真人口径：avgPace 为十进制分钟（如 9.63 = 9'38"/km），avgCadence /
/// avgStride 全精度，海拔/爬升来自含噪声的 bdA 序列。
fn build_laps(track: &Track, start_ms: i64) -> Vec<Value> {
    let mut laps = Vec::new();
    let locs = &track.locations;
    let Some(last) = locs.last() else { return laps };
    let (mut prev_d, mut prev_t, mut prev_steps, mut prev_gain) = (0.0f64, 0i64, 0i64, 0.0f64);
    let alt0 = locs.first().map(|p| p.bdA).unwrap_or(0.0);
    let lap_id_base = track.window_id_base % 83 + 3;
    while prev_d < last.totalDis {
        let d_now = (prev_d + 1000.0).min(last.totalDis);
        let mut gain = 0.0;
        let (mut t_now, mut steps, mut altitude) = (last.totalTime, last.steps, last.bdA);
        for pair in locs.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let climb = (b.bdA - a.bdA).max(0.0);
            // The final boundary uses the actual last sample, including any stop.
            if d_now < last.totalDis && b.totalDis >= d_now && b.totalDis > a.totalDis {
                let k = (d_now - a.totalDis) / (b.totalDis - a.totalDis);
                t_now =
                    (a.totalTime as f64 + (b.totalTime - a.totalTime) as f64 * k).round() as i64;
                steps = (a.steps as f64 + (b.steps - a.steps) as f64 * k).round() as i64;
                altitude = a.bdA + (b.bdA - a.bdA) * k;
                gain += climb * k;
                break;
            }
            gain += climb;
        }
        let lap_d = d_now - prev_d;
        let lap_t = t_now - prev_t;
        let lap_steps = steps - prev_steps;
        // 真人 avgPace = 十进制分钟每公里
        let avg_pace = if lap_t > 0 && lap_d > 0.0 {
            (lap_t as f64 / 60.0) / (lap_d / 1000.0)
        } else {
            0.0
        };
        laps.push(json!({
                "avgCadence": if lap_t > 0 { json!(lap_steps as f64 / (lap_t as f64 / 60.0)) } else { Value::Null },
                "avgPace": if lap_t > 0 { json!(avg_pace) } else { Value::Null },
                "avgStride": if lap_steps > 0 { json!(lap_d / lap_steps as f64 * 100.0) } else { Value::Null },
                "cumulativeDuration": t_now,
                "distance": lap_d,
                "duration": lap_t,
                "elevationGain": gain - prev_gain,
                "endAltAbs": altitude,
                "endAltRel": altitude - alt0,
                "flag": start_ms,
                "id": lap_id_base + laps.len() as i64,
                "isFullLap": lap_d >= 1000.0,
                "lapIndex": laps.len() as i64 + 1,
                "step": lap_steps,
            }));
        prev_d = d_now;
        prev_t = t_now;
        prev_steps = steps;
        prev_gain = gain;
    }
    laps
}

/// 组装 10 键 OBS 对象（值均 gzip+base64；runFaceCheck 为空串 gzip；
/// segment_json 为真人同构的分段数组）。
pub fn build_obs_object(
    track: &Track,
    rrid: i64,
    uuid: &str,
    uid: i64,
    five_point_json: &str,
) -> Value {
    build_obs_object_with_laps_bias(track, rrid, uuid, uid, five_point_json, 0.0)
}

/// laps_bias: 圈爬升与 bdA 正差累加的独立偏差（真机 laps 用独立气压计/滤波
/// 源，与轨迹 bdA 相差 2-22m）。
pub fn build_obs_object_with_laps_bias(
    track: &Track,
    rrid: i64,
    uuid: &str,
    uid: i64,
    five_point_json: &str,
    laps_bias: f64,
) -> Value {
    let start_ms = track.startTime;
    let mut pts: Vec<Value> = track
        .locations
        .iter()
        .enumerate()
        .map(|(i, p)| conv_point(p, start_ms, i))
        .collect();
    // 点级 queueNum = 所属 10s 窗序号（由窗口边界映射）
    {
        let mut boundaries: Vec<i64> = Vec::with_capacity(track.speedPerTenSec.len());
        let mut acc = track.window_t0_ms;
        for w in &track.speedPerTenSec {
            acc += w.time;
            boundaries.push(acc);
        }
        for (i, p) in pts.iter_mut().enumerate() {
            let t = p.get("totalTime").and_then(|v| v.as_i64()).unwrap_or(0) * 1000;
            // 真人前导 0 有 5-6 个（首窗覆盖更多采样点）：序号偏移 -2
            let qn = (boundaries.iter().filter(|&&b| b <= t).count() as i64 - 2).max(0);
            p["queueNum"] = json!(qn);
        }
    }
    let run_wrap = json!({ "allLocJson": Value::Array(pts).to_string(), "useZip": false });
    let (sp, stf) = build_windows(track, rrid);
    // 真机 OBS 对象内 rrid 恒为 "0"（真实记录 33/33），键路径仍携带真实 rrid
    let rrid_obj = 0i64;

    let mut laps = build_laps(track, start_ms);
    if laps_bias.abs() > 1e-9 {
        // 乘法偏差（1.0±0.25），负爬升不符合累计语义，保底 0
        for lap in laps.iter_mut() {
            if let Some(g) = lap.get("elevationGain").and_then(|v| v.as_f64()) {
                lap["elevationGain"] = json!((g * (1.0 + laps_bias / 44.0)).max(0.0));
            }
        }
    }
    let segments = build_segments(track);
    // 真机顶层键序（盲查实测）：uid, run_data, step_freq_json, speed_json,
    // runFaceCheck, laps_json, uuid, rrid, segment_json, fixed_point_json
    let mut obj = json!({
        "uid": gz_str(&uid.to_string()),
        "run_data": gz_json(&run_wrap),
        "step_freq_json": gz_json(&Value::Array(stf)),
        "speed_json": gz_json(&Value::Array(sp)),
        "runFaceCheck": gz_str(""),
        "laps_json": gz_json(&Value::Array(laps)),
        "uuid": gz_str(uuid),
        "rrid": gz_str(&rrid_obj.to_string()),
        "segment_json": gz_json(&Value::Array(segments)),
        "fixed_point_json": gz_str(five_point_json),
    });
    // 真机 6/33 记录整个缺失 laps_json 键（App 版本行为）
    if laps_bias.is_nan() {
        obj.as_object_mut().unwrap().remove("laps_json");
    }
    obj
}

/// OBS objectKey（两个都要传）。
pub fn obs_keys(track: &Track, rrid: i64, uuid: &str) -> Vec<String> {
    let t0 = super::geom::fmt_cst(track.startTime, "%Y%m%d%H");
    vec![
        format!("run_data/{t0}/{uuid}.json"),
        format!("run_data/{}/{}.json", rrid / 1000000, rrid),
    ]
}

#[cfg(test)]
mod regression_tests {
    use super::*;
    fn linear_track(distance: f64, seconds: i64, steps: i64) -> Track {
        let mut track = crate::track::generator::build(
            1200.0,
            600,
            1,
            (39.9, 116.4),
            0,
            &[(39.9, 116.4), (39.901, 116.401)],
        );
        let mut a = track.locations[0].clone();
        a.gLat = 39.9;
        a.gLng = 116.4;
        a.totalTime = 0;
        a.totalDis = 0.0;
        a.steps = 0;
        let mut b = a.clone();
        b.totalTime = seconds;
        b.totalDis = distance;
        b.steps = steps;
        track.startLatitude = a.gLat;
        track.startLongitude = a.gLng;
        track.locations = vec![a, b];
        crate::track::stats::rebuild(&mut track);
        track
    }
    #[test]
    fn lap_units_partial_laps_and_zero_steps() {
        // NEUQ 真人口径：avgPace = 十进制分钟/公里
        let track = linear_track(1000.0, 600, 1200);
        let laps = build_laps(&track, 0);
        assert_eq!(laps.len(), 1);
        assert_eq!(laps[0]["avgPace"], 10.0);
        assert_eq!(laps[0]["isFullLap"], true);
        let partial = build_laps(&linear_track(500.0, 300, 600), 0);
        assert_eq!(partial[0]["isFullLap"], false);
        assert_eq!(partial[0]["avgPace"], 10.0);
        assert!(build_laps(&linear_track(1000.0, 600, 0), 0)[0]["avgStride"].is_null());
        assert!(build_laps(&linear_track(0.0, 600, 0), 0).is_empty());
        let laps = build_laps(&linear_track(2500.0, 1500, 3000), 0);
        assert_eq!(laps.len(), 3);
        let tiny_tail = build_laps(&linear_track(1000.001, 600, 1200), 0);
        assert_eq!(
            tiny_tail
                .iter()
                .map(|l| l["duration"].as_i64().unwrap())
                .sum::<i64>(),
            600
        );
        assert!(tiny_tail.last().unwrap()["avgPace"].is_null());
        assert_eq!(
            laps.iter()
                .map(|l| l["distance"].as_f64().unwrap())
                .collect::<Vec<_>>(),
            vec![1000.0, 1000.0, 500.0]
        );
        assert_eq!(
            laps.iter()
                .map(|l| l["duration"].as_i64().unwrap())
                .sum::<i64>(),
            1500
        );
        assert_eq!(
            laps.iter()
                .map(|l| l["step"].as_i64().unwrap())
                .sum::<i64>(),
            3000
        );
    }
    #[test]
    fn encoded_obs_lap_pace_uses_decimal_minutes() {
        for (distance, seconds, pace) in [
            (1000.0, 408, 6.8),
            (1000.0, 419, 419.0 / 60.0),
            (500.0, 204, 6.8),
            (10.0, 9, 15.0),
        ] {
            let track = linear_track(distance, seconds, 1200);
            let obs = build_obs_object(&track, 1, "fixture", 1, "{}");
            let bytes =
                crate::crypto::envelope::b64_decode(obs["laps_json"].as_str().unwrap()).unwrap();
            let mut decoded = String::new();
            use std::io::Read;
            flate2::read::GzDecoder::new(&bytes[..])
                .read_to_string(&mut decoded)
                .unwrap();
            let laps: Vec<Value> = serde_json::from_str(&decoded).unwrap();
            assert_eq!(laps.len(), 1);
            assert!(
                (laps[0]["avgPace"].as_f64().unwrap() - pace).abs() < 1e-9,
                "pace={}",
                laps[0]["avgPace"]
            );
            assert_eq!(laps[0]["duration"], seconds);
            assert_eq!(laps[0]["cumulativeDuration"], seconds);
            assert_eq!(laps[0]["distance"], distance);
        }
        // 跨公里边界不得把舍入泄进总量
        let laps = build_laps(&linear_track(1010.0, 408, 1200), 0);
        assert!((laps[0]["avgPace"].as_f64().unwrap() - 404.0 / 60.0).abs() < 1e-9);
        assert_eq!(
            laps.iter()
                .map(|l| l["duration"].as_i64().unwrap())
                .sum::<i64>(),
            408
        );
    }
    #[test]
    fn point_status_and_metadata_are_identical_in_obs() {
        let track = linear_track(1000.0, 600, 1200);
        let points = vec![
            json!({"id":17,"lat":39.9,"lon":116.4,"isFixed":0}),
            json!({"id":18,"lat":39.91,"lon":116.41,"isFixed":1}),
        ];
        let payload = five_point_payload(&points, &track);
        assert_eq!(payload[0]["isPass"], true);
        assert_eq!(payload[1]["isPass"], false);
        assert_eq!(payload[0]["isFixed"], 0);
        assert_eq!(payload[1]["isFixed"], 1);
        // 点位 id 来自设备点表计数器（真机 NEUQ 346..），不再透传服务端字段
        assert_eq!(payload[0]["id"], track.five_point_id_base);
        assert_eq!(payload[1]["id"], track.five_point_id_base + 1);
        let area = RunAreaMeta {
            run_area_id: 42,
            geo_fences: vec![json!({"id":42})],
            freedom_show_fence: true,
        };
        let wrapper = five_point_wrapper(&points, &track, &area);
        let obj = build_obs_object(&track, 1, "test", 1, &wrapper);
        let bytes =
            crate::crypto::envelope::b64_decode(obj["fixed_point_json"].as_str().unwrap()).unwrap();
        let mut decoded = String::new();
        use std::io::Read;
        flate2::read::GzDecoder::new(&bytes[..])
            .read_to_string(&mut decoded)
            .unwrap();
        assert_eq!(decoded, wrapper);
        let lt = linear_track(1100.0, 603, 1201);
        // startTime 带真机毫秒偏移（flag=now_ms），窗口时刻相对 startTime
        let st = lt.startTime;
        let (speed, steps) = build_windows(&lt, 60);
        assert_eq!(speed.last().unwrap()["endTime"], st + 603000);
        assert!(speed.last().unwrap()["beginTime"].as_i64().unwrap() < st + 603000);
        // 真机 speed/steps 的 queueNum 各自独立 0..N-1（盲查实测步长 1）
        assert_eq!(speed[0]["queueNum"], 0);
        assert_eq!(speed[1]["queueNum"], 1);
        assert_eq!(steps[0]["queueNum"], 0);
        assert_eq!(steps[1]["queueNum"], 1);
        assert_eq!(
            steps
                .iter()
                .map(|v| v["stepsNum"].as_i64().unwrap())
                .sum::<i64>(),
            1201
        );
    }
    #[test]
    fn protocol_times_use_beijing_even_on_utc_hosts() {
        assert_eq!(crate::track::geom::fmt_gain_time(0), "1970-01-01 08:00:00");
        let mut track = linear_track(1000.0, 600, 1200);
        track.startTime = 16 * 3600 * 1000;
        assert_eq!(
            obs_keys(&track, 1, "test")[0],
            "run_data/1970010200/test.json"
        );
    }
}
