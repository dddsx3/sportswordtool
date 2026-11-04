//! 自然轨迹生成器。
//!
//! 画像驱动：打卡点拟合闭合环（Catmull-Rom，每段 18 采样）→ 弧长表；
//! 采样间隔主 5s（80%）；速度曲线 = ramp × 疲劳 × 三正弦 × 余弦凹陷 × 微噪；
//! 正常点按速度曲线分配位移并归一到精确总距离；异常点（-1）零位移/跳变；
//! 哨兵点（首 type∈{0,7}、索引1 type=5、末 type=6）；结尾断崖；点位吸附。
#![allow(non_snake_case)]

use super::geom::{
    fmt_gain_time, make_point_ring, ring_point_at, round_to, to_bd, Rng, MET_PER_DEG_LAT,
    MET_PER_DEG_LNG,
};
use super::model::{GenPoint, Track};
use super::postfix::apply_post_fixes;

/// GPS 瞬时速度尖峰上限的异常点低速口径（km/h）。
pub const MAX_EXCEPTION_GPS_SPEED_KMH: f64 = 6.0;

/// 有效配速窗口（m/s），保留原有生成器边界。
/// 下限对齐 NEUQ 真人均速 1.67 m/s（9'50"/km），留余量。
pub const SPEED_FLOOR: f64 = 1.55;
pub const SPEED_CEIL: f64 = 6.30;

/// 丢星点（type=-1）位移模型（NEUQ 真人实测/XieRun 基线一致）：
/// 42% 原地复制、50% 漂 0.6-20m、8% 漂 20-50m。
pub const LOST_FIX_HOLD: f64 = 0.42;
pub const LOST_FIX_NEAR: f64 = 0.50; // 其余为远漂（20-50m）

/// 等比缩放逐点速度至目标总距：越界点钳在窗口边界，剩余差量由未饱和点分摊（迭代收敛）。
/// 与整体等比缩放的区别：任何一点的瞬时配速都不会越出有效窗口。
fn fit_speeds(w: &mut [f64], dts: &[f64], target: f64) {
    for _ in 0..24 {
        let cur: f64 = w.iter().zip(dts).map(|(x, dt)| x * dt).sum();
        if (cur - target).abs() <= 1.0 {
            break;
        }
        let k = target / cur;
        for x in w.iter_mut() {
            *x = (*x * k).clamp(SPEED_FLOOR, SPEED_CEIL);
        }
    }
    for x in w.iter_mut() {
        *x = x.clamp(SPEED_FLOOR, SPEED_CEIL);
    }
}

/// 轨迹生成主入口。points_bd 为 BD 系打卡点。
pub fn build(
    dist: f64,
    dur: i64,
    seed: u64,
    _center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
) -> Track {
    let mut rng = Rng::new(seed);
    // 真机 flag=开跑时刻 now_ms：毫秒非零且每条恒定（同 generate_road）
    let start_ms = start_ms + 13 + (rng.uniform(0.0, 974.0)) as i64;
    let dur_f = dur as f64;
    let (dense, arcs, pc) = make_point_ring(points_bd);
    let (c_lat, c_lng) = pc;
    let direction: f64 = rng.choice(&[1.0, -1.0]);
    let s0 = rng.uniform(0.0, *arcs.last().unwrap_or(&400.0));
    let phase_v = rng.uniform(0.0, std::f64::consts::TAU);
    let phase_l = rng.uniform(0.0, std::f64::consts::TAU);

    let mut times = Vec::new();
    let mut t = 0.0;
    while t < dur_f {
        times.push(t);
        // 采样间隔对齐 NEUQ 真人（33 条实测）：主 5s ~88.5%，尾部 6s 主导、
        // 4s 次之、7-9s 罕见（真机定位节流形态）
        t += if rng.random() < 0.955 {
            5.0
        } else {
            rng.weighted(&[(4.0, 2), (6.0, 3), (9.0, 1)])
        };
    }
    let n = times.len();

    let n_dips = rng.choice(&[1, 1, 2]);
    let mut dips = Vec::new();
    for _ in 0..n_dips {
        dips.push((
            rng.uniform(0.15, 0.75) * dur_f,
            rng.uniform(25.0, 55.0),
            rng.uniform(0.08, 0.16),
        ));
    }
    let dip_factor = |t: f64| -> f64 {
        let mut f = 1.0;
        for &(c, hw, d) in &dips {
            if (t - c).abs() < hw {
                f *= 1.0 - d * 0.5 * (1.0 + std::f64::consts::PI * (t - c) / hw).cos();
            }
        }
        f
    };
    let base = dist / dur_f;
    let mut w = Vec::with_capacity(n);
    for &tt in &times {
        let ramp = if tt < 8.0 {
            (0.85 + 0.15 * (tt / 1.0f64.max(8.0f64.min(dur_f / 20.0)))).min(1.0)
        } else {
            1.0
        } * if tt > dur_f - 8.0 {
            1.0 + 0.03 * (tt - (dur_f - 8.0)) / 8.0
        } else {
            1.0
        };
        let km_done = (tt / dur_f) * dist / 1000.0;
        let fatigue = if km_done <= 0.5 {
            1.03
        } else {
            (1.03 - 0.06 * (km_done - 0.5)).max(0.80)
        };
        let wave = (1.0
            + 0.010 * (std::f64::consts::TAU * tt / 115.0 + phase_v).sin()
            + 0.035 * (std::f64::consts::TAU * tt / 47.0 + phase_v * 2.3).sin()
            + 0.015 * (std::f64::consts::TAU * tt / 19.0 + phase_v * 3.7).sin())
            * dip_factor(tt);
        let noise = 1.0 + rng.gauss(0.0, 0.008);
        w.push(base * ramp * fatigue * wave * noise);
    }
    let mut dts: Vec<f64> = (0..n - 1).map(|i| times[i + 1] - times[i]).collect();
    dts.push(1.0f64.max(dur_f - times[n - 1]));
    // 逐点速度全部约束在有效配速窗口内，并精确命中目标距离
    fit_speeds(&mut w, &dts, dist);
    let seg_dist: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();

    // 点位类型（NEUQ 33 条实测）：0 占 96.8%、2 占 2.0%、3 占 0.25%、-1 占 0.07%；
    // 真人不存在 type 1/8；首点/idx1/末点哨兵由 postfix 处理。
    let mut kinds: Vec<(i64, i64)> = Vec::with_capacity(n);
    for _ in 0..n {
        let u = rng.random();
        if u < 0.0202 {
            kinds.push((2, 1));
        } else if u < 0.0227 {
            kinds.push((3, 1));
        } else if u < 0.0234 {
            kinds.push((-1, 1));
        } else {
            kinds.push((0, 1));
        }
    }
    for i in 1..n {
        if kinds[i].0 == -1 && kinds[i - 1].0 == -1 {
            kinds[i] = (0, 1);
        }
    }
    // 逐次运行的 GPS 质量（真人：个别run整体走 WiFi/基站定位，其余基本全 GPS）。
    let state2_rate = if rng.random() < 0.06 {
        rng.uniform(0.60, 0.95)
    } else {
        rng.uniform(0.0, 0.02)
    };
    // 设备精度指纹（真人 radius 98% 恒定，罕见离群到 56m）。
    let radius_base = crate::api::model::device_profile().radius_base;
    let normal_idx: Vec<usize> = (0..n).filter(|&i| kinds[i].0 != -1 && i > 1).collect();
    let share: f64 = normal_idx.iter().map(|&i| seg_dist[i]).sum();
    let share = if share == 0.0 { 1.0 } else { share };
    let mut disp_of = vec![0.0f64; n];
    for &i in &normal_idx {
        disp_of[i] = seg_dist[i] * (dist / share);
    }

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = s0;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut jx = 0.0f64;
    let mut jy = 0.0f64;
    // 海拔：AR(1) 过程（NEUQ 真人 φ 中位 0.93、ε σ 中位 1.29m，起点 0-40m 沿海场地）
    let flat_run = rng.random() < 0.2;
    let mut alt = rng.uniform(-1.0, 25.0);
    let alt_base = alt;
    let alt_phi = rng.uniform(0.85, 0.97);
    let alt_sigma = if flat_run {
        0.05
    } else {
        rng.uniform(0.9, 2.6)
    };
    // 卫星数：真机为持续过程（lag-1 自相关 0.64-0.84），非逐点独立
    let mut count_next = (38.0 + rng.gauss(0.0, 6.0)).round().clamp(0.0, 80.0) as i64;
    // bdS 每 run 比例（SDK 速度换算系数因设备而异）
    let bds_ratio = rng.uniform(0.55, 0.8);
    // 网络定位 episode（劣 GPS run 的 radius 恒定 9-11m 连续段）
    let episode_radius = rng.uniform(9.0, 11.0);
    let radius_variable = rng.random() < 0.4;
    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        let pos = |ss: f64| ring_point_at(&dense, &arcs, ss);
        let mut d_step = 0.0f64;
        let px;
        let py;
        let x;
        let y;
        let rad;
        let state;
        if typ != -1 {
            d_step = disp_of[i];
            s += direction * d_step;
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            jx = 0.72 * jx + rng.gauss(0.0, 0.75);
            jy = 0.72 * jy + rng.gauss(0.0, 0.75);
            px = bx + jx;
            py = by + jy;
            // 设备精度指纹：98% 恒定基准，罕见离群（真人 3.79 → 最长 56m）
            // radius 每 run 风格：60% 近恒定（个别机型），40% 变化型
            // （真机 accuracy std 2.8-7.0m，1/2/6/7 位小数混合）
            rad = if state2_rate > 0.5 {
                episode_radius * (1.0 + rng.gauss(0.0, 0.08))
            } else if radius_variable {
                ((rng.uniform(2.2, 30.0).powf(1.35)) * 1e6).round() / 1e6
            } else if rng.random() < 0.9 {
                radius_base * (1.0 + rng.gauss(0.0, 0.05))
            } else {
                ((rng.uniform(8.0, 56.0)) * 1e7).round() / 1e7
            };
            state = if rng.random() < state2_rate {
                2
            } else if state2_rate > 0.5 && rng.random() < 0.04 {
                3
            } else {
                1
            };
        } else {
            let u = rng.random();
            if u < LOST_FIX_HOLD {
                d_step = 0.0;
            } else if u < LOST_FIX_HOLD + LOST_FIX_NEAR {
                d_step = rng.uniform(0.6, 20.0);
            } else {
                d_step = rng.uniform(20.0, 50.0);
            }
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            if d_step > 0.0 {
                let ang = rng.uniform(0.0, std::f64::consts::TAU);
                px = x + d_step * ang.sin();
                py = y + d_step * ang.cos();
            } else if let Some(last) = locs.last() {
                // 零位移：精确复制上一点坐标（BD→平面）
                py = (last.gLat - c_lat) * MET_PER_DEG_LAT;
                px = (last.gLng - c_lng) * MET_PER_DEG_LNG;
            } else {
                px = x + jx;
                py = y + jy;
            }
            rad = ((rng.uniform(8.0, 56.0)) * 1e7).round() / 1e7;
            state = rng.weighted(&[(1, 30), (2, 60), (3, 10)]);
        }
        if typ != -1 {
            dist_acc += d_step; // 异常点漂移不计入累计距离
        }
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt = alt_phi * alt + (1.0 - alt_phi) * alt_base + rng.gauss(0.0, alt_sigma);
        let v_now = dist / dur_f;
        let stride_target = 0.62
            + 0.17 * v_now
            + 0.03 * (std::f64::consts::TAU * t_acc / 200.0 + phase_l).sin()
            + rng.gauss(0.0, 0.008);
        let v_cad = v_now * (1.0 + 0.03 * (std::f64::consts::TAU * t_acc / 70.0 + phase_v).sin());
        let cad = (v_cad / stride_target * 60.0).clamp(100.0, 200.0);
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + direction * 2.0);
        let brg =
            ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 14.0)).rem_euclid(360.0);
        // 异常点（-1）：avgSpeed 为累计均值（真人与此一致，不为 0）；
        // GPS 瞬时速度为多普勒口径：真人均值的 1.15-1.7 倍，偶发 0 与尖峰毛刺
        let (avg_sp, gps_speed) = if typ == -1 {
            let avg = round_to(dist_acc / t_acc.max(1.0), 4);
            let gps = rng.uniform(0.5, super::generator::MAX_EXCEPTION_GPS_SPEED_KMH);
            (avg, round_to(gps, 4))
        } else {
            let avg = round_to(d_step / dt, 4);
            let kmh = avg * 3.6;
            // 真机毛刺两级化（真人分布）：小毛刺 ~30-120 km/h 常见，
            // 灾难值 1000-8500 km/h 罕见（每 run 至多一两个）
            let r = rng.random();
            let gps = if r < 0.004 {
                0.0
            } else if r < 0.012 {
                rng.uniform(30.0, 120.0)
            } else if r < 0.014 {
                rng.uniform(1000.0, 8500.0)
            } else {
                // 与位移同源：均值 = 位移速率，乘性噪声保持相关性
                (kmh * (1.0 + rng.gauss(0.0, 0.18))).max(0.0)
            };
            (avg, round_to(gps, 2))
        };

        locs.push(GenPoint {
            id: i as i64 + 1,
            flag: start_ms,
            lat: -1.0,
            lng: -1.0,
            gLat: round_to(lat, 7),
            gLng: round_to(lng, 7),
            speed: round_to(gps_speed, 2),
            avgSpeed: avg_sp,
            radius: rad,
            accuracy: rad,
            ptype: typ,
            locType: lt,
            hasAltitude: true,
            totalTime: round_to(t_acc, 0) as i64,
            // 全精度累计距离（真人 totalDis 为原始 double 累计值）
            totalDis: dist_acc,
            validDis: dist_acc,
            validTime: round_to(t_acc, 0) as i64,
            steps: steps_acc as i64,
            stepDistance: 0.0,
            gainTime: fmt_gain_time(start_ms + (t_acc * 1000.0) as i64),
            gainTimeMs: start_ms + (t_acc * 1000.0) as i64,
            queueNum: 0,
            coorType: "gcj02".into(),
            bdA: round_to(alt, 2),
            bdD: round_to(brg, 1),
            bdS: round_to(
                (avg_sp * bds_ratio * (1.0 + rng.gauss(0.0, 0.08))).max(0.0),
                2,
            ),
            bdG: 1,
            count: count_next,
            dtr: 0.0,
            state,
            locationId: String::new(),
        });
        count_next = (count_next as f64 * 0.75 + 38.0 * 0.25 + rng.gauss(0.0, 5.0))
            .round()
            .clamp(0.0, 80.0) as i64;
    }
    apply_post_fixes(&mut locs, &mut rng, start_ms);

    // 点位吸附：<40m 精确落位
    for pl in points_bd {
        let mut best_i = None;
        let mut best_d = 1e18f64;
        for (i, q) in locs.iter().enumerate().filter(|(_, q)| q.ptype != -1) {
            let dd = ((q.gLat - pl.0) * MET_PER_DEG_LAT).powi(2)
                + ((q.gLng - pl.1) * MET_PER_DEG_LNG).powi(2);
            if dd < best_d {
                best_d = dd;
                best_i = Some(i);
            }
        }
        if let Some(i) = best_i {
            if best_d < 40.0 * 40.0 {
                locs[i].gLat = round_to(pl.0, 7);
                locs[i].gLng = round_to(pl.1, 7);
            }
        }
    }

    let total_dis = round_to(dist, 3);
    let mut track = Track {
        totalTime: round_to(t_acc, 0) as i64,
        totalDistance: total_dis,
        validDistance: total_dis,
        validTime: round_to(t_acc, 0) as i64,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: steps_acc as i64,
        locations: locs,
        speedPerTenSec: Vec::new(),
        stepsPerTenSec: Vec::new(),
        segments: Vec::new(),
        id_base: 1,
        window_id_base: 1,
        window_t0_ms: 0,
        five_point_id_base: 1,
    };
    super::stats::rebuild(&mut track);
    track
}
