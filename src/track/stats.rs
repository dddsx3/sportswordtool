//! Derived statistics have one source: the final cumulative samples.
use super::model::{GenPoint, Segment, TenWindow, Track};

/// Interpolate cumulative counters at a fractional time boundary (seconds).
/// Rounding cumulative steps (rather than each interval) preserves the exact final step count.
pub fn counters_at_f64(locs: &[GenPoint], time: f64) -> (f64, f64) {
    let Some(first) = locs.first() else {
        return (0.0, 0.0);
    };
    if time <= first.totalTime as f64 {
        return (first.totalDis, first.steps as f64);
    }
    for pair in locs.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        if b.totalTime as f64 >= time && b.totalTime > a.totalTime {
            let k = (time - a.totalTime as f64) / (b.totalTime - a.totalTime) as f64;
            return (
                a.totalDis + (b.totalDis - a.totalDis) * k,
                a.steps as f64 + (b.steps - a.steps) as f64 * k,
            );
        }
    }
    let last = locs.last().unwrap();
    (last.totalDis, last.steps as f64)
}

pub fn counters_at(locs: &[GenPoint], time: i64) -> (f64, i64) {
    let (d, s) = counters_at_f64(locs, time as f64);
    (d, s.round() as i64)
}

/// NEUQ 真人窗口形态（33 条实测）：中位 10.0s，9.3-12s 为主，停顿合并出
/// 最长 36s；窗口数 = 时长/10 上下。这里用「10s 基准 + 小抖动 + 偶发停顿合并」
/// 生成毫秒级窗口边界，末窗吸收余量。
fn window_boundaries(total_time_s: i64, seed: u64) -> (i64, Vec<i64>) {
    use super::geom::Rng;
    let mut rng = Rng::new(seed);
    let total_ms = total_time_s * 1000;
    // 审计校准：真人窗口几乎精确 10s（p5=9.986, p95=10.014），罕见
    // 9.3-12s 与 15-36s（停顿合并）变体
    let mut acc = (rng.uniform(1500.0, 3000.0)) as i64;
    let t0 = acc.min((total_ms - 1500).max(0));
    acc = t0;
    let mut out = Vec::new();
    while acc < total_ms {
        // 真机计时器窗口精确 10.00s + 毫秒级漂移（真人 18-56 unique 值）
        acc += 10_000 + (rng.gauss(0.0, 80.0) as i64);
        if acc > total_ms - 1500 {
            acc = total_ms;
        }
        out.push(acc);
        if acc >= total_ms {
            break;
        }
    }
    if out.last() != Some(&total_ms) {
        if let Some(last) = out.last_mut() {
            *last = total_ms;
        } else {
            out.push(total_ms);
        }
    }
    (t0, out)
}

/// 设备语义重derive。审计校准要点：
/// - speed 字段 = 配速 min/km = 16.6667/avgSpeed（真人残差 std 0.02，
///   corr(speed, avgSpeed) = -0.96；"毛刺"即 avgSpeed≈0.01 的除法爆炸）；
/// - type=-1 失定位点 speed=0；
/// - totalDis 累计用与真人一致的恒定球面比（d(totalDis)/haversine=0.9989
///   恒定，无逐段噪声）；
/// - radius: 每跑设备常数（1-6 个 unique 值），部分跑有冷启动收敛段
///   （30-56m -> 3.79 型），15% 跑为变化型（std 2.8-7.0）；
/// - 冷启动：p0.totalDis≈1.6m、p0.bdD=0、p0.gainTime-flag≈-0.2s；
/// - bdS ≈ avgSpeed(m/s)（39% 恰等，其余 0.9-1.1 倍）；
/// - 首点 avgSpeed 为杂乱小值。
pub fn rederive_from_coordinates(track: &mut Track, rng: &mut super::geom::Rng) {
    use super::geom::round_to;
    let Some(_first) = track.locations.first() else {
        return;
    };
    // 真机原生 GCJ：累计距离走与序列化一致的 GCJ 折线
    let mut cum = 0.0f64;
    let mut prev: Option<(f64, f64)> = None;
    let n = track.locations.len();
    let mut prev_t = 0i64;
    for p in track.locations.iter_mut() {
        let (glat, glng) = crate::track::wire::bd09_to_gcj02(p.gLat, p.gLng);
        if let Some((pla, plo)) = prev {
            // 局地欧氏（cos(lat) 修正）：真机 d(totalDis)/haversine 恒定
            // （0.9989，R≈6364km 语义），途中可有缓慢漂移但无逐段噪声
            let coslat = ((glat + pla) * 0.5).to_radians().cos();
            let d = ((glat - pla) * 111_132.0).hypot((glng - plo) * 111_320.0 * coslat);
            cum += d * 0.9989;
        }
        p.totalDis = cum;
        p.validDis = cum;
        prev = Some((glat, glng));
    }
    // 逐点 avgSpeed/speed(配速)
    let mut prev_t = 0i64;
    let mut prev_d = 0.0f64;
    // avgSpeed = 精确逐点位移速度（终审：raw exact r=1.000）。
    // speed = 16.6667/avgSpeed_raw（未舍入）——真人精确成立（残差 std 0.02，
    // corr -0.96），除法爆炸值（停走毛刺 av 0.002-0.8 -> speed 21-8333）即毛刺形态。
    for (i, p) in track.locations.iter_mut().enumerate() {
        // 真机停走堆叠点（毛刺链）：totalTime 停在同一秒 -> 计时暂停，
        // totalDis/validDis 冻结；avgSpeed 为设备 1s 窗瞬时值（0.002-0.8），
        // speed=16.6667/av_raw 除法爆炸（21-8333）。真人 12d342f6 idx59-62。
        if p.totalTime == prev_t && i > 1 && p.ptype != 5 {
            p.totalDis = prev_d;
            p.validDis = prev_d;
            // 真人毛刺 av 偏高段为主（0.3-0.8），av<0.15 稀少 -> 70/30 混合
            let av_raw = if rng.random() < 0.70 {
                rng.uniform(0.30, 0.80)
            } else {
                rng.uniform(0.002, 0.30)
            };
            p.avgSpeed = round_to(av_raw, 2);
            p.speed = if p.ptype == -1 {
                0.0
            } else {
                round_to((16.6667 / av_raw).min(9000.0), 2)
            };
            continue; // prev_t/prev_d 不更新（同秒堆叠）
        }
        let dt = (p.totalTime - prev_t).max(1) as f64;
        let inst = (p.totalDis - prev_d) / dt;
        if i <= 1 || p.ptype == 5 {
            prev_t = p.totalTime;
            prev_d = p.totalDis;
            continue;
        }
        p.avgSpeed = round_to(inst, 2);
        // speed = 配速 min/km；失定位点 speed=0
        p.speed = if p.ptype == -1 {
            0.0
        } else {
            round_to((16.6667 / inst.max(0.002)).min(9000.0), 2)
        };
        prev_t = p.totalTime;
        prev_d = p.totalDis;
    }
    // radius: 底值用真机设备常数 3.7900925（本校主流指纹）；
    // 75% 恒定（1-2 unique），25% 冷启动收敛 + 点 5 后小幅游走
    // （真机游走型 std≤1.54、≤24 个取值）
    let style: f64 = rng.uniform(0.0, 1.0);
    let radius_base = 3.7900925;
    let cold_start = style < 0.25;
    let wandering = !cold_start && style < 0.50;
    let n_cold = (rng.uniform(3.0, 7.0)) as usize;
    let mut rad_var = radius_base;
    for (i, p) in track.locations.iter_mut().enumerate() {
        p.radius = if cold_start && i < n_cold {
            // 真人冷启动收敛差值无规律（盲查 0.02-1.49）：指数收敛+随机抖动
            let f = i as f64 / n_cold.max(1) as f64;
            let base = 56.0 * (1.0 - f) + radius_base * f;
            round_to((base + rng.gauss(0.0, 2.5)).max(radius_base), 6)
        } else if wandering {
            rad_var = (rad_var + rng.gauss(0.0, 0.55)).clamp(3.0, 5.2);
            round_to(rad_var, 6)
        } else {
            radius_base
        };
    }
    // bdS = 滞后的 SDK 速度（真机与当前 avgSpeed 相关 ≤0.76，滞后 1-2 拍）
    let mut bd_prev = 1.2f64;
    for p in track.locations.iter_mut() {
        let cur = p.avgSpeed;
        // 真人 bdS 场最大 2.24-3.63 m/s（盲查 12/12 分离）：钳到 3.6
        p.bdS = round_to((bd_prev * rng.uniform(0.96, 1.04)).max(0.0).min(3.55), 2);
        bd_prev = bd_prev * 0.4 + cur * 0.6;
    }
    // 首点冷启动语义（真机 p0.totalDis 中位 1.6m 且 validDis==totalDis）
    if n >= 2 {
        let p0 = &mut track.locations[0];
        p0.totalDis = round_to(rng.uniform(0.5, 3.0), 4);
        p0.validDis = p0.totalDis;
        p0.bdD = 0.0;
        p0.speed = round_to((16.6667 / p0.avgSpeed.max(0.05)).min(4000.0), 2);
    }
    // 哨兵语义保持
    if n >= 3 {
        for k in [0usize, 1usize] {
            track.locations[k].steps = 0;
            if k == 0 {
                track.locations[k].totalTime = 0;
                track.locations[k].validTime = 0;
            }
        }
        track.locations[1].totalDis = 0.0;
        track.locations[1].validDis = 0.0;
        track.locations[1].avgSpeed = 0.0;
        track.locations[1].speed = 0.0;
        track.locations[1].bdS = 0.0;
    }
    // 真机语义: totalDis 恒等于窗口距离之和（真机 segment/speed 管线和
    // == totalDis, 比值 0.9998-1.0000）。按窗口距离和重定标各点累计。
    let win_sum: f64 = track.speedPerTenSec.iter().map(|w| w.value).sum();
    if win_sum > 1.0 {
        let k = win_sum / cum;
        for p in track.locations.iter_mut() {
            p.totalDis *= k;
            p.validDis *= k;
        }
    }
    if let Some(last) = track.locations.last_mut() {
        track.totalDistance = last.totalDis;
        track.validDistance = last.validDis;
    }
    // 2km 停止语义: 坐标累计可能比路线规划长 10-15%（路弯+抖动），
    // 统一重定标到真机 2km 停止窗口 [2000.1, 2011.8]
    if let Some(last) = track.locations.last_mut() {
        if last.totalDis > 1800.0 && last.totalDis < 2500.0 {
            let target = 2000.0 + rng.uniform(0.1, 11.8);
            let k = target / last.totalDis;
            for p in track.locations.iter_mut() {
                p.totalDis *= k;
                p.validDis *= k;
            }
            track.totalDistance = track.locations.last().unwrap().totalDis;
            track.validDistance = track.totalDistance;
        }
    }
}

pub fn rebuild(track: &mut Track) {
    let Some(last) = track.locations.last() else {
        return;
    };
    track.totalTime = last.totalTime;
    track.totalDistance = last.totalDis;
    track.totalSteps = last.steps;
    track.validDistance = last.validDis;
    track.validTime = last.validTime;
    track.speedPerTenSec.clear();
    track.stepsPerTenSec.clear();
    track.segments.clear();

    let (t0, boundaries) = window_boundaries(track.totalTime, track.startTime as u64);
    track.window_t0_ms = t0;
    // 窗口距离从「实际上传的含噪坐标」累计（真机管线语义：派生字段由
    // 设备自己上报的点计算，与轨迹天然一致），步数沿用累计计数器插值。
    let mut coord_cum: Vec<f64> = Vec::with_capacity(track.locations.len());
    let mut c = 0.0f64;
    let mut prev_pt: Option<(f64, f64)> = None;
    for p in &track.locations {
        if let Some((pla, plo)) = prev_pt {
            c += ((p.gLat - pla) * 111_000.0).hypot((p.gLng - plo) * 85_000.0);
        }
        coord_cum.push(c);
        prev_pt = Some((p.gLat, p.gLng));
    }
    let coord_at = |locs: &[GenPoint], cum: &[f64], time: f64| -> f64 {
        let Some(first) = locs.first() else {
            return 0.0;
        };
        if time <= first.totalTime as f64 {
            return cum.first().copied().unwrap_or(0.0);
        }
        for (k, pair) in locs.windows(2).enumerate() {
            let (a, b) = (&pair[0], &pair[1]);
            if b.totalTime as f64 >= time && b.totalTime > a.totalTime {
                let f = (time - a.totalTime as f64) / (b.totalTime - a.totalTime) as f64;
                return cum[k] + (cum[k + 1] - cum[k]) * f;
            }
        }
        *cum.last().unwrap_or(&0.0)
    };
    let mut previous = (0.0f64, 0.0f64);
    let mut prev_coord = 0.0f64;
    // 真人窗口/段距离和 ≈ 0.9956×totalDis（略少）——每跑常数因子
    let window_factor =
        super::geom::Rng::new(track.startTime as u64 ^ 0x5E_ED_5E_ED).uniform(0.9930, 0.9980);
    let mut prev_ms = t0;
    let mut prev_seg_s = 0i64;
    let flag_sec = ((track.startTime as f64) / 1000.0).floor() as i64;
    // 真人首段 15-18s（覆盖启动段+首窗），其余段与窗口边界一致
    let mut first_seg_hi = 13 + (super::geom::Rng::new(track.startTime as u64 ^ 0xF1_57).uniform(0.0, 5.0)) as i64;
    for &hi_ms in &boundaries {
        let current = counters_at_f64(&track.locations, hi_ms as f64 / 1000.0);
        let coord_now = coord_at(&track.locations, &coord_cum, hi_ms as f64 / 1000.0);
        // 真机窗口距离来自独立滤波流：与坐标折线相关但绝不逐点相等
        // （真人 0/1256 窗与折线浮点相等），乘性偏差 ~1.5%
        let distance = (coord_now - prev_coord) * window_factor;
        prev_coord = coord_now;
        let steps = current.1 - previous.1;
        track.speedPerTenSec.push(TenWindow {
            time: hi_ms - prev_ms,
            value: distance,
        });
        track.stepsPerTenSec.push(TenWindow {
            time: hi_ms - prev_ms,
            value: steps,
        });
        // segment：真人与窗口一一对应，边界为绝对 epoch 秒
        // （真人 seg.startTime = flag 对齐秒 ±2s，首段 15-18s）
        let mut seg_hi_s = ((hi_ms + 500) / 1000).max(prev_seg_s + 1);
        if prev_seg_s == 0 && first_seg_hi > seg_hi_s {
            seg_hi_s = first_seg_hi;
        }
        first_seg_hi = 0;
        // 顺延不得越过总时长（短轨迹 cap；totalTime 单位为秒）
        let total_s = track.locations.last().map(|p| p.totalTime).unwrap_or(0);
        seg_hi_s = seg_hi_s.min(total_s).max(prev_seg_s + 1);
        let seg_lo_s = prev_seg_s;
        let seg_dur_s = (seg_hi_s - seg_lo_s).max(1);
        let seg_dist = coord_at(&track.locations, &coord_cum, seg_hi_s as f64)
            - coord_at(&track.locations, &coord_cum, seg_lo_s as f64);
        let seg_cur = counters_at_f64(&track.locations, seg_hi_s as f64);
        let seg_prev = counters_at_f64(&track.locations, seg_lo_s as f64);
        track.segments.push(Segment {
            totalTime: seg_dur_s,
            distance: seg_dist,
            startTime: (flag_sec + seg_lo_s) * 1000,
            endTime: (flag_sec + seg_hi_s) * 1000,
            avgSpeed: if seg_dist > 0.5 {
                ((seg_dur_s as f64 / 60.0) / (seg_dist / 1000.0)).min(40.0)
            } else {
                0.0
            },
            avgStep: ((seg_cur.1 - seg_prev.1) * 60.0 / seg_dur_s as f64).round() as i64,
            state: 0,
        });
        previous = current;
        prev_ms = hi_ms;
        prev_seg_s = seg_hi_s;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn final_samples_drive_totals_and_partial_windows() {
        for duration in [1, 9, 10, 11, 59, 60, 61, 180, 603] {
            let mut t = crate::track::generator::build(
                1200.0,
                600,
                7,
                (39.9, 116.4),
                0,
                &[(39.9, 116.4), (39.901, 116.401)],
            );
            let mut a = t.locations[0].clone();
            a.totalTime = 0;
            a.totalDis = 0.0;
            a.steps = 0;
            let mut b = a.clone();
            // 窗口距离从坐标累计：给 b 一个 ~1km 的真实位移
            b.gLat = a.gLat + 0.009;
            b.totalTime = duration;
            b.totalDis = 1000.0;
            b.steps = 1201;
            b.validTime = duration - 1;
            b.validDis = 999.0;
            t.locations = vec![a, b];
            rebuild(&mut t);
            assert_eq!(t.totalTime, duration);
            assert_eq!(t.validTime, duration - 1);
            assert_eq!(t.validDistance, 999.0);
            assert_eq!(
                t.stepsPerTenSec
                    .iter()
                    .map(|w| w.value)
                    .sum::<f64>()
                    .round() as i64,
                1201
            );
            // 坐标累计与标称距离允许亚米级差异（真机同样存在），
            // 窗口值含 ±1.5% 滤波流偏差
            assert!(
                (t.speedPerTenSec.iter().map(|w| w.value).sum::<f64>() - 1000.0).abs()
                    < 1000.0 * 0.05,
                "sum={}",
                t.speedPerTenSec.iter().map(|w| w.value).sum::<f64>()
            );
            assert_eq!(
                t.speedPerTenSec.iter().map(|w| w.time).sum::<i64>(),
                duration * 1000 - t.window_t0_ms
            );
            // segment 边界为绝对 epoch（flag 对齐秒起算）
            let flag_sec = ((t.startTime as f64) / 1000.0).floor() as i64;
            assert_eq!(t.segments[0].startTime, flag_sec * 1000);
            assert_eq!(
                t.segments.last().unwrap().endTime,
                flag_sec * 1000 + duration * 1000
            );
            assert_eq!(
                t.segments
                    .iter()
                    .map(|s| s.endTime - s.startTime)
                    .sum::<i64>(),
                duration * 1000
            );
        }
    }
}
