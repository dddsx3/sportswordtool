//! 轨迹后处理：
//! 收尾减速（保留原有结束时间）、起终点哨兵
//! （索引1 type=5 全零 / 末点 type=6 / 首点 type∈{0,7}）、后半段 type=1/2 散布、
//! 索引2 avgSpeed 边界修正、首点速度取首个非零值。

use super::geom::{round_to, Rng};
use super::model::GenPoint;

pub fn apply_post_fixes(locs: &mut [GenPoint], rng: &mut Rng, start_ms: i64) {
    // 仅调整末点速度；不要使末点越过总时长或早于尾部异常点。
    if locs.len() >= 4 {
        let ci = locs.len() - 1;
        if ci > 1 {
            // 收尾减速仍须落在有效配速窗口内（2.0-2.6 m/s ≈ 7.2-9.4 km/h）
            let stop_ms = round_to(rng.uniform(2.0, 2.6), 4);
            locs[ci].speed = round_to(stop_ms * 3.6, 4);
            locs[ci].avgSpeed = stop_ms;
            locs[ci].bdS = round_to(rng.uniform(0.05, 0.15), 3);
            // totalDis 保持（App 过滤条件 totalDis>0）
        }
    }
    if locs.len() > 2 {
        // 起点哨兵（type=5→setStartPoint）：全零累计 + state=1
        let sp = &mut locs[1];
        sp.ptype = 5;
        sp.state = 1;
        sp.locType = 1;
        sp.radius = round_to(rng.uniform(2.0, 3.5), 2);
        sp.bdD = 0.0;
        sp.speed = 0.0;
        sp.avgSpeed = 0.0;
        sp.bdS = 0.0;
        sp.totalDis = 0.0;
        sp.validDis = 0.0;
        sp.totalTime = 0;
        sp.validTime = 0;
        sp.steps = 0;
        // 终点哨兵
        let last = locs.len() - 1;
        let e = &mut locs[last];
        e.ptype = 6;
        e.locType = 1;
        e.radius = round_to(rng.uniform(1.5, 3.0), 2);
    }
    // 边界修正①：索引2 avgSpeed 由 rederive EMA 覆盖，此处仅确保非零
    if locs.len() > 3 && locs[2].ptype != -1 && locs[2].avgSpeed < 0.1 {
        locs[2].avgSpeed = 1.5;
    }
    // 首点 = 起点：totalTime=0、type∈{0,7}、速度取首个非零值
    if !locs.is_empty() {
        locs[0].totalTime = 0;
        locs[0].validTime = 0;
        locs[0].totalDis = 0.0;
        locs[0].validDis = 0.0;
        locs[0].steps = 0;
        locs[0].state = 1;
        locs[0].ptype = rng.choice(&[0, 0, 0, 7]);
        locs[0].locType = 1;
        locs[0].radius = round_to(rng.uniform(1.5, 3.0), 2);
        for j in 1..locs.len() {
            if locs[j].avgSpeed > 0.0 {
                locs[0].avgSpeed = locs[j].avgSpeed;
                locs[0].bdS = locs[j].bdS;
                break;
            }
        }
        if locs[0].speed == 0.0 {
            locs[0].speed = round_to(rng.uniform(7.0, 13.0), 4);
        }
    }
    // 真机 gainTime 是打点时钟：漂移恒定，偶发 ±3-17s 校正
    // （ΔgainTime ≡ ΔtotalTime 占 82-99%）
    let n_pts = locs.len();
    let mut drift_s = rng.uniform(0.5, 6.5);
    for (idx, p) in locs.iter_mut().enumerate() {
        if idx > 0 && rng.random() < 0.03 {
            drift_s = (drift_s + rng.uniform(-17.0, 17.0)).clamp(0.0, 37.0);
        }
        let remain = n_pts - idx;
        if remain <= 3 {
            drift_s *= 0.4;
        }
        p.gainTimeMs = start_ms + p.totalTime * 1000 + (drift_s * 1000.0) as i64;
        p.gainTime = super::geom::fmt_gain_time(p.gainTimeMs);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn post_processing_preserves_exception_type_and_speed_limits() {
        let track = crate::track::generator::build(
            1200.0,
            600,
            7,
            (39.9, 116.4),
            0,
            &[(39.9, 116.4), (39.901, 116.401)],
        );
        for seed in 0..64 {
            let mut points = track.locations.clone();
            let late = (points.len() as f64 * 0.8) as usize;
            for index in [2, late] {
                points[index].ptype = -1;
                points[index].avgSpeed = 1.0;
                points[index].speed = 3.6;
            }
            apply_post_fixes(&mut points, &mut Rng::new(seed), 0);
            for index in [2, late] {
                assert_eq!(points[index].ptype, -1);
                assert_eq!(points[index].avgSpeed, 1.0);
                assert_eq!(points[index].speed, 3.6);
            }
        }
    }
}
