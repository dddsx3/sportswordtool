//! 闭环路线生成：方向锚点搜索 + 已通过 physical edge（way_id）重复权值惩罚。
//!
//! 贪心游走 + 目标引导 + 回家闭合，配合多次重试择优，抑制折返。

use std::collections::HashMap;

use petgraph::algo::astar;
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use petgraph::Direction;
use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::graph::{angle_diff, bearing, dist, Coord, RoadGraph};

#[derive(Clone, Debug)]
pub struct RouteOptions {
    /// Plan one closed checkpoint circuit; callers repeat it for extra mileage.
    pub repeat_checkpoint_loop: bool,
    /// 最大向心加速度 m/s²（弯道限速）。
    pub a_c_max: f64,
    /// 转弯切角阈值（度）。
    pub turn_threshold_deg: f64,
    /// 最小转弯半径（米）。
    pub min_radius_m: f64,
    /// 重复边惩罚系数（已走 way 长度乘子）。
    pub repeat_penalty: f64,
    /// 判定「到达目标」的半径（米）。
    pub snap_m: f64,
    /// 转弯惩罚（等效米 / 弧度）。
    pub turn_weight_m: f64,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions {
            repeat_checkpoint_loop: false,
            a_c_max: 2.5,
            turn_threshold_deg: 30.0,
            min_radius_m: 6.0,
            repeat_penalty: 4.0,
            snap_m: 30.0,
            turn_weight_m: 70.0,
        }
    }
}

const MAX_ITERS: usize = 4000;
const RETRIES: usize = 4;

/// 按序打卡时对未到打卡点判定圈的避让半径（米）：路径不得进入后续
/// 打卡点的判定范围，否则 App 记为提前打卡（跳跃违规）。
/// 真人 33 条轨迹距打卡点最近 9.5-24.5m 全部成功打卡 → 判定半径 ≈25m；
/// 30m 避让圈留 5m 缓冲，保证未到点最近距离 > 判定半径。
const CHECKPOINT_CLEARANCE_M: f64 = 30.0;

/// 两点最短路径（节点序列，几何长度）。
pub fn shortest_path(g: &RoadGraph, from: NodeIndex, to: NodeIndex) -> Option<Vec<NodeIndex>> {
    let res = astar(
        &g.graph,
        from,
        |n| n == to,
        |e| e.weight().length_m * road_class_factor(&e.weight().highway),
        |_| 0.0,
    );
    res.map(|(_cost, path)| path)
}

/// 主干道偏好因子：真人路线偏爱宽直主干道（turns_per_km 2.4 vs 巷道网格 8+），
/// A* 与游走评分按道路等级加权（trunk/主次干道降权，footway/service 升权）。
pub fn road_class_factor(highway: &str) -> f64 {
    match highway {
        "trunk" | "primary" | "secondary" | "tertiary" => 0.68,
        "primary_link" | "secondary_link" | "tertiary_link" => 0.9,
        "residential" | "unclassified" | "living_street" => 1.0,
        _ => 1.75, // footway / path / service / steps / pedestrian…
    }
}

fn dist_nodes(g: &RoadGraph, a: NodeIndex, b: NodeIndex) -> f64 {
    dist(g.to_m(g.node_coord(a)), g.to_m(g.node_coord(b)))
}

fn shortest_len(g: &RoadGraph, from: NodeIndex, to: NodeIndex) -> f64 {
    shortest_path(g, from, to)
        .map(|p| path_len(g, &p))
        .unwrap_or(f64::INFINITY)
}

fn path_len(g: &RoadGraph, path: &[NodeIndex]) -> f64 {
    path.windows(2).map(|w| dist_nodes(g, w[0], w[1])).sum()
}

/// 一次方向锚点游走，返回节点路径（首尾为 start，闭合）。
///
/// 两阶段：
///   1) 最短路径依次经过所有必经点/打卡点（**保证必达**）；
///   2) 方向锚点游走补足目标距离，再最短路径回家闭合。
///
/// （游走核心在 [`wander_open`]：开放路径，不强制闭合；供打卡单次串联
/// 分支复用为「末端补距、终点留在路网上」的形态。）
fn directional_walk(
    g: &RoadGraph,
    start: NodeIndex,
    must_pass: &[NodeIndex],
    target_len_m: f64,
    anchor_deg: f64,
    rng: &mut StdRng,
    opts: &RouteOptions,
) -> Vec<NodeIndex> {
    let mut path = vec![start];
    let mut used: HashMap<u64, u32> = HashMap::new();
    // 节点重访计数：只惩罚 way_id 无法阻止"绕方块回同一路口"（每条边都只走一次）
    let mut node_visits: HashMap<NodeIndex, u32> = HashMap::new();
    node_visits.insert(start, 1);
    let mut length = 0.0f64;
    let mut current = start;
    let mut prev_dir: Option<f64> = None;

    // —— 阶段一：最短路径依次经过所有必经点（打卡点必达）——
    for &t in must_pass {
        if let Some(seg) = shortest_path(g, current, t) {
            for &n in seg.iter().skip(1) {
                if let Some(e) = g.graph.find_edge(current, n) {
                    let wid = g.graph[e].way_id;
                    *used.entry(wid).or_insert(0) += 1;
                    length += g.graph[e].length_m;
                }
                prev_dir = Some(bearing(
                    g.to_m(g.node_coord(current)),
                    g.to_m(g.node_coord(n)),
                ));
                *node_visits.entry(n).or_insert(0) += 1;
                current = n;
                path.push(n);
            }
        }
    }

    // —— 阶段二：游走补足距离并回家闭合（游走核心 = wander_open）——
    let prev_node = if path.len() >= 2 {
        Some(path[path.len() - 2])
    } else {
        None
    };
    let (ext, _walked) = wander_open(
        g,
        current,
        (target_len_m - length).max(0.0),
        anchor_deg,
        rng,
        opts,
        &used,
        &node_visits,
        prev_node,
        prev_dir,
    );
    path.extend(ext);
    // 回退分支保持旧行为：最短路径回家闭合
    let end = *path.last().unwrap();
    if end != start {
        if let Some(home) = shortest_path(g, end, start) {
            if home.len() > 1 {
                path.extend(home.into_iter().skip(1));
            }
        }
    }
    path
}

/// 开放路径游走（directional_walk 阶段二的共享核心）：
/// 从 `from` 出发沿路网贪心前进，累计 `need_m` 即停——不走回家路、不强制闭合
/// （真机跑步终点不必是起点，真人 33 条无一条严格闭合）。
/// `seed_used`/`seed_visits`：已走边 way_id 次数与已访问节点次数，用于重复惩罚
/// （补距游走不应重踩打卡串联段）。返回（追加节点序列, 游走长度）。
#[allow(clippy::too_many_arguments)]
fn wander_open(
    g: &RoadGraph,
    from: NodeIndex,
    need_m: f64,
    anchor_deg: f64,
    rng: &mut StdRng,
    opts: &RouteOptions,
    seed_used: &HashMap<u64, u32>,
    seed_visits: &HashMap<NodeIndex, u32>,
    mut prev_node: Option<NodeIndex>,
    mut prev_dir: Option<f64>,
) -> (Vec<NodeIndex>, f64) {
    let anchor = anchor_deg.to_radians();
    let mut desired = prev_dir.unwrap_or(anchor);
    let mut used: HashMap<u64, u32> = seed_used.clone();
    let mut node_visits: HashMap<NodeIndex, u32> = seed_visits.clone();
    let mut path: Vec<NodeIndex> = Vec::new();
    let mut length = 0.0f64;
    let mut current = from;

    for _ in 0..MAX_ITERS {
        // 开放路径：达到目标里程即停（终点留在路网上，不绕回起点）
        if length >= need_m {
            break;
        }

        // 候选边
        let mut candidates: Vec<(NodeIndex, f64)> = Vec::new();
        for e in g.graph.edges_directed(current, Direction::Outgoing) {
            let m = e.target();
            if Some(m) == prev_node {
                continue; // 禁止立即折返
            }
            let ed = e.weight();
            let dir_e = bearing(g.to_m(g.node_coord(current)), g.to_m(g.node_coord(m)));
            let used_n = used.get(&ed.way_id).copied().unwrap_or(0) as f64;
            let turn = match prev_dir {
                Some(pd) => opts.turn_weight_m * angle_diff(pd, dir_e),
                None => 0.0,
            };
            let mut score =
                ed.length_m * road_class_factor(&ed.highway) * (1.0 + opts.repeat_penalty * used_n)
                    + turn;
            // 方向偏置：对齐 desired 减分（更优）
            score -= ed.length_m * 0.6 * (dir_e - desired).cos();
            // 节点重访惩罚：绕圈回同一路口是本项专门阻止的形态
            let visit_n = node_visits.get(&m).copied().unwrap_or(0) as f64;
            if visit_n > 0.0 {
                score *= 1.0 + 12.0 * visit_n;
            }
            candidates.push((m, score));
        }
        if candidates.is_empty() {
            // 死胡同：允许折返一次
            if let Some(prev) = prev_node {
                if g.graph.find_edge(current, prev).is_some() {
                    candidates.push((prev, 1e6));
                }
            }
        }
        if candidates.is_empty() {
            break;
        }
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // 前 3 名 soft-max 加权随机
        let k = candidates.len().min(3);
        let min_score = candidates[0].1;
        let weights: Vec<f64> = candidates[..k]
            .iter()
            .map(|c| (-(c.1 - min_score) / 40.0).exp())
            .collect();
        let total: f64 = weights.iter().sum();
        let mut u: f64 = rand::Rng::gen_range(rng, 0.0..total.max(1e-9));
        let mut pick = 0usize;
        for (i, w) in weights.iter().enumerate() {
            u -= *w;
            if u <= 0.0 {
                pick = i;
                break;
            }
        }
        if pick >= weights.len() {
            pick = weights.len() - 1;
        }
        let next = candidates[pick].0;

        // 移动
        let dir_now = bearing(g.to_m(g.node_coord(current)), g.to_m(g.node_coord(next)));
        if let Some(e) = g.graph.find_edge(current, next) {
            let wid = g.graph[e].way_id;
            *used.entry(wid).or_insert(0) += 1;
            length += g.graph[e].length_m;
        }
        *node_visits.entry(next).or_insert(0) += 1;
        desired = dir_now; // 继续沿当前朝向探索
        prev_dir = Some(dir_now);
        prev_node = Some(current);
        current = next;
        path.push(next);
    }
    (path, length)
}

/// 点 p 到线段 ab 的最近距离（米制坐标）。
fn seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let ab2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if ab2 <= 1e-12 {
        0.0
    } else {
        ((ap[0] * ab[0] + ap[1] * ab[1]) / ab2).clamp(0.0, 1.0)
    };
    let c = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    dist(p, c)
}

fn repeat_cost(g: &RoadGraph, path: &[NodeIndex]) -> (u32, f64) {
    let mut used: HashMap<u64, u32> = HashMap::new();
    let mut repeats = 0u32;
    for w in path.windows(2) {
        if let Some(e) = g.graph.find_edge(w[0], w[1]) {
            let wid = g.graph[e].way_id;
            let c = used.entry(wid).or_insert(0);
            if *c > 0 {
                repeats += 1;
            }
            *c += 1;
        }
    }
    (repeats, path_len(g, path))
}

/// GraphHopper `RoundTripRouting` 的 petgraph 移植（Apache-2.0）。
///
/// 算法与结构忠实复刻自 GraphHopper core（版权归 GraphHopper GmbH，Apache-2.0）：
/// - `MultiPointTour`：初始航向随机，填充点数 `min(20, 2 + target/50000)`，
///   段距 = 目标/(点数+1) 带 ±10% 扰动，航向 = 初始 + 360°·i/点数（圆周分布）；
/// - `RoundTripRouting.generateValidPoint`：按 (距离, 航向) 外推坐标后吸附最近道路，
///   失败则距离 ×0.95 重试（默认 3 次）；
/// - `AvoidEdgesWeighting`：已用边权重 ×5（软惩罚），逐段累积，抑制重复与自交。
///
/// 对 GH 的最小扩展（GH 仅支持单起点、无有序途经点）：把有序必经点作为固定
/// 停靠点插入序列，填充点按段（段长占比）生成，全程共享同一 AvoidEdges 集合。
fn roundtrip_gh(
    g: &mut RoadGraph,
    start: NodeIndex,
    must_pass: &[NodeIndex],
    target_len_m: f64,
    seed: u64,
    _opts: &RouteOptions,
) -> Option<Vec<NodeIndex>> {
    use rand::Rng;
    let mut rng = StdRng::seed_from_u64(seed.wrapping_add(0x9E37_79B9_7F4A_7C15));
    let initial_heading: f64 = rng.gen_range(0.0..360.0);
    // GH Params: pointCount = min(20, 2 + distance/50000)；GH 面向 10km+ 车 Tour，
    // 校园尺度按其 API 的 RoundTrip.POINTS 参数语义以 target/150 取点数。
    let point_count = ((2.0 + target_len_m / 150.0) as usize).clamp(4, 20);

    // 固定停靠点: start -> cp1..cpN -> start
    let stops: Vec<NodeIndex> = must_pass
        .iter()
        .copied()
        .chain(std::iter::once(start))
        .collect();
    let mut leg_len: Vec<f64> = Vec::with_capacity(stops.len());
    let mut cur = start;
    for &t in &stops {
        leg_len.push(shortest_len(g, cur, t));
        cur = t;
    }
    let base_len: f64 = leg_len.iter().sum();
    if !base_len.is_finite() {
        return None;
    }
    let extra = target_len_m - base_len;
    if extra <= 30.0 {
        return None; // 目标过短，交回调用方的最短闭环路径
    }

    // 每段填充点数按腿长占比（和 = point_count）
    let mut fill_per_leg: Vec<usize> = Vec::with_capacity(stops.len());
    let mut assigned = 0usize;
    for i in 0..stops.len() {
        let n_i = ((leg_len[i] / base_len) * point_count as f64).round() as usize;
        fill_per_leg.push(n_i);
        assigned += n_i;
    }

    // —— MultiPointTour + generateValidPoint：逐腿生成填充点并吸附 ——
    // segment 距离 = 腿目标/(填充数+1)，±10% 扰动；失败 ×0.95 重试 3 次。
    let mut sequence: Vec<NodeIndex> = vec![start];
    let mut last = g.node_coord(start);
    let mut global_k = 0usize;
    for (i, &stop) in stops.iter().enumerate() {
        let n_i = fill_per_leg[i];
        if n_i > 0 {
            let leg_target = leg_len[i] + extra * (leg_len[i] / base_len);
            let seg_dist = leg_target / (n_i + 1) as f64;
            for _ in 0..n_i {
                let heading = if global_k == 0 {
                    initial_heading
                } else {
                    initial_heading + 360.0 * global_k as f64 / point_count as f64
                };
                // GH: slightlyModifyDistance（±10%）
                let mut d = seg_dist * (0.9 + 0.2 * rng.gen::<f64>());
                let mut snapped = None;
                for _ in 0..3 {
                    let rad = heading.to_radians();
                    let p0 = g.to_m(last);
                    let px = p0[0] + d * rad.sin();
                    let py = p0[1] + d * rad.cos();
                    let coord = g.to_deg([px, py]);
                    match crate::project::add_virtual_nodes(g, &[coord]) {
                        Ok(v) => {
                            snapped = Some(v[0]);
                            break;
                        }
                        Err(_) => d *= 0.95, // GH: 距离收缩重试
                    }
                }
                let vn = snapped?;
                last = g.node_coord(vn);
                sequence.push(vn);
                global_k += 1;
            }
        }
        sequence.push(stop);
        last = g.node_coord(stop);
        let _ = global_k;
    }

    // —— AvoidEdgesWeighting：逐段 A*，已用边 ×5 累积 ——
    let mut avoided: std::collections::HashSet<usize> = Default::default();
    let mut path: Vec<NodeIndex> = vec![start];
    for pair in sequence.windows(2) {
        let res = astar(
            &g.graph,
            pair[0],
            |n| n == pair[1],
            |e| {
                let base_w = e.weight().length_m;
                if avoided.contains(&e.id().index()) {
                    base_w * 15.0
                } else {
                    base_w
                }
            },
            |_| 0.0,
        )?;
        let (_, seg) = res;
        for w in seg.windows(2) {
            if let Some(e) = g.graph.find_edge(w[0], w[1]) {
                avoided.insert(e.index());
            }
        }
        path.extend(seg.into_iter().skip(1));
    }
    if path.len() < 2 || path.last() != Some(&start) {
        return None;
    }
    Some(path)
}

/// 生成闭合路线（节点序列，首尾一致）。
///
/// `must_pass` 为强制必经点（按序）；`anchor_pts` 为软引导点，仅用于方向锚点
/// （不必全部经过，只引导游走朝向，最终由调用方在 <40m 内吸附）。
pub fn plan_loop(
    g: &mut RoadGraph,
    start: NodeIndex,
    must_pass: &[NodeIndex],
    anchor_pts: &[NodeIndex],
    target_len_m: f64,
    seed: u64,
    opts: &RouteOptions,
) -> Result<Vec<NodeIndex>, String> {
    // 方向锚点：指向最远软引导点，否则最远必经点，否则 0（随机多角度重试兜底）。
    let anchor_deg = anchor_pts
        .iter()
        .chain(must_pass.iter())
        .max_by(|a, b| {
            dist_nodes(g, start, **a)
                .partial_cmp(&dist_nodes(g, start, **b))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|far| bearing(g.to_m(g.node_coord(start)), g.to_m(g.node_coord(*far))).to_degrees())
        .unwrap_or(0.0);

    // 预检查：必经点必须依次可达（围栏裁剪可能把路网切成多个连通分量，落在被裁掉
    // 分量上的必经点会静默丢失，破坏「必达」承诺）。不可达直接报错，而非静默跳过。
    let mut cur = start;
    for &t in must_pass {
        if shortest_path(g, cur, t).is_none() {
            return Err(format!(
                "必经点 ({:.6},{:.6}) 不可达：可能落在围栏外或路网不连通",
                g.node_coord(t).lat,
                g.node_coord(t).lon
            ));
        }
        cur = t;
    }

    if opts.repeat_checkpoint_loop {
        // —— 打卡点按序单次串联 + 末端路网补距（开放路径）——
        //
        // 策略（按需求）：非必须不绕圈。App 要求按 1→2→3→… 顺序打卡、不允许
        // 跳跃，因此严格按给定顺序用 A* 最短路径串联（A* 是简单路径，段内无环）。
        // 里程不足时从最后一个打卡点沿路网继续前进直到目标里程
        // （wander_open：节点重访 ×12/次 + way 重复惩罚），终点停在里程达标处
        // ——真机跑步终点不必是起点（真人 33 条无一条严格闭合）。

        let mut path: Vec<NodeIndex> = vec![start];
        let mut cur = start;
        for (i, &t) in must_pass.iter().enumerate() {
            // 未到的后续打卡点（must_pass[i+1..]）判定圈是禁区：按序打卡下，
            // 路径顺路蹭进后续点范围会被 App 记为提前打卡（跳跃违规）。
            // 硬禁（×1e9）：cp3 旁 27m 廊道为割边必经时才退化为通过。
            let future = must_pass.get(i + 1..).unwrap_or(&[]);
            let seg = if future.is_empty() {
                shortest_path(g, cur, t)
            } else {
                astar(
                    &g.graph,
                    cur,
                    |n| n == t,
                    |e| {
                        let base_w = e.weight().length_m * road_class_factor(&e.weight().highway);
                        let (a, b) = g.graph.edge_endpoints(e.id()).unwrap();
                        // 点到线段最近距离：端点贴圈的边同样算入（只查中点会漏掉
                        // 「端点在圈内、中点在圈外」的长边，路径可合法经过距
                        // 打卡点 30m 的节点 → 提前打卡）
                        let blocked = future.iter().any(|&f| {
                            seg_dist(
                                g.to_m(g.node_coord(f)),
                                g.to_m(g.node_coord(a)),
                                g.to_m(g.node_coord(b)),
                            ) < CHECKPOINT_CLEARANCE_M
                        });
                        if blocked {
                            base_w * 1e9
                        } else {
                            base_w
                        }
                    },
                    |_| 0.0,
                )
                .map(|(_, p)| p)
                .or_else(|| shortest_path(g, cur, t))
            }
            .ok_or("打卡点之间道路不连通，无法按序串联")?;
            path.extend(seg.into_iter().skip(1));
            cur = t;
        }
        let base_len = path_len(g, &path);

        // 里程不足 → 末端游走补距（RETRIES 择优：长度贴合优先，其次少重复）
        if base_len < target_len_m {
            let need = target_len_m - base_len;
            // 种子计数：打卡串联段已用 way / 已访节点，游走不再重踩
            let mut seed_used: HashMap<u64, u32> = HashMap::new();
            let mut seed_visits: HashMap<NodeIndex, u32> = HashMap::new();
            for w in path.windows(2) {
                if let Some(e) = g.graph.find_edge(w[0], w[1]) {
                    *seed_used.entry(g.graph[e].way_id).or_insert(0) += 1;
                }
                *seed_visits.entry(w[1]).or_insert(0) += 1;
            }
            let prev_node = if path.len() >= 2 {
                Some(path[path.len() - 2])
            } else {
                None
            };
            let last_dir = if path.len() >= 2 {
                Some(bearing(
                    g.to_m(g.node_coord(path[path.len() - 2])),
                    g.to_m(g.node_coord(cur)),
                ))
            } else {
                None
            };

            let mut best: Option<(Vec<NodeIndex>, f64)> = None;
            let mut best_key: Option<(f64, u32)> = None;
            for i in 0..RETRIES {
                let mut rng = StdRng::seed_from_u64(
                    seed.wrapping_add(0x9E37_79B9_7F4A_7C15u64.wrapping_mul(i as u64 + 1)),
                );
                let (ext, walked) = wander_open(
                    g,
                    cur,
                    need,
                    anchor_deg + i as f64 * 90.0,
                    &mut rng,
                    opts,
                    &seed_used,
                    &seed_visits,
                    prev_node,
                    last_dir,
                );
                if ext.len() < 2
                    || !ext
                        .windows(2)
                        .all(|p| g.graph.find_edge(p[0], p[1]).is_some())
                {
                    continue;
                }
                let mut full = path.clone();
                full.extend(ext.iter().copied());
                let (repeats, _total_len) = repeat_cost(g, &full);
                // 评分：长度贴合优先，其次少重复
                let key = ((walked - need).abs() + repeats as f64 * 60.0, repeats);
                let better = match best_key {
                    Some((bk, _)) => key.0 < bk,
                    None => true,
                };
                if better {
                    best = Some((ext, walked));
                    best_key = Some(key);
                }
            }
            if let Some((ext, _walked)) = best {
                path.extend(ext);
            }
        }
        return Ok(path);
    }

    let mut best: Option<Vec<NodeIndex>> = None;
    let mut best_key: Option<(f64, u32)> = None;
    for i in 0..RETRIES {
        let mut rng = StdRng::seed_from_u64(seed.wrapping_add(i as u64));
        let path = directional_walk(
            g,
            start,
            must_pass,
            target_len_m,
            anchor_deg + i as f64 * 90.0,
            &mut rng,
            opts,
        );
        if path.len() < 2
            || path.last() != Some(&start)
            || !path
                .windows(2)
                .all(|p| g.graph.find_edge(p[0], p[1]).is_some())
        {
            continue;
        }
        let (repeats, len) = repeat_cost(g, &path);
        let len_err = (len - target_len_m).abs();
        // 评分：优先长度贴合，其次少折返
        let key = (len_err + repeats as f64 * 60.0, repeats);
        let better = match best_key {
            Some((bk, _)) => key.0 < bk,
            None => true,
        };
        if better {
            best = Some(path);
            best_key = Some(key);
        }
    }

    let path = best.ok_or("无法生成路线".to_string())?;
    Ok(path)
}

/// 节点路径 → 度系折线坐标（供 UI/平滑使用）。
/// 去环但保留必经点：扫描重复节点，若两次出现之间的子路径长度
/// < `max_loop_len_m` 且**不含任何必经点**，则截断该小环。
/// 与无约束版本的区别：不会删掉打卡点，因此不会破坏必达保证。
pub fn remove_loops_keep_checkpoints(
    g: &RoadGraph,
    path: Vec<NodeIndex>,
    must_nodes: &[NodeIndex],
    max_loop_len_m: f64,
) -> Vec<NodeIndex> {
    if path.len() < 3 {
        return path;
    }
    let must: std::collections::HashSet<NodeIndex> = must_nodes.iter().copied().collect();
    let mut result: Vec<NodeIndex> = Vec::with_capacity(path.len());
    let mut last_pos: HashMap<NodeIndex, usize> = HashMap::new();
    for &node in &path {
        if let Some(&prev_idx) = last_pos.get(&node) {
            let inner = &result[prev_idx + 1..];
            let loop_len: f64 = result[prev_idx..]
                .windows(2)
                .map(|w| dist_nodes(g, w[0], w[1]))
                .sum();
            // 子路径内不得含必经点（除了两端）
            let has_checkpoint = inner.iter().any(|n| must.contains(n));
            if loop_len < max_loop_len_m && !has_checkpoint {
                result.truncate(prev_idx + 1);
                let kept: std::collections::HashSet<NodeIndex> = result.iter().copied().collect();
                let removed: Vec<NodeIndex> = last_pos
                    .keys()
                    .copied()
                    .filter(|n| !kept.contains(n))
                    .collect();
                for n in removed {
                    last_pos.remove(&n);
                }
                continue;
            }
        }
        last_pos.insert(node, result.len());
        result.push(node);
    }
    result
}

/// 移除路径中短于 `max_loop_len_m` 的闭环子路径（"画圈"）。
pub fn remove_small_loops(
    g: &RoadGraph,
    path: Vec<NodeIndex>,
    max_loop_len_m: f64,
) -> Vec<NodeIndex> {
    if path.len() < 3 {
        return path;
    }
    let mut result: Vec<NodeIndex> = Vec::with_capacity(path.len());
    let mut last_pos: HashMap<NodeIndex, usize> = HashMap::new();
    for &node in &path {
        if let Some(&prev_idx) = last_pos.get(&node) {
            let loop_len: f64 = if result.len() > prev_idx + 1 {
                result[prev_idx..]
                    .windows(2)
                    .map(|w| dist_nodes(g, w[0], w[1]))
                    .sum()
            } else {
                0.0
            };
            if loop_len < max_loop_len_m {
                result.truncate(prev_idx + 1);
                let kept: std::collections::HashSet<NodeIndex> = result.iter().copied().collect();
                let removed: Vec<NodeIndex> = last_pos
                    .keys()
                    .copied()
                    .filter(|n| !kept.contains(n))
                    .collect();
                for n in removed {
                    last_pos.remove(&n);
                }
                continue;
            }
        }
        last_pos.insert(node, result.len());
        result.push(node);
    }
    result
}

pub fn path_coords(g: &RoadGraph, path: &[NodeIndex]) -> Vec<Coord> {
    path.iter().map(|&n| g.node_coord(n)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn repeat_loop_requires_a_return_path_even_for_a_single_checkpoint() {
        let mut g = RoadGraph::new();
        let a = g.add_node(Coord::new(116.4, 39.9));
        let b = g.add_node(Coord::new(116.401, 39.9));
        g.compute_anchor();
        let edge = crate::graph::EdgeData {
            way_id: 1,
            oneway: true,
            highway: "footway".into(),
            maxspeed_kmh: None,
            length_m: 85.,
        };
        g.add_edge(a, b, edge.clone());
        let opts = RouteOptions {
            repeat_checkpoint_loop: true,
            ..Default::default()
        };
        assert!(plan_loop(&g, a, &[b], &[], 5000., 1, &opts).is_err());
        assert!(plan_loop(&g, a, &[a], &[], 5000., 1, &opts).is_err());
        g.add_edge(b, a, edge);
        assert_eq!(
            plan_loop(&g, a, &[a], &[], 5000., 1, &opts).unwrap(),
            vec![a, b, a]
        );
    }
}
