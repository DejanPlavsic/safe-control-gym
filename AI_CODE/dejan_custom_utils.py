import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Node:
    point: np.ndarray
    parent: Optional[int]
    cost: float


def _to_xy(point: Sequence[float]) -> np.ndarray:
    return np.array([float(point[0]), float(point[1])], dtype=float)


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def _sample(bounds: dict, goal_xy: np.ndarray, goal_sample_rate: float) -> np.ndarray:
    if random.random() < goal_sample_rate:
        return goal_xy.copy()
    return np.array([
        random.uniform(bounds["x"][0], bounds["x"][1]),
        random.uniform(bounds["y"][0], bounds["y"][1]),
    ], dtype=float)


def _nearest(nodes: List[Node], point: np.ndarray) -> int:
    dists = [_dist(node.point, point) for node in nodes]
    return int(np.argmin(dists))


def _steer(from_point: np.ndarray, to_point: np.ndarray, step_size: float) -> np.ndarray:
    delta = to_point - from_point
    length = np.linalg.norm(delta)
    if length <= step_size:
        return to_point.copy()
    return from_point + (delta / length) * step_size


def _point_in_collision(point_xy: np.ndarray,
                        obstacles_xy: Sequence[np.ndarray],
                        inflated_radius: float,
                        bounds: dict) -> bool:
    if not (bounds["x"][0] <= point_xy[0] <= bounds["x"][1] and bounds["y"][0] <= point_xy[1] <= bounds["y"][1]):
        return True
    for obs_xy in obstacles_xy:
        if _dist(point_xy, obs_xy) <= inflated_radius:
            return True
    return False


def collision_free_segment(p1_xy: np.ndarray,
                           p2_xy: np.ndarray,
                           obstacles_xy: Sequence[np.ndarray],
                           inflated_radius: float,
                           bounds: dict,
                           resolution: float = 0.05) -> bool:
    seg_len = _dist(p1_xy, p2_xy)
    num = max(2, int(math.ceil(seg_len / resolution)) + 1)
    for alpha in np.linspace(0.0, 1.0, num):
        pt = (1.0 - alpha) * p1_xy + alpha * p2_xy
        if _point_in_collision(pt, obstacles_xy, inflated_radius, bounds):
            return False
    return True


def _near(nodes: List[Node], point: np.ndarray, radius: float) -> List[int]:
    return [idx for idx, node in enumerate(nodes) if _dist(node.point, point) <= radius]


def _extract_path(nodes: List[Node], goal_index: int) -> List[np.ndarray]:
    path = []
    idx = goal_index
    while idx is not None:
        path.append(nodes[idx].point.copy())
        idx = nodes[idx].parent
    path.reverse()
    return path


def rrt_star(start_xy: Sequence[float],
             goal_xy: Sequence[float],
             obstacles_xy: Sequence[Sequence[float]],
             bounds: dict,
             step_size: float = 0.25,
             search_radius: float = 0.6,
             goal_tolerance: float = 0.2,
             max_iter: int = 2500,
             goal_sample_rate: float = 0.12,
             inflated_radius: float = 0.35,
             rng_seed: int = 0) -> List[np.ndarray]:
    random.seed(rng_seed)
    np.random.seed(rng_seed)

    start_xy = _to_xy(start_xy)
    goal_xy = _to_xy(goal_xy)
    obstacles_xy = [_to_xy(obs) for obs in obstacles_xy]

    if _point_in_collision(start_xy, obstacles_xy, inflated_radius, bounds):
        raise ValueError("RRT*: start lies inside an inflated obstacle or outside bounds.")
    if _point_in_collision(goal_xy, obstacles_xy, inflated_radius, bounds):
        raise ValueError("RRT*: goal lies inside an inflated obstacle or outside bounds.")

    nodes: List[Node] = [Node(start_xy, None, 0.0)]
    best_goal_index: Optional[int] = None
    best_goal_cost = float("inf")

    for _ in range(max_iter):
        sample = _sample(bounds, goal_xy, goal_sample_rate)
        nearest_idx = _nearest(nodes, sample)
        new_point = _steer(nodes[nearest_idx].point, sample, step_size)

        if not collision_free_segment(nodes[nearest_idx].point, new_point, obstacles_xy, inflated_radius, bounds):
            continue

        near_indices = _near(nodes, new_point, search_radius)
        parent_idx = nearest_idx
        new_cost = nodes[nearest_idx].cost + _dist(nodes[nearest_idx].point, new_point)

        for idx in near_indices:
            cand_cost = nodes[idx].cost + _dist(nodes[idx].point, new_point)
            if cand_cost < new_cost and collision_free_segment(nodes[idx].point, new_point, obstacles_xy, inflated_radius, bounds):
                parent_idx = idx
                new_cost = cand_cost

        nodes.append(Node(new_point, parent_idx, new_cost))
        new_idx = len(nodes) - 1

        for idx in near_indices:
            rewired_cost = nodes[new_idx].cost + _dist(nodes[new_idx].point, nodes[idx].point)
            if rewired_cost < nodes[idx].cost and collision_free_segment(nodes[new_idx].point, nodes[idx].point, obstacles_xy, inflated_radius, bounds):
                nodes[idx].parent = new_idx
                nodes[idx].cost = rewired_cost

        dist_to_goal = _dist(new_point, goal_xy)
        if dist_to_goal <= goal_tolerance and collision_free_segment(new_point, goal_xy, obstacles_xy, inflated_radius, bounds):
            goal_cost = nodes[new_idx].cost + dist_to_goal
            if goal_cost < best_goal_cost:
                nodes.append(Node(goal_xy.copy(), new_idx, goal_cost))
                best_goal_index = len(nodes) - 1
                best_goal_cost = goal_cost

    if best_goal_index is None:
        if collision_free_segment(start_xy, goal_xy, obstacles_xy, inflated_radius, bounds):
            return [start_xy, goal_xy]
        raise RuntimeError("RRT*: failed to find a path within the iteration budget.")

    return _extract_path(nodes, best_goal_index)


def shortcut_path(path_xy: Sequence[Sequence[float]],
                  obstacles_xy: Sequence[Sequence[float]],
                  inflated_radius: float,
                  bounds: dict) -> List[np.ndarray]:
    if len(path_xy) <= 2:
        return [np.array(p, dtype=float) for p in path_xy]

    path = [np.array(p, dtype=float) for p in path_xy]
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if collision_free_segment(path[i], path[j], [_to_xy(o) for o in obstacles_xy], inflated_radius, bounds):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def resample_polyline_with_z(path_xyz: Sequence[Sequence[float]], spacing: float = 0.08) -> np.ndarray:
    path = np.array(path_xyz, dtype=float)
    if len(path) == 1:
        return path.copy()

    pts = [path[0]]
    for idx in range(len(path) - 1):
        p0 = path[idx]
        p1 = path[idx + 1]
        seg = p1 - p0
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-9:
            continue
        num = max(1, int(math.ceil(seg_len / spacing)))
        for k in range(1, num + 1):
            alpha = k / num
            pts.append((1.0 - alpha) * p0 + alpha * p1)
    return np.array(pts, dtype=float)


def build_gate_targets(gates: Sequence[Sequence[float]],
                       start_xyz: Sequence[float],
                       final_xyz: Sequence[float],
                       approach_offset: float = 0.35,
                       exit_offset: float = 0.35) -> List[np.ndarray]:
    targets: List[np.ndarray] = [np.array(start_xyz, dtype=float)]
    prev_xy = np.array(start_xyz[:2], dtype=float)

    for gate in gates:
        gx, gy, gz, _r, _p, yaw, _gate_type = gate
        center = np.array([gx, gy, gz], dtype=float)
        gate_dir = np.array([math.cos(float(yaw)), math.sin(float(yaw))], dtype=float)
        if np.linalg.norm(gate_dir) < 1e-9:
            gate_dir = np.array([1.0, 0.0], dtype=float)

        # Choose the side that faces the previous waypoint to encourage a clean pass-through.
        if np.dot(prev_xy - center[:2], gate_dir) >= 0.0:
            approach_xy = center[:2] + approach_offset * gate_dir
            exit_xy = center[:2] - exit_offset * gate_dir
        else:
            approach_xy = center[:2] - approach_offset * gate_dir
            exit_xy = center[:2] + exit_offset * gate_dir

        approach = np.array([approach_xy[0], approach_xy[1], gz], dtype=float)
        exit_pt = np.array([exit_xy[0], exit_xy[1], gz], dtype=float)
        targets.extend([approach, center, exit_pt])
        prev_xy = exit_pt[:2]

    targets.append(np.array(final_xyz, dtype=float))
    return targets


def plan_full_mission(start_xyz: Sequence[float],
                      gates: Sequence[Sequence[float]],
                      obstacles: Sequence[Sequence[float]],
                      final_xyz: Sequence[float],
                      bounds: dict,
                      inflated_radius: float = 0.35,
                      step_size: float = 0.25,
                      search_radius: float = 0.6,
                      max_iter: int = 2500,
                      rng_seed: int = 7) -> np.ndarray:
    targets = build_gate_targets(gates, start_xyz, final_xyz)
    obstacles_xy = [np.array([obs[0], obs[1]], dtype=float) for obs in obstacles]

    mission_pts: List[np.ndarray] = [np.array(start_xyz, dtype=float)]
    for seg_id in range(len(targets) - 1):
        p0 = np.array(targets[seg_id], dtype=float)
        p1 = np.array(targets[seg_id + 1], dtype=float)

        path_xy = rrt_star(
            p0[:2],
            p1[:2],
            obstacles_xy,
            bounds=bounds,
            step_size=step_size,
            search_radius=search_radius,
            goal_tolerance=max(0.18, step_size),
            max_iter=max_iter,
            inflated_radius=inflated_radius,
            rng_seed=rng_seed + seg_id,
        )
        path_xy = shortcut_path(path_xy, obstacles_xy, inflated_radius, bounds)

        path_xyz = []
        if len(path_xy) == 1:
            path_xyz = [np.array([path_xy[0][0], path_xy[0][1], p0[2]], dtype=float)]
        else:
            total_len = sum(_dist(path_xy[i], path_xy[i + 1]) for i in range(len(path_xy) - 1))
            walked = 0.0
            for i, pt_xy in enumerate(path_xy):
                if i > 0:
                    walked += _dist(path_xy[i - 1], path_xy[i])
                alpha = 0.0 if total_len < 1e-9 else walked / total_len
                z = (1.0 - alpha) * p0[2] + alpha * p1[2]
                path_xyz.append(np.array([pt_xy[0], pt_xy[1], z], dtype=float))

        if seg_id > 0:
            path_xyz = path_xyz[1:]
        mission_pts.extend(path_xyz)

    return np.array(mission_pts, dtype=float)
