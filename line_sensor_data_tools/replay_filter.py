from __future__ import annotations

import argparse
import colorsys
from collections import deque
import json
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from line_sensor_data_tools.common import (
    RECORDING_FORMAT,
    copy_status,
    frame_to_status,
    latest_recording,
    make_line_source,
)
from stretch4_under_base_hazard.line_sensor_source import (
    BinClass,
    LineSensorHits,
    as_range_array,
)
from stretch4_under_base_hazard.pointcloud_io import numpy_to_pointcloud2


FILTER_NAME = 'none'
FILTER_WINDOW = 5
CLUSTER_EPS_M = 0.035


Candidate = tuple[int, str, int, BinClass, np.ndarray]
RunItem = tuple[int, int, BinClass, np.ndarray]
ClusterDebug = tuple[list[Candidate], bool, str, int]
FilterFn = Callable[[Any, dict[str, Any], dict[str, Any]], LineSensorHits]


def _stack_xy(points: list[np.ndarray]) -> np.ndarray:
    if not points:
        return np.zeros((0, 2), dtype=np.float64)
    return np.vstack(points)


def _hits_from_candidates(candidates: list[Candidate]) -> LineSensorHits:
    obstacle_pts = [pt[:2] for _sidx, _name, _bidx, cls, pt in candidates if cls == BinClass.OBSTACLE]
    small_drop_pts = [pt[:2] for _sidx, _name, _bidx, cls, pt in candidates if cls == BinClass.SMALL_DROP]
    return LineSensorHits(
        obstacle_xy=_stack_xy(obstacle_pts),
        small_drop_xy=_stack_xy(small_drop_pts),
        raw_obstacle_xy=_stack_xy(obstacle_pts),
        raw_small_drop_xy=_stack_xy(small_drop_pts),
    )


def _extract_candidates(source: Any, status: dict[str, Any]) -> list[Candidate]:
    cfg = source.config
    candidates: list[Candidate] = []

    for sensor_idx, sensor_name in enumerate(source.sensor_names):
        sensor_status = status.get(sensor_name, {})
        if not isinstance(sensor_status, dict):
            continue
        ranges = as_range_array(sensor_status.get('ranges'))
        if ranges.size == 0:
            continue
        if source.apply_tare is not None:
            ranges = source.apply_tare(ranges, sensor_name)

        projected = source._project_sensor_bins(sensor_idx, ranges)
        for bin_idx in range(len(ranges)):
            if (
                not np.isfinite(ranges[bin_idx])
                or ranges[bin_idx] <= 0.0
                or ranges[bin_idx] >= cfg.max_range
            ):
                continue
            pt = projected[bin_idx]
            r2 = pt[0] * pt[0] + pt[1] * pt[1]
            if r2 > cfg.line_sensor_radius_m * cfg.line_sensor_radius_m:
                continue
            cls = source._classify_bin(pt[2])
            if cls in (BinClass.OBSTACLE, BinClass.SMALL_DROP):
                candidates.append((sensor_idx, sensor_name, bin_idx, cls, pt))

    return candidates


def filter_none(source: Any, status: dict[str, Any], _state: dict[str, Any]) -> LineSensorHits:
    return _hits_from_candidates(_extract_candidates(source, status))


def filter_moving_average(source: Any, status: dict[str, Any], state: dict[str, Any]) -> LineSensorHits:
    out = copy_status(status)
    history = state.setdefault('history', {})
    for sensor_name, sensor_status in out.items():
        ranges = np.asarray(sensor_status.get('ranges', []), dtype=np.float64)
        sensor_history = history.setdefault(sensor_name, deque(maxlen=FILTER_WINDOW))
        sensor_history.append(ranges)
        if len(sensor_history) == 0:
            continue
        sensor_status['ranges'] = np.nanmean(np.vstack(sensor_history), axis=0).tolist()
    return _hits_from_candidates(_extract_candidates(source, out))


def filter_xy_cluster(source: Any, status: dict[str, Any], _state: dict[str, Any]) -> LineSensorHits:
    candidates = _extract_candidates(source, status)
    kept: list[Candidate] = []
    cluster_decisions: list[tuple[list[Candidate], bool, str]] = []
    for hazard_cls in (BinClass.OBSTACLE, BinClass.SMALL_DROP):
        cls_candidates = [candidate for candidate in candidates if candidate[3] == hazard_cls]
        for cluster in _xy_clusters(cls_candidates, CLUSTER_EPS_M):
            keep, reason = _cluster_filter_decision(source, cluster)
            cluster_decisions.append((cluster, keep, reason))
            if keep:
                kept.extend(cluster)

    confirmed = _confirmed_candidates(source, kept)
    confirmed_keys = {
        (sensor_idx, bin_idx, cls)
        for sensor_idx, _sensor_name, bin_idx, cls, _pt in confirmed
    }
    debug_clusters: list[ClusterDebug] = []
    for cluster, keep, reason in cluster_decisions:
        confirmed_count = sum(
            1
            for sensor_idx, _sensor_name, bin_idx, cls, _pt in cluster
            if (sensor_idx, bin_idx, cls) in confirmed_keys
        )
        debug_clusters.append((cluster, keep, reason, confirmed_count))

    _state['last_clusters'] = debug_clusters
    return _hits_from_candidates(confirmed)


def _xy_clusters(candidates: list[Candidate], eps_m: float) -> list[list[Candidate]]:
    if not candidates:
        return []

    points = np.vstack([candidate[4][:2] for candidate in candidates]).astype(np.float64)
    visited = np.zeros(len(candidates), dtype=bool)
    clusters: list[list[Candidate]] = []

    for start_idx in range(len(candidates)):
        if visited[start_idx]:
            continue
        visited[start_idx] = True
        component = [start_idx]
        queue = [start_idx]
        while queue:
            idx = queue.pop()
            distances = np.linalg.norm(points - points[idx], axis=1)
            neighbors = np.flatnonzero((distances <= eps_m) & (~visited))
            for neighbor in neighbors:
                visited[neighbor] = True
                queue.append(int(neighbor))
                component.append(int(neighbor))
        clusters.append([candidates[idx] for idx in component])

    return clusters


def _cluster_filter_decision(source: Any, cluster: list[Candidate]) -> tuple[bool, str]:
    run = _cluster_as_run(cluster)
    if not run:
        return False, 'empty'
    if source._is_spray(run):
        return False, 'spray'
    if source._is_point_noise(run):
        return False, 'point_noise'
    if source._valid_run(run):
        return True, 'keep'
    return False, 'invalid_run'


def _cluster_as_run(cluster: list[Candidate]) -> list[RunItem]:
    ordered = sorted(cluster, key=lambda candidate: (candidate[0], candidate[2]))
    return [
        (sensor_idx, bin_idx, cls, pt)
        for sensor_idx, _sensor_name, bin_idx, cls, pt in ordered
    ]


def _confirmed_candidates(source: Any, candidates: list[Candidate]) -> list[Candidate]:
    frame_map = {
        (sensor_idx, bin_idx): cls
        for sensor_idx, _sensor_name, bin_idx, cls, _pt in candidates
    }
    source._history.append(frame_map)

    confirmed: list[Candidate] = []
    for candidate in candidates:
        sensor_idx, _sensor_name, bin_idx, cls, pt = candidate
        confirm_frames = source._confirm_frames_for_bin(pt)
        if source._bin_confirmed(sensor_idx, bin_idx, cls, confirm_frames):
            confirmed.append(candidate)
    return confirmed


FILTERS: dict[str, FilterFn] = {
    'none': filter_none,
    'moving_average': filter_moving_average,
    'xy_cluster': filter_xy_cluster,
}


class ReplayPublisher(Node):
    def __init__(self, frame_id: str):
        super().__init__('line_sensor_replay_filter')
        self.frame_id = frame_id
        self.raw_candidates_obstacle_pub = self.create_publisher(
            PointCloud2, '/line_sensor_trial/raw_candidates/obstacle_points', 10,
        )
        self.raw_candidates_small_drop_pub = self.create_publisher(
            PointCloud2, '/line_sensor_trial/raw_candidates/small_drop_points', 10,
        )
        self.baseline_obstacle_pub = self.create_publisher(
            PointCloud2, '/line_sensor_trial/baseline/obstacle_points', 10,
        )
        self.baseline_small_drop_pub = self.create_publisher(
            PointCloud2, '/line_sensor_trial/baseline/small_drop_points', 10,
        )
        self.trial_obstacle_pub = self.create_publisher(
            PointCloud2, '/line_sensor_trial/trial/obstacle_points', 10,
        )
        self.trial_small_drop_pub = self.create_publisher(
            PointCloud2, '/line_sensor_trial/trial/small_drop_points', 10,
        )
        self.trial_clusters_pub = self.create_publisher(
            MarkerArray, '/line_sensor_trial/trial/clusters', 10,
        )
        self.trial_cluster_debug_pub = self.create_publisher(
            String, '/line_sensor_trial/trial/cluster_debug', 10,
        )

    def publish_raw_candidates(self, hits: LineSensorHits) -> None:
        self._publish_pair(
            self.raw_candidates_obstacle_pub,
            self.raw_candidates_small_drop_pub,
            hits.raw_obstacle_xy,
            hits.raw_small_drop_xy,
        )

    def publish_baseline(self, hits: LineSensorHits) -> None:
        self._publish_pair(
            self.baseline_obstacle_pub,
            self.baseline_small_drop_pub,
            hits.obstacle_xy,
            hits.small_drop_xy,
        )

    def publish_trial(self, hits: LineSensorHits) -> None:
        self._publish_pair(
            self.trial_obstacle_pub,
            self.trial_small_drop_pub,
            hits.obstacle_xy,
            hits.small_drop_xy,
        )

    def publish_trial_clusters(self, clusters: list[ClusterDebug]) -> None:
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = Header(stamp=stamp, frame_id=self.frame_id)
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for cluster_idx, (cluster, accepted, reason, confirmed_count) in enumerate(clusters):
            if not cluster:
                continue
            points_xy = np.vstack([candidate[4][:2] for candidate in cluster]).astype(np.float64)
            color = self._cluster_color(cluster_idx, accepted)

            points_marker = Marker()
            points_marker.header = Header(stamp=stamp, frame_id=self.frame_id)
            points_marker.ns = 'xy_cluster_points'
            points_marker.id = cluster_idx * 2 + 1
            points_marker.type = Marker.SPHERE_LIST
            points_marker.action = Marker.ADD
            points_marker.pose.orientation.w = 1.0
            points_marker.scale.x = 0.025
            points_marker.scale.y = 0.025
            points_marker.scale.z = 0.025
            points_marker.color.r = color[0]
            points_marker.color.g = color[1]
            points_marker.color.b = color[2]
            points_marker.color.a = color[3]
            for x, y in points_xy:
                points_marker.points.append(self._point(x, y, 0.05))
            marker_array.markers.append(points_marker)

            label_marker = Marker()
            label_marker.header = Header(stamp=stamp, frame_id=self.frame_id)
            label_marker.ns = 'xy_cluster_labels'
            label_marker.id = cluster_idx * 2 + 2
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.orientation.w = 1.0
            centroid = np.mean(points_xy, axis=0)
            label_marker.pose.position = self._point(centroid[0], centroid[1], 0.11)
            label_marker.scale.z = 0.06
            label_marker.color.r = color[0]
            label_marker.color.g = color[1]
            label_marker.color.b = color[2]
            label_marker.color.a = 1.0
            label_marker.text = f'C{cluster_idx} {reason} n={len(cluster)} c={confirmed_count}'
            marker_array.markers.append(label_marker)

        self.trial_clusters_pub.publish(marker_array)
        self.trial_cluster_debug_pub.publish(String(data=json.dumps(self._cluster_debug(clusters))))

    def _publish_pair(
        self,
        obstacle_pub,
        small_drop_pub,
        obstacle_xy: np.ndarray,
        small_drop_xy: np.ndarray,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        obstacle_pub.publish(numpy_to_pointcloud2(self._xy_to_xyz(obstacle_xy, 0.02), header))
        small_drop_pub.publish(numpy_to_pointcloud2(self._xy_to_xyz(small_drop_xy, -0.05), header))

    @staticmethod
    def _xy_to_xyz(xy: np.ndarray, z: float) -> np.ndarray:
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if len(xy) == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.column_stack([xy[:, 0], xy[:, 1], np.full(len(xy), z)])

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def _cluster_color(cluster_idx: int, accepted: bool) -> tuple[float, float, float, float]:
        hue = (cluster_idx * 0.61803398875) % 1.0
        red, green, blue = colorsys.hsv_to_rgb(hue, 0.75, 1.0)
        alpha = 0.95 if accepted else 0.25
        return red, green, blue, alpha

    @staticmethod
    def _cluster_debug(clusters: list[ClusterDebug]) -> dict[str, Any]:
        out: list[dict[str, Any]] = []
        for cluster_idx, (cluster, accepted, reason, confirmed_count) in enumerate(clusters):
            if not cluster:
                continue
            ordered = sorted(cluster, key=lambda candidate: (candidate[0], candidate[2]))
            points_xy = np.vstack([candidate[4][:2] for candidate in ordered]).astype(np.float64)
            centroid = np.mean(points_xy, axis=0)
            radius_m = float(np.max(np.linalg.norm(points_xy - centroid, axis=1)))
            out.append(
                {
                    'id': cluster_idx,
                    'class': ordered[0][3].name,
                    'accepted': bool(accepted),
                    'reason': reason,
                    'n': len(ordered),
                    'confirmed_n': int(confirmed_count),
                    'centroid_xy': [float(centroid[0]), float(centroid[1])],
                    'radius_m': radius_m,
                    'members': [
                        {
                            'sensor_idx': int(sensor_idx),
                            'sensor': sensor_name,
                            'bin': int(bin_idx),
                        }
                        for sensor_idx, sensor_name, bin_idx, _cls, _pt in ordered
                    ],
                },
            )
        return {'clusters': out}


def load_recording(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    frames: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as f:
        first_line = f.readline().strip()
        if not first_line:
            raise ValueError(f'empty recording: {path}')
        metadata = json.loads(first_line)
        if (
            metadata.get('record_type') != 'metadata'
            or metadata.get('format') != RECORDING_FORMAT
        ):
            raise ValueError(
                'recording is missing metadata. '
                'Record a new file with line_sensor_record so replay has the exact line sensor calibration/geometry.',
            )

        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    return metadata, frames


def replay_once(
    node: ReplayPublisher,
    metadata: dict[str, Any],
    frames: list[dict[str, Any]],
    filter_fn: FilterFn,
    speed: float,
) -> None:
    raw_source = make_line_source(metadata)
    baseline_source = make_line_source(metadata)
    trial_source = make_line_source(metadata)
    filter_state: dict[str, Any] = {}
    previous_t: float | None = None

    for frame in frames:
        current_t = float(frame.get('t', 0.0))
        if previous_t is not None:
            time.sleep(max(0.0, (current_t - previous_t) / speed))
        previous_t = current_t

        raw_status = frame_to_status(frame)
        raw_hits = filter_none(raw_source, raw_status, {})
        baseline_hits = baseline_source.process(raw_status)
        trial_hits = filter_fn(trial_source, raw_status, filter_state)
        node.publish_raw_candidates(raw_hits)
        node.publish_baseline(baseline_hits)
        node.publish_trial(trial_hits)
        node.publish_trial_clusters(filter_state.get('last_clusters', []))
        rclpy.spin_once(node, timeout_sec=0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description='Replay line sensor data and publish raw/baseline/trial output.')
    parser.add_argument('--file', type=Path, default=None, help='JSONL recording. Defaults to latest package data file.')
    parser.add_argument('--filter', default=FILTER_NAME, choices=sorted(FILTERS), help='Filter to apply.')
    parser.add_argument('--speed', type=float, default=1.0, help='Playback speed multiplier.')
    parser.add_argument('--loop', action='store_true', help='Replay forever.')
    parser.add_argument('--frame-id', default='base_link', help='Frame id for output point clouds.')
    args = parser.parse_args()

    if args.speed <= 0.0:
        parser.error('--speed must be positive')

    path = args.file or latest_recording()
    if path is None:
        parser.error('no recording found; pass --file or record data first')
    try:
        metadata, frames = load_recording(path)
    except ValueError as exc:
        parser.error(str(exc))
    if not frames:
        parser.error(f'no frames in {path}')

    rclpy.init()
    node = ReplayPublisher(args.frame_id)
    print(f'Replaying {path}')
    print(f'Trial filter: {args.filter}')
    print('raw_candidates: projected/classified candidate bins before LineSensorSource gates.')
    print('baseline: current stretch4_under_base_hazard LineSensorSource bin-level filtering.')
    print('trial: shared calibration/projection/classification, then the selected trial filter.')
    print('Publishing /line_sensor_trial/raw_candidates/*, /baseline/*, /trial/*, and cluster debug.')

    try:
        while rclpy.ok():
            replay_once(node, metadata, frames, FILTERS[args.filter], args.speed)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
