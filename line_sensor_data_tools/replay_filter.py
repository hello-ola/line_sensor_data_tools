from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from line_sensor_data_tools.common import copy_status, frame_to_status, latest_recording, make_line_source
from stretch4_under_base_hazard.line_sensor_source import (
    BinClass,
    LineSensorHits,
    as_range_array,
)
from stretch4_under_base_hazard.pointcloud_io import numpy_to_pointcloud2


FILTER_NAME = 'none'
FILTER_WINDOW = 5
SPACE_TIME_FRAMES = 4
SPACE_TIME_SPATIAL_RADIUS = 1
SPACE_TIME_MIN_VALID = 4
SPACE_TIME_VARIANCE_THRESHOLD = 0.0025


Candidate = tuple[int, str, int, BinClass, np.ndarray]
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


def _calibrated_ranges_by_sensor(source: Any, status: dict[str, Any]) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for sensor_name in source.sensor_names:
        sensor_status = status.get(sensor_name, {})
        if not isinstance(sensor_status, dict):
            continue
        ranges = as_range_array(sensor_status.get('ranges'))
        if ranges.size == 0:
            continue
        if source.apply_tare is not None:
            ranges = source.apply_tare(ranges, sensor_name)
        out[sensor_name] = np.asarray(ranges, dtype=np.float64)
    return out


def _extract_candidates(
    source: Any,
    status: dict[str, Any],
    ranges_by_sensor: dict[str, np.ndarray] | None = None,
) -> list[Candidate]:
    cfg = source.config
    candidates: list[Candidate] = []

    for sensor_idx, sensor_name in enumerate(source.sensor_names):
        if ranges_by_sensor is None:
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges = as_range_array(sensor_status.get('ranges'))
            if ranges.size == 0:
                continue
            if source.apply_tare is not None:
                ranges = source.apply_tare(ranges, sensor_name)
        else:
            ranges = ranges_by_sensor.get(sensor_name, np.array([], dtype=np.float64))
            if ranges.size == 0:
                continue

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


def filter_space_time_patch(source: Any, status: dict[str, Any], state: dict[str, Any]) -> LineSensorHits:
    ranges_by_sensor = _calibrated_ranges_by_sensor(source, status)
    history = state.setdefault('calibrated_history', deque(maxlen=SPACE_TIME_FRAMES))
    history.append(ranges_by_sensor)

    kept: list[Candidate] = []
    for candidate in _extract_candidates(source, status, ranges_by_sensor):
        _sensor_idx, sensor_name, bin_idx, _cls, _pt = candidate
        values: list[float] = []
        for frame in history:
            ranges = frame.get(sensor_name)
            if ranges is None or ranges.size == 0:
                continue
            start = max(0, bin_idx - SPACE_TIME_SPATIAL_RADIUS)
            stop = min(len(ranges), bin_idx + SPACE_TIME_SPATIAL_RADIUS + 1)
            patch = ranges[start:stop]
            valid = patch[
                np.isfinite(patch)
                & (patch > 0.0)
                & (patch < source.config.max_range)
            ]
            values.extend(float(value) for value in valid)

        if len(values) >= SPACE_TIME_MIN_VALID:
            if float(np.var(np.asarray(values, dtype=np.float64))) > SPACE_TIME_VARIANCE_THRESHOLD:
                continue
        kept.append(candidate)

    return _hits_from_candidates(kept)


FILTERS: dict[str, FilterFn] = {
    'none': filter_none,
    'moving_average': filter_moving_average,
    'space_time_patch': filter_space_time_patch,
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


def load_frames(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    return frames


def replay_once(
    node: ReplayPublisher,
    frames: list[dict[str, Any]],
    filter_fn: FilterFn,
    speed: float,
) -> None:
    baseline_source = make_line_source()
    trial_source = make_line_source()
    filter_state: dict[str, Any] = {}
    previous_t: float | None = None

    for frame in frames:
        current_t = float(frame.get('t', 0.0))
        if previous_t is not None:
            time.sleep(max(0.0, (current_t - previous_t) / speed))
        previous_t = current_t

        raw_status = frame_to_status(frame)
        baseline_hits = baseline_source.process(raw_status)
        trial_hits = filter_fn(trial_source, raw_status, filter_state)
        node.publish_raw_candidates(baseline_hits)
        node.publish_baseline(baseline_hits)
        node.publish_trial(trial_hits)
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
    frames = load_frames(path)
    if not frames:
        parser.error(f'no frames in {path}')

    rclpy.init()
    node = ReplayPublisher(args.frame_id)
    print(f'Replaying {path}')
    print(f'Trial filter: {args.filter}')
    print('raw_candidates: projected/classified candidate bins before LineSensorSource gates.')
    print('baseline: current stretch4_under_base_hazard LineSensorSource bin-level filtering.')
    print('trial: shared calibration/projection/classification, then the selected trial filter.')
    print('Publishing /line_sensor_trial/raw_candidates/*, /baseline/*, and /trial/*')

    try:
        while rclpy.ok():
            replay_once(node, frames, FILTERS[args.filter], args.speed)
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
