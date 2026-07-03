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
from stretch4_under_base_hazard.line_sensor_source import LineSensorHits
from stretch4_under_base_hazard.pointcloud_io import numpy_to_pointcloud2


FILTER_NAME = 'baseline'
FILTER_WINDOW = 5


FilterFn = Callable[[dict[str, Any], dict[str, Any]], dict[str, Any]]


def filter_none(status: dict[str, Any], _state: dict[str, Any]) -> dict[str, Any]:
    return copy_status(status)


def filter_baseline(status: dict[str, Any], _state: dict[str, Any]) -> dict[str, Any]:
    # Baseline means no extra range filtering before the existing bin-level
    # LineSensorSource logic from stretch4_under_base_hazard.
    return copy_status(status)


def filter_moving_average(status: dict[str, Any], state: dict[str, Any]) -> dict[str, Any]:
    out = copy_status(status)
    history = state.setdefault('history', {})
    for sensor_name, sensor_status in out.items():
        ranges = np.asarray(sensor_status.get('ranges', []), dtype=np.float64)
        sensor_history = history.setdefault(sensor_name, deque(maxlen=FILTER_WINDOW))
        sensor_history.append(ranges)
        if len(sensor_history) == 0:
            continue
        sensor_status['ranges'] = np.nanmean(np.vstack(sensor_history), axis=0).tolist()
    return out


FILTERS: dict[str, FilterFn] = {
    'baseline': filter_baseline,
    'none': filter_none,
    'moving_average': filter_moving_average,
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
        trial_status = filter_fn(raw_status, filter_state)

        baseline_hits = baseline_source.process(raw_status)
        trial_hits = trial_source.process(trial_status)
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
    print(f'Filter: {args.filter}')
    print('raw_candidates: projected/classified candidate bins before LineSensorSource gates.')
    print('baseline: current stretch4_under_base_hazard LineSensorSource bin-level filtering.')
    print('trial: selected filter, then the same LineSensorSource bin-level filtering.')
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
