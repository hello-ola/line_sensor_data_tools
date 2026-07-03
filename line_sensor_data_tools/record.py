from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from line_sensor_data_tools.common import (
    package_data_dir,
    robot_sensor_names,
    status_to_json_sensors,
)


def default_output_path() -> Path:
    stamp = time.strftime('%Y%m%d_%H%M%S')
    return package_data_dir() / f'line_sensor_{stamp}.jsonl'


def main() -> int:
    parser = argparse.ArgumentParser(description='Record raw Stretch line sensor ranges.')
    parser.add_argument('--out', type=Path, default=None, help='Output JSONL file.')
    parser.add_argument('--rate', type=float, default=30.0, help='Sample rate in Hz.')
    parser.add_argument('--frames', type=int, default=0, help='Stop after N frames. 0 means run until Ctrl+C.')
    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error('--rate must be positive')
    if args.frames < 0:
        parser.error('--frames must be >= 0')

    out_path = args.out or default_output_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sensor_names = robot_sensor_names()

    from stretch4_body.robot.robot_client import RobotClient

    client = RobotClient()
    if not client.startup():
        raise RuntimeError('RobotClient startup failed')
    if not hasattr(client, 'line_sensor_loop'):
        client.stop()
        raise RuntimeError('line_sensor_loop not available on robot_server')

    period_s = 1.0 / args.rate
    start_s = time.monotonic()
    count = 0
    print(f'Writing {out_path}')
    print('Press Ctrl+C to stop.')

    try:
        with out_path.open('w', encoding='utf-8') as f:
            while args.frames == 0 or count < args.frames:
                frame_start_s = time.monotonic()
                client.pull_status()
                sensors = status_to_json_sensors(client.line_sensor_loop.status, sensor_names)
                frame = {
                    'seq': count,
                    't': frame_start_s - start_s,
                    'sensors': sensors,
                }
                f.write(json.dumps(frame, separators=(',', ':')) + '\n')
                f.flush()
                count += 1

                elapsed_s = time.monotonic() - frame_start_s
                time.sleep(max(0.0, period_s - elapsed_s))
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()

    print(f'Saved {count} frames to {out_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
