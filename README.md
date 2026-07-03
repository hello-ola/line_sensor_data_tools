# line_sensor_data_tools

Small tools to record Stretch line sensor ranges and replay them for filter tests.

## Build

```bash
cd ~/ament_ws
colcon build --packages-select line_sensor_data_tools
source install/setup.bash
```

## Record

```bash
ros2 run line_sensor_data_tools line_sensor_record
```

This saves JSONL files in `line_sensor_data_tools/data/`.

Optional:

```bash
ros2 run line_sensor_data_tools line_sensor_record --rate 30 --out ~/ament_ws/src/line_sensor_data_tools/data/test.jsonl
```

Stop with `Ctrl+C`.

## Replay

```bash
ros2 run line_sensor_data_tools line_sensor_replay_filter --filter baseline
```

Or pick a file:

```bash
ros2 run line_sensor_data_tools line_sensor_replay_filter --file ~/ament_ws/src/line_sensor_data_tools/data/test.jsonl --filter moving_average
```

Published topics:

```text
/line_sensor_trial/raw_candidates/obstacle_points
/line_sensor_trial/raw_candidates/small_drop_points
/line_sensor_trial/baseline/obstacle_points
/line_sensor_trial/baseline/small_drop_points
/line_sensor_trial/trial/obstacle_points
/line_sensor_trial/trial/small_drop_points
```

`raw_candidates` are projected/classified bins before the current bin-level gates.

`baseline` is the current `stretch4_under_base_hazard` bin-level
`LineSensorSource` output.

`trial` is your selected filter, then the same `LineSensorSource` output.

It does not run the rolling hazard map.

## Add A Filter

Edit `line_sensor_data_tools/replay_filter.py`.

Add a function:

```python
def filter_my_filter(status, state):
    return copy_status(status)
```

Then add it:

```python
FILTERS = {
    'baseline': filter_baseline,
    'none': filter_none,
    'moving_average': filter_moving_average,
    'my_filter': filter_my_filter,
}
```

Run it:

```bash
ros2 run line_sensor_data_tools line_sensor_replay_filter --filter my_filter
```
