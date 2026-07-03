# line_sensor_data_tools

Small tools to record Stretch line sensor ranges and replay them for filter tests.

## Build

```bash
cd <your_ros2_ws>
colcon build --packages-select line_sensor_data_tools
source install/setup.bash
```

## Record

```bash
ros2 run line_sensor_data_tools line_sensor_record
```

This saves JSONL files in the package data directory. The first line stores
the line sensor metadata needed for offline replay.

Optional:

```bash
ros2 run line_sensor_data_tools line_sensor_record --rate 30 --out test.jsonl
```

Stop with `Ctrl+C`.

## Replay

```bash
ros2 run line_sensor_data_tools line_sensor_replay_filter --filter none
```

Or pick a file:

```bash
ros2 run line_sensor_data_tools line_sensor_replay_filter --file test.jsonl --filter xy_cluster
```

Published topics:

```text
/line_sensor_trial/raw_candidates/obstacle_points
/line_sensor_trial/raw_candidates/small_drop_points
/line_sensor_trial/baseline/obstacle_points
/line_sensor_trial/baseline/small_drop_points
/line_sensor_trial/trial/obstacle_points
/line_sensor_trial/trial/small_drop_points
/line_sensor_trial/trial/clusters
/line_sensor_trial/trial/cluster_debug
```

`raw_candidates` are projected/classified bins before the current bin-level gates.

`baseline` is the current `stretch4_under_base_hazard` bin-level
`LineSensorSource` output.

`trial` uses shared calibration/projection/classification, then your selected
trial filter.

`xy_cluster` groups candidates by XY distance, then applies the existing
run-level noise checks to each cluster: spray, point-noise, and valid-run.
It then applies the same bin temporal confirmation as baseline. It does not
use contiguous-bin grouping.

`/line_sensor_trial/trial/clusters` shows each XY cluster as a marker with a
label like `C3 keep n=8 c=4` or `C4 spray n=16 c=0`.

`/line_sensor_trial/trial/cluster_debug` publishes JSON with each cluster id,
class, reason, centroid, radius, confirmed count, and sensor/bin members.

It does not run the rolling hazard map.

Old range-only JSONL files are not replayable offline. Record again so the
file includes its own line sensor geometry/config metadata.

## Add A Filter

Edit `line_sensor_data_tools/replay_filter.py`.

Add a function:

```python
def filter_my_filter(source, status, state):
    candidates = _extract_candidates(source, status)
    return _hits_from_candidates(candidates)
```

Then add it:

```python
FILTERS = {
    'none': filter_none,
    'moving_average': filter_moving_average,
    'xy_cluster': filter_xy_cluster,
    'my_filter': filter_my_filter,
}
```

Run it:

```bash
ros2 run line_sensor_data_tools line_sensor_replay_filter --filter my_filter
```
