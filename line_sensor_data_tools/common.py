from __future__ import annotations

import contextlib
from dataclasses import fields
import io
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


PACKAGE_NAME = 'line_sensor_data_tools'


from stretch4_under_base_hazard.line_sensor_source import (
    LineSensorConfig,
    LineSensorSource,
)


class CalibrationParamsHolder:
    def __init__(self, params: dict[str, Any]):
        self.params = params


def package_data_dir() -> Path:
    source_data = Path(__file__).resolve().parents[1] / 'data'
    if source_data.exists():
        return source_data
    try:
        data_dir = Path(get_package_share_directory(PACKAGE_NAME)) / 'data'
    except PackageNotFoundError:
        data_dir = Path.cwd() / 'data'
    data_dir.mkdir(parents=True, exist_ok=True)
    return data_dir


def latest_recording() -> Path | None:
    files = sorted(package_data_dir().glob('*.jsonl'), key=lambda path: path.stat().st_mtime)
    return files[-1] if files else None


def load_line_config() -> LineSensorConfig:
    config = LineSensorConfig()
    try:
        path = Path(get_package_share_directory('stretch4_under_base_hazard')) / 'config' / 'hazard_map.yaml'
    except PackageNotFoundError:
        return config
    if not path.exists():
        return config

    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = data.get('hazard_map_node', {}).get('ros__parameters', {})
    config_fields = {field.name for field in fields(LineSensorConfig)}
    values = {key: value for key, value in params.items() if key in config_fields}
    return LineSensorConfig(**{**config.__dict__, **values})


def make_line_source() -> LineSensorSource:
    from stretch4_body.core.robot_params import RobotParams
    from stretch4_body.subsystem.line_sensor.line_sensor_utils import (
        LineSensorCalibration,
        LineSensorGeometry,
    )

    _, robot_params = RobotParams.get_params()
    line_params = robot_params.get('line_sensor_loop', {})
    sensor_names = line_params.get('sensor_names', [])
    geometry_params = line_params.get('line_sensor_geometry', {})
    with contextlib.redirect_stdout(io.StringIO()):
        geometry = LineSensorGeometry(geometry_params)
    calibration = LineSensorCalibration(CalibrationParamsHolder(line_params))
    return LineSensorSource(
        geometry=geometry,
        sensor_names=sensor_names,
        config=load_line_config(),
        apply_tare=calibration.apply_tare,
    )


def robot_sensor_names() -> list[str]:
    from stretch4_body.core.robot_params import RobotParams

    _, robot_params = RobotParams.get_params()
    return list(robot_params.get('line_sensor_loop', {}).get('sensor_names', []))


def ranges_to_json(ranges: Any) -> list[float | None]:
    arr = np.asarray(ranges, dtype=np.float64).reshape(-1)
    return [float(value) if np.isfinite(value) else None for value in arr]


def json_to_ranges(values: Any) -> list[float]:
    if values is None:
        return []
    return [float('nan') if value is None else float(value) for value in values]


def status_to_json_sensors(status: dict[str, Any], sensor_names: list[str]) -> dict[str, Any]:
    names = sensor_names or sorted(status.keys())
    sensors: dict[str, Any] = {}
    for name in names:
        sensor_status = status.get(name, {})
        if not isinstance(sensor_status, dict) or 'ranges' not in sensor_status:
            continue
        sensors[name] = {'ranges': ranges_to_json(sensor_status.get('ranges'))}
    return sensors


def frame_to_status(frame: dict[str, Any]) -> dict[str, Any]:
    sensors = frame.get('sensors', {})
    return {
        name: {'ranges': json_to_ranges(sensor_status.get('ranges'))}
        for name, sensor_status in sensors.items()
        if isinstance(sensor_status, dict)
    }


def copy_status(status: dict[str, Any]) -> dict[str, Any]:
    return {
        name: {'ranges': list(sensor_status.get('ranges', []))}
        for name, sensor_status in status.items()
        if isinstance(sensor_status, dict)
    }
