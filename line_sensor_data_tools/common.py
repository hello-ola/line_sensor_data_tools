from __future__ import annotations

from dataclasses import asdict, fields
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


PACKAGE_NAME = 'line_sensor_data_tools'
RECORDING_FORMAT = 'line_sensor_data_tools_jsonl'
RECORDING_VERSION = 1


from stretch4_under_base_hazard.line_sensor_source import (
    LineSensorConfig,
    LineSensorSource,
)


class RecordedLineSensorGeometry:
    def __init__(self, params: dict[str, Any]):
        self.params = params
        self.param_height_cm = params.get('emitter_height_above_floor_mm', 100.67) / 10.0
        self.param_diameter_cm = params.get('emitter_pitch_diameter_mm', 404.04) / 10.0
        self.sensor_angles = params.get(
            'sensor_angles_deg',
            [10.18, 39.64, 80.36, 39.64, 80.36, 39.64],
        )
        self.sensor_normals = params.get(
            'sensor_normals_deg',
            [0.0, 60.0, 120.0, 180.0, 240.0, 300.0],
        )
        self.pixart_report_num = params.get('pixart_report_num', 320)
        self.horizontal_fov_degrees = params.get('sensor_horizontal_fov_degrees', 103.0)
        self.horizontal_fov_rad = np.deg2rad(self.horizontal_fov_degrees)
        self.angle_down_deg = params.get('sensor_angle_down_deg', 26.0)

    def get_angles(self) -> np.ndarray:
        return np.deg2rad(90.0) - np.linspace(
            -self.horizontal_fov_rad / 2.0,
            self.horizontal_fov_rad / 2.0,
            self.pixart_report_num,
        )

    def to_floor_coordinate_system(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        y_b = -x
        hypotenuse_m = y
        floor_y = self.param_height_cm / 100.0
        z_b = floor_y - (hypotenuse_m * np.sin(np.deg2rad(self.angle_down_deg)))
        x_b = hypotenuse_m * np.cos(np.deg2rad(self.angle_down_deg))
        return x_b, y_b, z_b


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


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return [_json_safe(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    return value


def make_recording_metadata() -> dict[str, Any]:
    from stretch4_body.core.robot_params import RobotParams

    _, robot_params = RobotParams.get_params()
    line_params = robot_params.get('line_sensor_loop', {})
    if not line_params:
        raise RuntimeError('line_sensor_loop params not found')
    if not line_params.get('sensor_names'):
        raise RuntimeError('line_sensor_loop sensor_names not found')
    if not line_params.get('line_sensor_geometry'):
        raise RuntimeError('line_sensor_loop line_sensor_geometry not found')

    return {
        'record_type': 'metadata',
        'format': RECORDING_FORMAT,
        'version': RECORDING_VERSION,
        'line_sensor_loop': _json_safe(line_params),
        'line_sensor_config': _json_safe(asdict(load_line_config())),
    }


def line_sensor_config_from_metadata(metadata: dict[str, Any]) -> LineSensorConfig:
    values = metadata.get('line_sensor_config')
    if not isinstance(values, dict):
        raise ValueError('recording metadata is missing line_sensor_config')
    config = LineSensorConfig()
    config_fields = {field.name for field in fields(LineSensorConfig)}
    clean_values = {key: value for key, value in values.items() if key in config_fields}
    return LineSensorConfig(**{**config.__dict__, **clean_values})


def make_line_source(metadata: dict[str, Any]) -> LineSensorSource:
    line_params = metadata.get('line_sensor_loop')
    if not isinstance(line_params, dict):
        raise ValueError('recording metadata is missing line_sensor_loop')
    sensor_names = line_params.get('sensor_names', [])
    if not sensor_names:
        raise ValueError('recording metadata line_sensor_loop is missing sensor_names')
    geometry_params = line_params.get('line_sensor_geometry', {})
    if not isinstance(geometry_params, dict) or not geometry_params:
        raise ValueError('recording metadata line_sensor_loop is missing line_sensor_geometry')

    tare_offsets = line_params.get('tare_offsets')
    tare_arrays = {}
    if isinstance(tare_offsets, dict):
        tare_arrays = {
            name: np.asarray(offsets, dtype=np.float64)
            for name, offsets in tare_offsets.items()
        }

    def apply_tare(ranges: np.ndarray, sensor_name: str) -> np.ndarray:
        adjustment = tare_arrays.get(sensor_name)
        if adjustment is None or ranges.shape != adjustment.shape:
            return ranges
        return ranges - adjustment

    return LineSensorSource(
        geometry=RecordedLineSensorGeometry(geometry_params),
        sensor_names=sensor_names,
        config=line_sensor_config_from_metadata(metadata),
        apply_tare=apply_tare if tare_arrays else None,
    )


def metadata_sensor_names(metadata: dict[str, Any]) -> list[str]:
    line_params = metadata.get('line_sensor_loop')
    if not isinstance(line_params, dict):
        raise ValueError('recording metadata is missing line_sensor_loop')
    sensor_names = line_params.get('sensor_names', [])
    if not sensor_names:
        raise ValueError('recording metadata line_sensor_loop is missing sensor_names')
    return list(sensor_names)


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
