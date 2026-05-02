#!/usr/bin/env python3

import json
import struct
from pathlib import Path
from typing import Iterable, List


def find_descriptions_msgs_path() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        source_candidate = parent / 'config' / 'json'
        if source_candidate.is_dir():
            return source_candidate

        installed_candidate = parent / 'share' / 'motor_control' / 'config' / 'json'
        if installed_candidate.is_dir():
            return installed_candidate

    raise FileNotFoundError(
        'Could not locate config/json folder from %s' % current
    )


def get_description_file_path(filename: str) -> Path:
    return find_descriptions_msgs_path() / filename


def resolve_joint_names_file_path(names_file: str) -> Path:
    """Resolve a joint-name file from absolute path, repo path, or config/json."""
    candidate = Path(names_file)

    if candidate.is_absolute() and candidate.is_file():
        return candidate

    if candidate.is_file():
        return candidate.resolve()

    # Legacy location under Software/config/json (packaged with motor_control)
    legacy_path = get_description_file_path(candidate.name)
    if legacy_path.is_file():
        return legacy_path

    # Shared sim source-of-truth location in repo.
    for parent in Path(__file__).resolve().parents:
        sim_path = parent / 'simulation' / 'isaac' / 'configuration' / candidate.name
        if sim_path.is_file():
            return sim_path

        # Allow relative repo-style paths, e.g. simulation/.../joint_limits_config.json
        repo_style = parent / candidate
        if repo_style.is_file():
            return repo_style

    raise FileNotFoundError(f'Could not locate joint-names file: {names_file}')


def load_json_file(path: Path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def load_motor_names(names_file: str, motor_count: int, logger) -> List[str]:
    try:
        path = resolve_joint_names_file_path(names_file)
        data = load_json_file(path)

        # Preferred schema from simulation/isaac/configuration/joint_limits_config.json
        names = data.get('joint_names', [])
        if not names:
            # Backward compatibility with Software/config/json/motor_names.json
            names = data.get('motors', [])

        if not isinstance(names, list):
            raise ValueError('joint names field must be a list')
        return [str(name) for name in names]
    except Exception as exc:
        logger.warning(
            f'Failed to load joint names from JSON ({exc}), using default names'
        )
        return [f'motor_{i + 1}' for i in range(motor_count)]


def pack_floats(values: Iterable[float], fmt: str = '<ffff') -> bytes:
    return struct.pack(fmt, *values)


def bytes_to_uint8_list(payload: bytes) -> List[int]:
    return list(payload)


def get_software_log_dir() -> Path:
    """Return the Software/logs directory, searching up the directory tree.

    Works from both the source tree and the colcon-installed tree:
      source:    .../Software/src/motor_control/motor_control/common.py
      installed: .../Software/install/.../site-packages/motor_control/common.py

    Falls back to ~/motor_control_logs if the Software root cannot be located.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        # Source tree: Software/ contains src/motor_control/
        if (parent / 'src' / 'motor_control').is_dir():
            return parent / 'logs'
        # Installed tree: Software/ contains both src/ and install/
        if (parent / 'src').is_dir() and (parent / 'install').is_dir():
            return parent / 'logs'
    return Path.home() / 'motor_control_logs'


def clamp_rate(rate: float, default: float = 10.0) -> float:
    try:
        value = float(rate)
    except (TypeError, ValueError):
        return default
    return value if value > 0.0 else default
