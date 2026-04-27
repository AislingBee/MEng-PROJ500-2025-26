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

        installed_candidate = parent / 'share' / 'motor_test' / 'config' / 'json'
        if installed_candidate.is_dir():
            return installed_candidate

    raise FileNotFoundError(
        'Could not locate config/json folder from %s' % current
    )


def get_description_file_path(filename: str) -> Path:
    return find_descriptions_msgs_path() / filename


def load_json_file(path: Path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def load_motor_names(names_file: str, motor_count: int, logger) -> List[str]:
    try:
        path = get_description_file_path(names_file)
        data = load_json_file(path)
        names = data.get('motors', [])
        if not isinstance(names, list):
            raise ValueError('motor_names.json "motors" must be a list')
        return [str(name) for name in names]
    except Exception as exc:
        logger.warning(
            f'Failed to load motor names from JSON ({exc}), using default names'
        )
        return [f'motor_{i + 1}' for i in range(motor_count)]


def pack_floats(values: Iterable[float], fmt: str = '<ffff') -> bytes:
    return struct.pack(fmt, *values)


def bytes_to_uint8_list(payload: bytes) -> List[int]:
    return list(payload)


def get_software_log_dir() -> Path:
    """Return the Software/logs directory, searching up the directory tree.

    Works from both the source tree and the colcon-installed tree:
      source:    .../Software/src/motor_test/motor_test/common.py
      installed: .../Software/install/.../site-packages/motor_test/common.py

    Falls back to ~/motor_test_logs if the Software root cannot be located.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        # Source tree: Software/ contains src/motor_test/
        if (parent / 'src' / 'motor_test').is_dir():
            return parent / 'logs'
        # Installed tree: Software/ contains both src/ and install/
        if (parent / 'src').is_dir() and (parent / 'install').is_dir():
            return parent / 'logs'
    return Path.home() / 'motor_test_logs'


def clamp_rate(rate: float, default: float = 10.0) -> float:
    try:
        value = float(rate)
    except (TypeError, ValueError):
        return default
    return value if value > 0.0 else default
