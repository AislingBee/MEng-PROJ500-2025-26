#!/usr/bin/env python3

import json
import struct
from pathlib import Path
from typing import Iterable, List


def get_workspace_src_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    return package_root.parent


def get_description_file_path(filename: str) -> Path:
    return get_workspace_src_path() / 'descriptions' / 'msgs' / filename


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


def clamp_rate(rate: float, default: float = 10.0) -> float:
    try:
        value = float(rate)
    except (TypeError, ValueError):
        return default
    return value if value > 0.0 else default
