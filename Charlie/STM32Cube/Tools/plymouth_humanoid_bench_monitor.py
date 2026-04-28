#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plymouth Humanoid Bench Monitor
================================
PyQt6 + pyqtgraph telemetry, control and analysis GUI for the Plymouth
Humanoid bench rig (PDU + RCU + MachXO2 FPGA).

Requirements:
    pip install PyQt6 pyqtgraph numpy

Usage:
    python plymouth_humanoid_bench_monitor.py [--rcu-ip 192.168.100.10]

Protocol ports:
    7700  RCU→PC  slow telem (0x01), motor FB (0x02), debug reply (0x21)
    7701  PC→RCU  commands   (0x10 motor cmd, 0x20 debug cmd)
    7702  RCU→PC  supervision events (0x03)
"""

# ---------------------------------------------------------------------------
# Standard library
# ---------------------------------------------------------------------------
import argparse
import collections
import copy
import csv
import json
import math
import pathlib
import select
import socket
import struct
import threading
import time

# ---------------------------------------------------------------------------
# Third-party
# ---------------------------------------------------------------------------
import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QFrame, QLabel, QPushButton, QLineEdit, QComboBox,
    QCheckBox, QSlider, QDoubleSpinBox, QSpinBox,
    QTextEdit, QScrollArea, QGroupBox, QTabWidget,
    QTreeWidget, QTreeWidgetItem, QHeaderView,
    QSplitter, QMessageBox, QFileDialog,
    QListWidget, QListWidgetItem,
    QTableWidget, QTableWidgetItem,
    QSizePolicy, QAbstractItemView,
)


# ===========================================================================
# Protocol constants — must match rcu_pkt.h
# ===========================================================================

PKT_MAGIC        = 0x5243          # 'RC' little-endian
HDR_FMT          = "<HBBH"         # magic(u16), type(u8), seq(u8), len(u16)
HDR_SIZE         = struct.calcsize(HDR_FMT)   # 6

PKT_SLOW_TELEM   = 0x01
PKT_MOTOR_FB     = 0x02
PKT_SUPERVISION  = 0x03
PKT_MOTOR_CMD    = 0x10
PKT_DEBUG_CMD    = 0x20
PKT_DEBUG_REPLY  = 0x21

# rcu_telem_payload_t (rcu_pkt.h) — 72 bytes, little-endian
#   6×u8 + u16  =  8  (FPGA fields: status0, fc, sc, act, inputs, version; pchg_ms)
#   8×i16       = 16  (ext ADC: vraw_dv, 12v_mv, 24v_mv, i_vraw_sw, i_12v, i_24v, th1_dc, th2_dc)
#   4×i16       =  8  (energy meter: i_ma, v_raw, p_raw, t_raw)
#   6×i16       = 12  (local ADC: therm0, therm1, therm2, vsource, vbus, icoil)
#   7×i16       = 14  (IMU0: accel[3], gyro[3], temp)
#   7×i16       = 14  (IMU1: same layout)
TELEM_FMT  = "<6BH8h4h6h7h7h"
TELEM_SIZE = struct.calcsize(TELEM_FMT)   # 72

# Motor feedback header — 4 bytes (u8 count + 3 pad), from rcu_monitor.py.
# NOTE: the old rcu_bench_gui.py used FB_HEADER_SIZE=1 which is wrong.
FB_HEADER_FMT  = "<B3x"
FB_HEADER_SIZE = struct.calcsize(FB_HEADER_FMT)   # 4
FB_SLOT_FMT    = "<BBHHHBx"                        # bus, id, pos, vel, torque, err, pad
FB_SLOT_SIZE   = struct.calcsize(FB_SLOT_FMT)      # 10
FB_SLOTS       = 16

# Debug reply (rcu_debug_reply_t)
DBG_REPLY_FMT  = "<II6Bxx I"
DBG_REPLY_SIZE = struct.calcsize(DBG_REPLY_FMT)   # 20

# Debug sub-command bytes (RCU_DBGCMD_* in rcu_pkt.h)
DBGCMD_PING              = 0x01   # Request status reply immediately
DBGCMD_BUZZ              = 0x02   # 200 ms buzzer (RCU + PDU AUX bit3)
DBGCMD_LED_BLINK         = 0x03   # Blink orange LED (RCU + PDU AUX bit4)
DBGCMD_CAN_LOOPBACK      = 0x04   # Run FDCAN1+FDCAN3 loopback test
DBGCMD_FORCE_TELEM       = 0x05   # Force immediate slow-telem TX
# --- The following require new cases in eth_udp.c handle_debug_cmd() ---
DBGCMD_CLEAR_PDU_FAULT   = 0x06   # Trigger mcan_pdu_send_fault_req(false)
DBGCMD_ASSERT_PDU_FAULT  = 0x07   # Trigger mcan_pdu_send_fault_req(true) — test
DBGCMD_SOFT_RESET        = 0x08   # payload: 0=RCU only, 1=RCU+PDU via CAN
DBGCMD_SET_TELEM_RATE    = 0x09   # payload byte = rate Hz (5 / 10 / 20)
DBGCMD_MOTOR_BUS_CTRL    = 0x0A   # payload: bit0=L_STB, bit1=R_STB (1=standby)
DBGCMD_REQUEST_SUPV_DUMP = 0x0B   # Force supervision state dump

# Commands that require unreleased firmware (used to label buttons in the UI)
_FW_REQUIRED_CMDS = {
    DBGCMD_CLEAR_PDU_FAULT, DBGCMD_ASSERT_PDU_FAULT, DBGCMD_SOFT_RESET,
    DBGCMD_SET_TELEM_RATE, DBGCMD_MOTOR_BUS_CTRL, DBGCMD_REQUEST_SUPV_DUMP,
}

PORT_TELEM     = 7700    # RCU → PC telemetry
PORT_CMD       = 7701    # PC  → RCU commands
PORT_SUPV      = 7702    # RCU → PC supervision events

RCU_IP_DEFAULT = "192.168.100.10"

# IMU scale factors (LSM6DSOX, ±4g / ±500 dps)
IMU_ACCEL_G_PER_LSB  = 0.122e-3    # 0.122 mg/LSB at ±4g
IMU_GYRO_DPS_PER_LSB = 17.5e-3     # 17.5 mdps/LSB at ±500 dps
IMU_TEMP_SCALE       = 1.0 / 256.0
IMU_TEMP_OFFSET_C    = 25.0

# RS04 motor physical limits (rs04.h)
RS04_POS_MAX = 12.57
RS04_VEL_MAX = 15.0
RS04_TRQ_MAX = 120.0
RS04_KP_MAX  = 5000.0
RS04_KD_MAX  = 100.0

HISTORY_LEN = 6000       # ~10 min at 10 Hz
EMA_ALPHA   = 0.25       # EMA smoothing coefficient

# FPGA state codes
FPGA_STATES = {0: "IDLE", 1: "PRECHARGE", 2: "ARMED", 3: "COMPUTE"}

# STATUS0 register (0x00) bit definitions — pdu_glue_mxo2.sv
S0_BITS = [
    (7, "FAULT_LATCH"), (6, "PCHG_LATCH"), (5, "MOTOR_EN"),
    (4, "COMPUTE_EN"),  (3, "K_SEL"),       (2, "K_EN"),
    (1, "OVUV_OK"),     (0, "ARM_PERMIT"),
]

# INPUTS register (0x04) bit definitions
IN_BITS = [
    (7, "IS_ARMED"),  (6, "ARM_LATCH"),  (5, "ESTOP_OK"),
    (4, "MCU_ARM"),   (3, "MCU_ALIVE"),  (2, "FB_CLOSED"),
    (1, "PCHG_OK"),   (0, "VBUS_OV"),
]

# CAN AUX command bitmasks (0x530 frame, PDU)
AUX_BUZZ_MASK = 0x08
AUX_LED_MASK  = 0x10


# ===========================================================================
# Telemetry signal registry
# ===========================================================================
# Each entry: (display_label, telem_dict_key, default_pdu_checked, default_imu_checked)
#
# Thermistor map:
#   therm1_c        ext ADC ch1   — PDU PCB, soldered, critical thermal spot 1
#   therm2_c        ext ADC ch2   — PDU PCB, soldered, critical thermal spot 2
#   ladc_therm0_c   STM32 local   — RCU PCB NTC, physically next to IMU0
#   ladc_therm1_c   STM32 local   — PDU external connector (typically unpopulated)
#   ladc_therm2_c   STM32 local   — PDU onboard NTC (trusted, always present)
TELEM_SIGNALS: list[tuple[str, str, bool, bool]] = [
    # --- External ADC power rails ---
    ("V_SRC",          "v_vraw_v",        True,  False),  # Switched source ~53 V
    ("12V_SW",         "v_12v_v",         True,  False),  # Switched 12V output
    ("24V_SW",         "v_24v_v",         True,  False),  # Switched 24V output
    ("I_SRC",          "i_vraw_sw_ma",    False, False),  # Source current
    ("I_12V_SW",       "i_12v_ma",        False, False),  # 12V switched current
    ("I_24V_SW",       "i_24v_ma",        False, False),  # 24V switched current
    # PDU PCB thermistors (both soldered directly on PSU PCB, critical spots)
    ("PDU Therm 1",    "therm1_c",        False, False),  # PDU PCB thermal spot 1
    ("PDU Therm 2",    "therm2_c",        False, False),  # PDU PCB thermal spot 2
    # --- Energy meter (RS485/SSD, 5 Hz) ---
    ("EM Volt",        "em_v_v",          False, False),
    ("EM Curr",        "em_i_ma",         False, False),
    ("EM Power",       "em_p_w",          False, False),
    ("EM Temp",        "em_t_c",          False, False),
    # --- STM32 local ADC ---
    ("RCU Board NTC",  "ladc_therm0_c",   False, False),  # RCU PCB, next to IMU0
    ("PDU Ext Therm",  "ladc_therm1_c",   False, False),  # PDU ext connector (unpopulated)
    ("PDU Board NTC",  "ladc_therm2_c",   False, False),  # PDU onboard NTC (trusted)
    ("V_SRC (loc)",    "ladc_vsource_v",  False, False),  # STM32 ADC on source rail
    ("12V (loc)",      "ladc_vbus_v",     False, False),  # STM32 ADC on 12V bus
    ("I_COIL",         "ladc_icoil_ma",   False, False),  # Coil current
    # --- IMU 0 (SPI4, CS=PC13) ---
    ("IMU0 Ax",        "imu0_ax_g",       False, True),
    ("IMU0 Ay",        "imu0_ay_g",       False, True),
    ("IMU0 Az",        "imu0_az_g",       False, True),
    ("IMU0 |a|",       "imu0_mag",        False, True),
    ("IMU0 Gx",        "imu0_gx_dps",     False, False),
    ("IMU0 Gy",        "imu0_gy_dps",     False, False),
    ("IMU0 Gz",        "imu0_gz_dps",     False, False),
    ("IMU0 Temp",      "imu0_temp_c",     False, False),
    # --- IMU 1 (SPI3, CS=PA15) ---
    ("IMU1 Ax",        "imu1_ax_g",       False, True),
    ("IMU1 Ay",        "imu1_ay_g",       False, True),
    ("IMU1 Az",        "imu1_az_g",       False, True),
    ("IMU1 |a|",       "imu1_mag",        False, True),
    ("IMU1 Gx",        "imu1_gx_dps",     False, False),
    ("IMU1 Gy",        "imu1_gy_dps",     False, False),
    ("IMU1 Gz",        "imu1_gz_dps",     False, False),
    ("IMU1 Temp",      "imu1_temp_c",     False, False),
]

# Key → (label, default_pdu, default_imu) lookup
_SIG_BY_KEY: dict[str, tuple[str, bool, bool]] = {
    key: (lbl, dp, di) for lbl, key, dp, di in TELEM_SIGNALS
}


# ===========================================================================
# General-purpose helpers
# ===========================================================================

def u16_to_f(raw: int, lo: float, hi: float) -> float:
    return lo + (raw / 65535.0) * (hi - lo)


def f_to_u16(v: float, lo: float, hi: float) -> int:
    v = max(lo, min(hi, v))
    return int((v - lo) / (hi - lo) * 65535 + 0.5)


def decode_bits(val: int, bit_defs: list) -> str:
    active = [name for bit, name in bit_defs if (val >> bit) & 1]
    return " | ".join(active) if active else "none"


def ema(prev: float, new_val: float, alpha: float = EMA_ALPHA) -> float:
    """Exponential moving average; returns new_val on first real sample (NaN prev)."""
    if math.isnan(new_val):
        return prev
    if math.isnan(prev):
        return new_val
    return prev * (1.0 - alpha) + new_val * alpha


def _color_label(val: float, lo_warn: float, hi_warn: float,
                 lo_err: float | None = None, hi_err: float | None = None) -> str:
    """Return a One Dark hex colour based on value against thresholds."""
    if math.isnan(val):
        return DIM
    if lo_err is not None and val <= lo_err:
        return ERROR
    if hi_err is not None and val >= hi_err:
        return ERROR
    if val < lo_warn or val > hi_warn:
        return WARN
    return OK


def _fmt_val(val, fmt: str = "", suffix: str = "") -> str:
    """Format a float, or return '---' for NaN/None."""
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return "---"
    return f"{val:{fmt}}{suffix}"


def _is_no_sensor(val: float) -> bool:
    """Detect an unpopulated / open-circuit thermistor channel."""
    if math.isnan(val):
        return True
    return abs(val) < 1.0 or val > 150.0 or val < -40.0


# ===========================================================================
# One Dark palette
# ===========================================================================

BG     = "#1e2229"
PANEL  = "#282c34"
ACCENT = "#61afef"
OK     = "#98c379"
WARN   = "#e5c07b"
ERROR  = "#e06c75"
TEXT   = "#abb2bf"
DIM    = "#5c6370"
WHITE  = "#ffffff"
PURPLE = "#c678dd"

# Stable per-signal colours — index is fixed to TELEM_SIGNALS position,
# so toggling a checkbox never changes another signal's colour.
_SIG_PALETTE: list[str] = [
    ACCENT,    OK,        WARN,      ERROR,     PURPLE,    WHITE,
    "#ff9e64", "#56b6c2", "#d19a66", "#be5046", "#7ec8e3", "#e8b86d",
    "#98c379", "#c678dd", "#61afef", "#e06c75",
]
SIGNAL_COLORS: dict[str, str] = {
    key: _SIG_PALETTE[i % len(_SIG_PALETTE)]
    for i, (_, key, _, _) in enumerate(TELEM_SIGNALS)
}


# ===========================================================================
# Dark Qt stylesheet
# ===========================================================================

DARK_STYLESHEET = f"""
QMainWindow, QDialog {{
    background-color: {BG};
}}
QWidget {{
    background-color: {BG};
    color: {TEXT};
    font-family: "Consolas", "Courier New", monospace;
    font-size: 11px;
}}
QTabWidget::pane {{
    border: 1px solid {DIM};
    background-color: {BG};
    top: -1px;
}}
QTabBar::tab {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {DIM};
    border-bottom: none;
    padding: 6px 16px;
    min-width: 72px;
}}
QTabBar::tab:selected {{
    background-color: {ACCENT};
    color: {BG};
    font-weight: bold;
    border-color: {ACCENT};
}}
QTabBar::tab:hover:!selected {{
    background-color: #323842;
    color: {WHITE};
}}
QFrame {{
    background-color: {PANEL};
    border: none;
    border-radius: 6px;
}}
QGroupBox {{
    background-color: {PANEL};
    border: 1px solid {DIM};
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 6px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 8px;
    padding: 0 4px;
    color: {ACCENT};
    font-weight: bold;
}}
QPushButton {{
    background-color: {ACCENT};
    color: {BG};
    border: none;
    border-radius: 4px;
    padding: 5px 14px;
    font-weight: bold;
    font-size: 11px;
}}
QPushButton:hover {{
    background-color: #4d9bd4;
}}
QPushButton:pressed {{
    background-color: #3a7ab0;
}}
QPushButton:disabled {{
    background-color: {DIM};
    color: #3a3f4b;
}}
QPushButton[role="warn"] {{
    background-color: transparent;
    color: {WARN};
    border: 1px solid {WARN};
}}
QPushButton[role="warn"]:hover {{
    background-color: #3a3520;
}}
QPushButton[role="danger"] {{
    background-color: {ERROR};
    color: {WHITE};
    border: none;
}}
QPushButton[role="danger"]:hover {{
    background-color: #c0404f;
}}
QPushButton[role="ok"] {{
    background-color: {OK};
    color: {BG};
    border: none;
}}
QPushButton[role="ok"]:hover {{
    background-color: #7aad5c;
}}
QLineEdit {{
    background-color: {BG};
    color: {TEXT};
    border: 1px solid {DIM};
    border-radius: 4px;
    padding: 3px 6px;
    selection-background-color: {ACCENT};
    selection-color: {BG};
}}
QLineEdit:focus {{
    border-color: {ACCENT};
}}
QComboBox {{
    background-color: {BG};
    color: {TEXT};
    border: 1px solid {DIM};
    border-radius: 4px;
    padding: 3px 28px 3px 8px;
    min-width: 80px;
}}
QComboBox::drop-down {{
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 20px;
    border: none;
}}
QComboBox QAbstractItemView {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {DIM};
    selection-background-color: {ACCENT};
    selection-color: {BG};
    outline: none;
}}
QScrollArea {{
    background-color: {BG};
    border: none;
}}
QScrollBar:vertical {{
    width: 8px;
    background: {BG};
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {DIM};
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
    height: 0px;
    background: none;
}}
QScrollBar:horizontal {{
    height: 8px;
    background: {BG};
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: {DIM};
    border-radius: 4px;
    min-width: 24px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal,
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
    width: 0px;
    background: none;
}}
QLabel {{
    background: transparent;
    color: {TEXT};
}}
QCheckBox {{
    color: {TEXT};
    spacing: 6px;
    background: transparent;
}}
QCheckBox::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid {DIM};
    border-radius: 3px;
    background: {BG};
}}
QCheckBox::indicator:checked {{
    background-color: {ACCENT};
    border-color: {ACCENT};
}}
QSlider::groove:horizontal {{
    height: 4px;
    background: {DIM};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {ACCENT};
    border: none;
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 7px;
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT};
    border-radius: 2px;
}}
QTextEdit, QPlainTextEdit {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {DIM};
    border-radius: 4px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 10px;
}}
QTreeWidget, QTreeView, QListWidget, QListView,
QTableWidget, QTableView {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {DIM};
    border-radius: 4px;
    gridline-color: {DIM};
    alternate-background-color: #262b33;
    outline: none;
}}
QTreeWidget::item:selected, QListWidget::item:selected,
QTableWidget::item:selected {{
    background-color: {ACCENT};
    color: {BG};
}}
QHeaderView::section {{
    background-color: {BG};
    color: {ACCENT};
    border: none;
    border-right: 1px solid {DIM};
    border-bottom: 1px solid {DIM};
    padding: 4px 8px;
    font-weight: bold;
}}
QDoubleSpinBox, QSpinBox {{
    background-color: {BG};
    color: {TEXT};
    border: 1px solid {DIM};
    border-radius: 4px;
    padding: 3px 6px;
}}
QDoubleSpinBox:focus, QSpinBox:focus {{
    border-color: {ACCENT};
}}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button, QSpinBox::down-button {{
    width: 16px;
    border: none;
    background: {DIM};
}}
QToolTip {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {DIM};
    padding: 4px 6px;
}}
QSplitter::handle:horizontal {{
    background-color: {DIM};
    width: 2px;
}}
QSplitter::handle:vertical {{
    background-color: {DIM};
    height: 2px;
}}
QMessageBox {{
    background-color: {PANEL};
}}
QMessageBox QLabel {{
    color: {TEXT};
    background: transparent;
}}
"""


def _configure_pyqtgraph(use_opengl: bool = True, antialias: bool = True) -> None:
    """Apply pyqtgraph global settings. Call once before creating any pg widgets."""
    pg.setConfigOption('background', BG)
    pg.setConfigOption('foreground', TEXT)
    pg.setConfigOption('antialias', antialias)
    pg.setConfigOption('useOpenGL', use_opengl)
    pg.setConfigOption('leftButtonPan', False)


# ===========================================================================
# Non-volatile configuration
# ===========================================================================

_CONFIG_PATH = pathlib.Path(__file__).resolve().parent / "bench_config.json"

_DEFAULTS: dict = {
    "connection": {
        "rcu_ip":               RCU_IP_DEFAULT,
        "auto_reconnect":       True,
        "reconnect_interval_s": 2.0,
    },
    "display": {
        "default_time_window_s": 30,
        "update_rate_ms":        100,
    },
    "plots": {
        "line_width":           1.5,
        "antialiasing":         True,
        "use_opengl":           True,
        "pdu_signals_checked":  {},   # key → bool; missing key uses TELEM_SIGNALS default
        "imu_signals_checked":  {},
    },
    "alerts": {
        "v_src_warn_min_v":     45.0,
        "v_src_warn_max_v":     58.0,
        "v_12v_warn_min_v":     11.0,
        "v_12v_warn_max_v":     13.0,
        "v_24v_warn_min_v":     22.0,
        "v_24v_warn_max_v":     26.0,
        "temp_warn_max_c":      55.0,
        "temp_err_max_c":       70.0,
    },
    "recording": {
        "default_dir":          "",
        "signals":              [],
    },
    "ui": {
        "last_tab_index":       0,
        "window_geometry":      "",
    },
}


class Config:
    """
    Non-volatile JSON configuration backed by bench_config.json.

    Design principles:
    - Never raises. Any filesystem / parse failure silently uses defaults.
    - Deep-merges user file into defaults so new keys are always present.
    - Type mismatches between file and defaults keep the default value.
    """

    def __init__(self) -> None:
        self._data: dict = copy.deepcopy(_DEFAULTS)
        self._load()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _load(self) -> None:
        try:
            with open(_CONFIG_PATH, "r", encoding="utf-8") as fh:
                user_data = json.load(fh)
            Config._deep_merge(self._data, user_data)
        except Exception:
            pass  # FileNotFoundError, JSONDecodeError, PermissionError — all silent

    @staticmethod
    def _deep_merge(base: dict, overlay: dict) -> None:
        """
        Recursively copy values from *overlay* into *base*.
        Unknown keys in overlay are ignored.
        Type mismatches (beyond int↔float coercion) keep the base default.
        """
        for k, v in overlay.items():
            if k not in base:
                continue
            if isinstance(base[k], dict) and isinstance(v, dict):
                Config._deep_merge(base[k], v)
            elif type(base[k]) is type(v):
                base[k] = v
            elif isinstance(base[k], float) and isinstance(v, int):
                base[k] = float(v)
            elif isinstance(base[k], int) and isinstance(v, float) and v == int(v):
                base[k] = int(v)
            # else: type mismatch → keep base default

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get(self, *keys, default=None):
        """Safe nested read: cfg.get("plots", "line_width", default=1.5)"""
        node = self._data
        try:
            for k in keys:
                node = node[k]
            return node
        except (KeyError, TypeError):
            return default

    def set(self, *keys_and_val) -> None:
        """Safe nested write: cfg.set("plots", "line_width", 2.0)"""
        if len(keys_and_val) < 2:
            return
        *keys, val = keys_and_val
        node = self._data
        try:
            for k in keys[:-1]:
                if k not in node or not isinstance(node[k], dict):
                    node[k] = {}
                node = node[k]
            node[keys[-1]] = val
        except Exception:
            pass

    def save(self) -> None:
        """Write current config to disk. Silently ignores any failure."""
        try:
            with open(_CONFIG_PATH, "w", encoding="utf-8") as fh:
                json.dump(self._data, fh, indent=2)
        except Exception:
            pass

    def snapshot(self) -> dict:
        """Return a deep copy of the full config dict (e.g. for Settings UI)."""
        return copy.deepcopy(self._data)


# ===========================================================================
# Telemetry recorder
# ===========================================================================

class TelemetryRecorder:
    """Thread-safe CSV recorder. Writes one row per telemetry update."""

    def __init__(self) -> None:
        self._lock       = threading.Lock()
        self._active     = False
        self._file       = None
        self._writer     = None
        self._keys: list[str] = []
        self._row_count  = 0
        self._start_time = 0.0

    def start(self, filepath: str, keys: list[str]) -> bool:
        """Open *filepath* for writing; begin recording *keys*. Returns False on error."""
        with self._lock:
            if self._active:
                return False
            try:
                self._file   = open(filepath, "w", newline="", encoding="utf-8")
                self._keys   = list(keys)
                fields       = ["timestamp_s"] + self._keys
                self._writer = csv.DictWriter(self._file, fieldnames=fields,
                                              extrasaction="ignore")
                self._writer.writeheader()
                self._active     = True
                self._row_count  = 0
                self._start_time = time.monotonic()
                return True
            except Exception:
                self._cleanup_locked()
                return False

    def stop(self) -> None:
        with self._lock:
            if not self._active:
                return
            self._active = False
            self._cleanup_locked()

    def _cleanup_locked(self) -> None:
        """Must be called with self._lock held."""
        try:
            if self._file:
                self._file.flush()
                self._file.close()
        except Exception:
            pass
        self._file   = None
        self._writer = None

    def record(self, timestamp: float, t_dict: dict) -> None:
        with self._lock:
            if not self._active or self._writer is None:
                return
            try:
                row: dict = {"timestamp_s": f"{timestamp:.4f}"}
                for k in self._keys:
                    v = t_dict.get(k, float('nan'))
                    row[k] = ("" if isinstance(v, float) and math.isnan(v)
                               else str(v))
                self._writer.writerow(row)
                self._row_count += 1
                if self._row_count % 20 == 0:
                    self._file.flush()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Status properties (read from GUI thread without acquiring the lock
    # on scalar reads — acceptable for display-only use)
    # ------------------------------------------------------------------

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def row_count(self) -> int:
        with self._lock:
            return self._row_count

    @property
    def elapsed_s(self) -> float:
        with self._lock:
            if not self._active:
                return 0.0
            return time.monotonic() - self._start_time

    @property
    def file_size_bytes(self) -> int:
        with self._lock:
            if self._file is None:
                return 0
            try:
                return self._file.tell()
            except Exception:
                return 0


# ===========================================================================
# Sequence tracker (logic from rcu_monitor.py)
# ===========================================================================

class SeqTracker:
    """
    Tracks the single global UDP sequence counter.

    The RCU firmware increments g_seq for every outgoing packet regardless
    of packet type, so tracking per-type produces false gap alarms.
    """

    def __init__(self) -> None:
        self.total = 0
        self.gaps  = 0
        self._last = -1

    def update(self, seq: int) -> int:
        """Process received seq; returns number of gaps detected (usually 0)."""
        self.total += 1
        if self._last < 0:
            self._last = seq
            return 0
        delta    = (seq - self._last) & 0xFF
        new_gaps = 0
        if delta == 0:
            new_gaps = 1          # duplicate / reorder
        elif delta > 1:
            new_gaps = delta - 1  # missed packets
        self.gaps  += new_gaps
        self._last  = seq
        return new_gaps

    def reset(self) -> None:
        self.total = 0
        self.gaps  = 0
        self._last = -1


# ===========================================================================
# Thread-safe telemetry state
# ===========================================================================

class RcuState:
    """
    Shared state object — written by the receiver thread, read by the GUI thread.
    All public methods are thread-safe via a single lock.
    """

    def __init__(self) -> None:
        self._lock              = threading.Lock()
        self.connected          = False
        self.rcu_ip             = RCU_IP_DEFAULT
        self._pkt_ts_deque      = collections.deque(maxlen=50)
        self._last_telem_t      = 0.0
        self.t: dict            = {}
        self._ema: dict         = {}
        self.motor_slots: list  = []
        self.last_ping: dict    = {}
        self.seq_tracker        = SeqTracker()
        self.supervision_events: collections.deque = collections.deque(maxlen=200)
        self._recorder: TelemetryRecorder | None = None

        # Per-signal history deques — pre-filled with NaN so plots render cleanly
        self._hist: dict[str, collections.deque] = {
            key: collections.deque(
                [float('nan')] * HISTORY_LEN, maxlen=HISTORY_LEN
            )
            for _, key, _, _ in TELEM_SIGNALS
        }

    # ------------------------------------------------------------------
    # Telemetry update (receiver thread)
    # ------------------------------------------------------------------

    def update_telem(self, t: dict) -> None:
        now = time.monotonic()
        with self._lock:
            self._last_telem_t = now
            self.t = dict(t)

            # EMA-smoothed values for numeric display
            for key in (
                "v_vraw_v", "v_12v_v", "v_24v_v",
                "i_vraw_sw_ma", "i_12v_ma", "i_24v_ma",
                "em_v_v", "em_i_ma", "em_p_w",
                "ladc_vsource_v", "ladc_vbus_v",
            ):
                raw  = t.get(key, float('nan'))
                prev = self._ema.get(key, float('nan'))
                self._ema[key] = ema(prev, raw)

            # Derive IMU vector magnitudes
            for pfx in ("imu0", "imu1"):
                ax = t.get(f"{pfx}_ax_g", 0.0)
                ay = t.get(f"{pfx}_ay_g", 0.0)
                az = t.get(f"{pfx}_az_g", 0.0)
                valid = ax != 0 or ay != 0 or az != 0
                t[f"{pfx}_mag"] = (math.sqrt(ax*ax + ay*ay + az*az)
                                   if valid else float('nan'))
                if not valid:
                    for ax_key in (
                        f"{pfx}_ax_g", f"{pfx}_ay_g", f"{pfx}_az_g",
                        f"{pfx}_gx_dps", f"{pfx}_gy_dps", f"{pfx}_gz_dps",
                    ):
                        t[ax_key] = float('nan')

            # Push all signals into history deques
            for _, key, _, _ in TELEM_SIGNALS:
                self._hist[key].append(t.get(key, float('nan')))

        # CSV recording (outside the lock to avoid holding it during I/O)
        if self._recorder is not None and self._recorder.is_active:
            self._recorder.record(now, t)

    # ------------------------------------------------------------------
    # Packet / connection bookkeeping
    # ------------------------------------------------------------------

    def bump_pkt(self, seq: int) -> None:
        with self._lock:
            self.seq_tracker.update(seq)
            self._pkt_ts_deque.append(time.monotonic())

    def pkts_per_sec(self) -> int:
        with self._lock:
            now = time.monotonic()
            return sum(1 for ts in self._pkt_ts_deque if now - ts <= 1.0)

    def telem_age_ms(self) -> int:
        with self._lock:
            if self._last_telem_t == 0.0:
                return 9999
            return int((time.monotonic() - self._last_telem_t) * 1000)

    def set_connected(self, v: bool) -> None:
        with self._lock:
            self.connected = v

    def is_connected(self) -> bool:
        with self._lock:
            return self.connected

    def reset_gap_counter(self) -> None:
        with self._lock:
            self.seq_tracker.reset()

    # ------------------------------------------------------------------
    # Getters — return safe copies for the GUI thread
    # ------------------------------------------------------------------

    def get_telem(self) -> dict:
        with self._lock:
            return dict(self.t)

    def get_ema(self, key: str) -> float:
        """EMA-smoothed value, or NaN if no data has been received yet."""
        with self._lock:
            return self._ema.get(key, float('nan'))

    def get_motor_slots(self) -> list:
        with self._lock:
            return list(self.motor_slots)

    def update_motor_fb(self, slots: list) -> None:
        with self._lock:
            self.motor_slots = slots

    def update_ping(self, reply: dict) -> None:
        with self._lock:
            self.last_ping = dict(reply)

    def get_ping(self) -> dict:
        with self._lock:
            return dict(self.last_ping)

    def add_supervision_event(self, ts: float, raw: bytes) -> None:
        with self._lock:
            self.supervision_events.append((ts, raw))

    def get_supervision_events(self) -> list:
        with self._lock:
            return list(self.supervision_events)

    def get_hist_window(self, key: str, n: int) -> list:
        """Return the last *n* history samples for *key* as a Python list."""
        with self._lock:
            d = self._hist.get(key)
            if d is None:
                return [float('nan')] * n
            data = list(d)
            if len(data) >= n:
                return data[-n:]
            return [float('nan')] * (n - len(data)) + data

    def attach_recorder(self, recorder: TelemetryRecorder | None) -> None:
        with self._lock:
            self._recorder = recorder


# ===========================================================================
# UDP receiver thread
# ===========================================================================

class RcuReceiver(threading.Thread):
    """
    Binds UDP on PORT_TELEM (7700) and PORT_SUPV (7702).
    Decodes incoming packets and updates RcuState.
    Uses select() to multiplex both sockets in a single thread.
    Restarts automatically on socket errors after a 1-second delay.
    """

    def __init__(self, state: RcuState) -> None:
        super().__init__(daemon=True, name="RcuReceiver")
        self.rcu   = state
        self._stop = threading.Event()
        self._socks: list[socket.socket] = []

    def stop(self) -> None:
        self._stop.set()
        for s in self._socks:
            try:
                s.close()
            except Exception:
                pass

    def run(self) -> None:
        while not self._stop.is_set():
            socks: list[socket.socket] = []
            try:
                for port in (PORT_TELEM, PORT_SUPV):
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind(("0.0.0.0", port))
                    s.setblocking(False)
                    socks.append(s)
                self._socks = socks
                self.rcu.set_connected(True)

                while not self._stop.is_set():
                    readable, _, _ = select.select(socks, [], [], 1.0)
                    for s in readable:
                        try:
                            data, _ = s.recvfrom(4096)
                            self._process(data)
                        except OSError:
                            pass

            except Exception:
                pass
            finally:
                self.rcu.set_connected(False)
                for s in socks:
                    try:
                        s.close()
                    except Exception:
                        pass
                self._socks = []

            if not self._stop.is_set():
                time.sleep(1.0)

    def _process(self, data: bytes) -> None:
        if len(data) < HDR_SIZE:
            return
        magic, pkt_type, seq, plen = struct.unpack_from(HDR_FMT, data, 0)
        if magic != PKT_MAGIC:
            return
        payload = data[HDR_SIZE: HDR_SIZE + plen]
        self.rcu.bump_pkt(seq)

        if pkt_type == PKT_SLOW_TELEM:
            t = _decode_telem(payload)
            if t:
                self.rcu.update_telem(t)
        elif pkt_type == PKT_MOTOR_FB:
            self.rcu.update_motor_fb(_decode_motor_fb(payload))
        elif pkt_type == PKT_DEBUG_REPLY:
            r = _decode_debug_reply(payload)
            if r:
                self.rcu.update_ping(r)
        elif pkt_type == PKT_SUPERVISION:
            self.rcu.add_supervision_event(time.monotonic(), payload)


# ===========================================================================
# Packet decoders — scale factors match rcu_monitor.py / rcu_pkt.h
# ===========================================================================

def _decode_telem(payload: bytes) -> dict | None:
    if len(payload) < TELEM_SIZE:
        return None
    (s0, fc, sc, act, inp, ver, pchg_ms,
     # External ADC
     v_vraw_dv, v_12v, v_24v, i_vraw_sw, i_12v, i_24v, th1_dc, th2_dc,
     # Energy meter
     em_i, em_v_raw, em_p_raw, em_t_raw,
     # Local STM32 ADC
     lt0, lt1, lt2, lvsrc, lvbus, licoil,
     # IMU 0
     a0x, a0y, a0z, g0x, g0y, g0z, t0,
     # IMU 1
     a1x, a1y, a1z, g1x, g1y, g1z, t1) = struct.unpack_from(TELEM_FMT, payload)

    return {
        # FPGA status
        "fpga_sts0":      s0,
        "fpga_fc":        fc,
        "fpga_sc":        sc,
        "fpga_act":       act,
        "fpga_inputs":    inp,
        "fpga_version":   ver,
        "fpga_pchg_ms":   pchg_ms,
        # External ADC — v_vraw in 10mV units (÷100→V); 12V/24V in mV (÷1000→V)
        "v_vraw_v":       v_vraw_dv / 100.0,
        "v_12v_v":        v_12v     / 1000.0,
        "v_24v_v":        v_24v     / 1000.0,
        "i_vraw_sw_ma":   float(i_vraw_sw),
        "i_12v_ma":       float(i_12v),
        "i_24v_ma":       float(i_24v),
        "therm1_c":       th1_dc  * 0.1,   # PDU PCB thermal spot 1
        "therm2_c":       th2_dc  * 0.1,   # PDU PCB thermal spot 2
        # Energy meter (5 Hz via CAN 0x523 — values repeat on alternate telem frames)
        "em_i_ma":        float(em_i),
        "em_v_v":         em_v_raw / 100.0,  # 10mV units
        "em_p_w":         em_p_raw * 0.1,
        "em_t_c":         em_t_raw * 0.1,
        # STM32 local ADC
        "ladc_therm0_c":  lt0   * 0.1,    # RCU PCB NTC, next to IMU0
        "ladc_therm1_c":  lt1   * 0.1,    # PDU external connector (typically unpopulated)
        "ladc_therm2_c":  lt2   * 0.1,    # PDU onboard NTC (trusted)
        "ladc_vsource_v": lvsrc / 100.0,  # 10mV units — PDU firmware packs as ×100
        "ladc_vbus_v":    lvbus / 100.0,  # 10mV units
        "ladc_icoil_ma":  float(licoil),
        # IMU 0
        "imu0_ax_g":      a0x * IMU_ACCEL_G_PER_LSB,
        "imu0_ay_g":      a0y * IMU_ACCEL_G_PER_LSB,
        "imu0_az_g":      a0z * IMU_ACCEL_G_PER_LSB,
        "imu0_gx_dps":    g0x * IMU_GYRO_DPS_PER_LSB,
        "imu0_gy_dps":    g0y * IMU_GYRO_DPS_PER_LSB,
        "imu0_gz_dps":    g0z * IMU_GYRO_DPS_PER_LSB,
        "imu0_temp_c":    t0  * IMU_TEMP_SCALE + IMU_TEMP_OFFSET_C,
        # IMU 1
        "imu1_ax_g":      a1x * IMU_ACCEL_G_PER_LSB,
        "imu1_ay_g":      a1y * IMU_ACCEL_G_PER_LSB,
        "imu1_az_g":      a1z * IMU_ACCEL_G_PER_LSB,
        "imu1_gx_dps":    g1x * IMU_GYRO_DPS_PER_LSB,
        "imu1_gy_dps":    g1y * IMU_GYRO_DPS_PER_LSB,
        "imu1_gz_dps":    g1z * IMU_GYRO_DPS_PER_LSB,
        "imu1_temp_c":    t1  * IMU_TEMP_SCALE + IMU_TEMP_OFFSET_C,
    }


def _decode_motor_fb(payload: bytes) -> list:
    """
    Decode motor feedback payload.
    Uses 4-byte header (u8 count + 3 pad) from rcu_monitor.py.
    The old GUI used a 1-byte header which is incorrect.
    """
    if len(payload) < FB_HEADER_SIZE:
        return []
    (count,) = struct.unpack_from("<B", payload, 0)  # pad bytes discarded
    slots: list = []
    off = FB_HEADER_SIZE
    for _ in range(min(count, FB_SLOTS)):
        if off + FB_SLOT_SIZE > len(payload):
            break
        bus, mid, pos_u16, vel_u16, trq_u16, err = struct.unpack_from(
            FB_SLOT_FMT, payload, off)
        off += FB_SLOT_SIZE
        slots.append({
            "bus":       bus,
            "motor_id":  mid,
            "pos_rad":   u16_to_f(pos_u16, -RS04_POS_MAX, RS04_POS_MAX),
            "vel_rads":  u16_to_f(vel_u16, -RS04_VEL_MAX, RS04_VEL_MAX),
            "torque_nm": u16_to_f(trq_u16, -RS04_TRQ_MAX, RS04_TRQ_MAX),
            "error":     err,
        })
    return slots


def _decode_debug_reply(payload: bytes) -> dict | None:
    if len(payload) < DBG_REPLY_SIZE:
        return None
    (uptime, boot_rsr,
     imu0_v, imu1_v, fpga_v, rails_v, ssd_v, can_lb,
     hb_age) = struct.unpack_from(DBG_REPLY_FMT, payload)

    rst: list[str] = []
    if boot_rsr & (1 << 26): rst.append("PIN")
    if boot_rsr & (1 << 24): rst.append("SW")
    if boot_rsr & (1 << 21): rst.append("BOR")
    if boot_rsr & (1 << 20): rst.append("IWDG")
    if boot_rsr & (1 << 28): rst.append("WWDG")

    can_str = {0: "untested", 1: "R only", 2: "L only",
               3: "BOTH OK", 0xFF: "FAIL"}.get(can_lb, f"0x{can_lb:02X}")

    return {
        "uptime_ms":    uptime,
        "boot_rsr":     boot_rsr,
        "reset_causes": rst if rst else ["clean"],
        "imu0_ok":      bool(imu0_v),
        "imu1_ok":      bool(imu1_v),
        "fpga_ok":      bool(fpga_v),
        "rails_ok":     bool(rails_v),
        "em_ok":        bool(ssd_v),
        "can_lb":       can_lb,
        "can_lb_str":   can_str,
        "hb_age_ms":    hb_age,
    }


# ===========================================================================
# Command senders — all fire-and-forget UDP datagrams in background threads
# ===========================================================================

_seq_lock = threading.Lock()
_seq_val  = 0


def _next_seq() -> int:
    global _seq_val
    with _seq_lock:
        v        = _seq_val
        _seq_val = (_seq_val + 1) & 0xFF
        return v


def _send_udp(rcu_ip: str, payload: bytes, port: int = PORT_CMD) -> None:
    """Fire-and-forget UDP datagram. Silently ignores all errors."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(payload, (rcu_ip, port))
    except Exception:
        pass


def send_debug_cmd(rcu_ip: str, subcmd: int, extra: bytes = b"") -> None:
    """Send PKT_DEBUG_CMD with optional extra payload bytes."""
    body = bytes([subcmd]) + extra
    hdr  = struct.pack(HDR_FMT, PKT_MAGIC, PKT_DEBUG_CMD, _next_seq(), len(body))
    threading.Thread(
        target=_send_udp, args=(rcu_ip, hdr + body), daemon=True
    ).start()


def send_motor_cmd(rcu_ip: str, bus: int, motor_id: int,
                   pos: float, vel: float, trq: float,
                   kp: float, kd: float) -> None:
    """Send a single RS04 motor position/velocity/torque command."""
    pos_u16 = f_to_u16(pos, -RS04_POS_MAX, RS04_POS_MAX)
    vel_u16 = f_to_u16(vel, -RS04_VEL_MAX, RS04_VEL_MAX)
    trq_u16 = f_to_u16(trq, -RS04_TRQ_MAX, RS04_TRQ_MAX)
    kp_u8   = int(max(0, min(255, (kp / RS04_KP_MAX) * 255)))
    kd_u8   = int(max(0, min(255, (kd / RS04_KD_MAX) * 255)))
    entry   = struct.pack("<BBHHHBBxx",
                          bus, motor_id, pos_u16, vel_u16, trq_u16, kp_u8, kd_u8)
    hdr     = struct.pack(HDR_FMT, PKT_MAGIC, PKT_MOTOR_CMD, _next_seq(), len(entry))
    threading.Thread(
        target=_send_udp, args=(rcu_ip, hdr + entry), daemon=True
    ).start()


def send_motor_estop(rcu_ip: str,
                     active_ids: list[tuple[int, int]] | None = None) -> None:
    """
    Send zero-velocity / zero-torque commands to stop all motors.

    *active_ids*: list of (bus, motor_id) from recent motor_fb data.
    If None, sends to bus 0 and bus 1, IDs 1–8 (16 slots total).
    Uses existing PKT_MOTOR_CMD — no new firmware required.
    """
    if active_ids is None:
        active_ids = [(bus, mid) for bus in (0, 1) for mid in range(1, 9)]
    for bus, mid in active_ids:
        send_motor_cmd(rcu_ip, bus, mid, 0.0, 0.0, 0.0, 0.0, 0.0)


# ===========================================================================
# Phase 3 — TelemetryPlotPanel
# ===========================================================================

from collections import namedtuple

PlotGroupDef = namedtuple("PlotGroupDef", ["title", "y_label", "keys"])

PDU_PLOT_GROUPS: list[PlotGroupDef] = [
    PlotGroupDef("Voltages",     "V",   [
        "v_vraw_v", "v_12v_v", "v_24v_v",
        "ladc_vsource_v", "ladc_vbus_v", "em_v_v",
    ]),
    PlotGroupDef("Currents",     "mA",  [
        "i_vraw_sw_ma", "i_12v_ma", "i_24v_ma",
        "em_i_ma", "ladc_icoil_ma",
    ]),
    PlotGroupDef("Temperatures", "°C",  [
        "therm1_c", "therm2_c",
        "ladc_therm0_c", "ladc_therm1_c", "ladc_therm2_c", "em_t_c",
    ]),
]

IMU_PLOT_GROUPS: list[PlotGroupDef] = [
    PlotGroupDef("Acceleration", "g",   [
        "imu0_ax_g", "imu0_ay_g", "imu0_az_g", "imu0_mag",
        "imu1_ax_g", "imu1_ay_g", "imu1_az_g", "imu1_mag",
    ]),
    PlotGroupDef("Gyroscope",    "dps", [
        "imu0_gx_dps", "imu0_gy_dps", "imu0_gz_dps",
        "imu1_gx_dps", "imu1_gy_dps", "imu1_gz_dps",
    ]),
    PlotGroupDef("IMU Temps",    "°C",  [
        "imu0_temp_c", "imu1_temp_c",
    ]),
]

_TIME_WINDOW_LABELS = ["5 s", "10 s", "30 s", "1 min", "5 min", "10 min"]
_TIME_WINDOW_SEC    = [5,      10,     30,     60,      300,     600]


def _colored_square_label(color: str, size: int = 10) -> QLabel:
    """Return a small QLabel painted as a solid coloured square."""
    lbl = QLabel()
    lbl.setFixedSize(size, size)
    lbl.setStyleSheet(
        f"background-color: {color}; border: none; border-radius: 2px;"
    )
    return lbl


class TelemetryPlotPanel(QWidget):
    """
    A self-contained plot panel for one set of PlotGroupDefs.

    Layout
    ------
    Left: scrollable sidebar with:
      - One QGroupBox per group, each containing:
          QCheckBox + coloured square per signal in the group
      - Time window selector (QComboBox)
      - Pause / Resume toggle button
      - Export CSV button

    Right: pg.GraphicsLayoutWidget with one pg.PlotItem per group,
    stacked vertically and X-linked for synchronised panning/zooming.

    Parameters
    ----------
    groups     : list[PlotGroupDef]  — defines which signals to show
    state      : RcuState            — live telemetry source
    cfg        : Config              — non-volatile config
    cfg_key    : str                 — config sub-key for checkbox persistence
                                       ("pdu_signals_checked" or "imu_signals_checked")
    parent     : QWidget | None
    """

    def __init__(
        self,
        groups: list,
        state: "RcuState",
        cfg: "Config",
        cfg_key: str = "pdu_signals_checked",
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._groups   = groups
        self._state    = state
        self._cfg      = cfg
        self._cfg_key  = cfg_key
        self._paused   = False

        # All keys managed by this panel (preserves order)
        self._all_keys: list[str] = []
        for grp in groups:
            for k in grp.keys:
                if k not in self._all_keys:
                    self._all_keys.append(k)

        # checkbox widgets keyed by signal key
        self._chk: dict[str, QCheckBox] = {}

        # pyqtgraph PlotItem per group, PlotDataItem per (group, key)
        self._plot_items:  list[pg.PlotItem]   = []
        self._data_items:  dict[str, pg.PlotDataItem] = {}
        self._crosshairs:  list[pg.InfiniteLine] = []

        # floating tooltip label (overlaid on the GraphicsLayoutWidget)
        self._tooltip_lbl: QLabel | None = None
        # remember last x-values array for crosshair readout
        self._last_x: np.ndarray | None = None
        # {key: last_plotted_array} for crosshair readout
        self._last_y: dict[str, np.ndarray] = {}

        self._build_ui()
        self._restore_checkboxes()
        self._refresh_curve_visibility()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(6)

        # ---- Left sidebar -------------------------------------------------
        sidebar = QWidget()
        sidebar.setFixedWidth(220)
        sb_layout = QVBoxLayout(sidebar)
        sb_layout.setContentsMargins(4, 4, 4, 4)
        sb_layout.setSpacing(6)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        inner_layout = QVBoxLayout(inner)
        inner_layout.setContentsMargins(2, 2, 2, 2)
        inner_layout.setSpacing(4)

        for grp in self._groups:
            gb = QGroupBox(grp.title)
            gb_layout = QVBoxLayout(gb)
            gb_layout.setContentsMargins(6, 4, 6, 4)
            gb_layout.setSpacing(2)
            for key in grp.keys:
                lbl_text = _SIG_BY_KEY.get(key, (key, False, False))[0]
                color    = SIGNAL_COLORS.get(key, WHITE)
                row      = QHBoxLayout()
                row.setSpacing(4)
                sq = _colored_square_label(color, 10)
                chk = QCheckBox(lbl_text)
                chk.setObjectName(f"chk_{key}")
                chk.stateChanged.connect(
                    lambda _state, k=key: self._on_checkbox(k)
                )
                self._chk[key] = chk
                row.addWidget(sq)
                row.addWidget(chk)
                row.addStretch()
                gb_layout.addLayout(row)
            inner_layout.addWidget(gb)

        inner_layout.addStretch()
        scroll.setWidget(inner)
        sb_layout.addWidget(scroll)

        # Time window selector
        tw_row = QHBoxLayout()
        tw_row.setSpacing(6)
        tw_row.addWidget(QLabel("Time window:"))
        self._tw_combo = QComboBox()
        self._tw_combo.addItems(_TIME_WINDOW_LABELS)
        # restore last used value
        default_s = self._cfg.get("display", "default_time_window_s", default=30)
        best_idx  = 2  # 30 s fallback
        for i, s in enumerate(_TIME_WINDOW_SEC):
            if s == default_s:
                best_idx = i
                break
        self._tw_combo.setCurrentIndex(best_idx)
        tw_row.addWidget(self._tw_combo)
        sb_layout.addLayout(tw_row)

        # Pause / Resume
        self._pause_btn = QPushButton("⏸  Pause")
        self._pause_btn.clicked.connect(self._on_pause_toggle)
        sb_layout.addWidget(self._pause_btn)

        # Export CSV
        export_btn = QPushButton("⬇  Export CSV")
        export_btn.clicked.connect(self._on_export_csv)
        sb_layout.addWidget(export_btn)

        root.addWidget(sidebar)

        # ---- Right: pyqtgraph plots ---------------------------------------
        self._glw = pg.GraphicsLayoutWidget()
        self._glw.setBackground(BG)

        # Overlay label for crosshair tooltip
        self._tooltip_lbl = QLabel("", self._glw)
        self._tooltip_lbl.setStyleSheet(
            f"background: {PANEL}; color: {TEXT}; border: 1px solid {DIM};"
            " padding: 3px 6px; font-size: 10px; border-radius: 3px;"
        )
        self._tooltip_lbl.setVisible(False)
        self._tooltip_lbl.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

        n_groups = len(self._groups)
        for gi, grp in enumerate(self._groups):
            plot = self._glw.addPlot(row=gi, col=0)
            plot.setLabel("left", grp.y_label,
                          **{"color": TEXT, "font-size": "10px"})
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.getAxis("bottom").setStyle(showValues=(gi == n_groups - 1))
            plot.getAxis("bottom").setPen(pg.mkPen(DIM))
            plot.getAxis("left").setPen(pg.mkPen(DIM))
            plot.getAxis("left").setTextPen(pg.mkPen(TEXT))
            plot.getAxis("bottom").setTextPen(pg.mkPen(TEXT))
            plot.setMinimumHeight(110)
            plot.setMenuEnabled(False)
            plot.hideButtons()

            # X-link all plots to the first one
            if gi > 0:
                plot.setXLink(self._plot_items[0])

            # Title inside the plot (top-left legend area)
            title_lbl = pg.LabelItem(
                grp.title, color=ACCENT, size="10pt"
            )
            title_lbl.setParentItem(plot)

            self._plot_items.append(plot)

            # Create PlotDataItems
            lw = self._cfg.get("plots", "line_width", default=1.5)
            for key in grp.keys:
                color = SIGNAL_COLORS.get(key, WHITE)
                curve = plot.plot(
                    x=np.array([], dtype=np.float64),
                    y=np.array([], dtype=np.float64),
                    pen=pg.mkPen(color, width=lw),
                    connect="finite",  # draws NaN gaps cleanly
                    name=_SIG_BY_KEY.get(key, (key,))[0],
                )
                self._data_items[key] = curve

            # Vertical crosshair for this plot
            ch = pg.InfiniteLine(angle=90, movable=False,
                                 pen=pg.mkPen(DIM, width=1, style=Qt.PenStyle.DashLine))
            ch.setVisible(False)
            plot.addItem(ch, ignoreBounds=True)
            self._crosshairs.append(ch)

            # Mouse tracking
            plot.scene().sigMouseMoved.connect(self._on_mouse_moved)

        root.addWidget(self._glw, stretch=1)

    # ------------------------------------------------------------------
    # Checkbox / config helpers
    # ------------------------------------------------------------------

    def _restore_checkboxes(self) -> None:
        saved = self._cfg.get("plots", self._cfg_key) or {}
        for key, chk in self._chk.items():
            if key in saved:
                chk.setChecked(bool(saved[key]))
            else:
                # fall back to TELEM_SIGNALS default for this panel type
                sig_info = _SIG_BY_KEY.get(key)
                if sig_info:
                    default = (sig_info[1] if self._cfg_key == "pdu_signals_checked"
                               else sig_info[2])
                    chk.setChecked(default)

    def _on_checkbox(self, key: str) -> None:
        val = self._chk[key].isChecked()
        self._cfg.set("plots", self._cfg_key, key, val)
        self._refresh_curve_visibility()

    def _refresh_curve_visibility(self) -> None:
        for key, curve in self._data_items.items():
            chk = self._chk.get(key)
            visible = chk.isChecked() if chk else True
            curve.setVisible(visible)

    # ------------------------------------------------------------------
    # Pause / Resume
    # ------------------------------------------------------------------

    def _on_pause_toggle(self) -> None:
        self._paused = not self._paused
        self._pause_btn.setText("▶  Resume" if self._paused else "⏸  Pause")

    def is_paused(self) -> bool:
        return self._paused

    # ------------------------------------------------------------------
    # Main update — called by the 100 ms QTimer
    # ------------------------------------------------------------------

    def update_plots(self) -> None:
        if self._paused:
            return

        tw_idx    = self._tw_combo.currentIndex()
        window_s  = _TIME_WINDOW_SEC[tw_idx]
        n         = int(window_s / 0.1)  # telem at 10 Hz → 0.1 s/sample
        n         = max(1, min(n, HISTORY_LEN))

        x = np.linspace(-window_s, 0.0, n)
        self._last_x = x

        for key, curve in self._data_items.items():
            if not curve.isVisible():
                continue
            raw = self._state.get_hist_window(key, n)
            y   = np.asarray(raw, dtype=np.float64)
            self._last_y[key] = y
            curve.setData(x=x, y=y)

        # Refit Y axes per plot to only visible curves
        for gi, plot in enumerate(self._plot_items):
            plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

    # ------------------------------------------------------------------
    # Crosshair mouse tracking
    # ------------------------------------------------------------------

    def _on_mouse_moved(self, scene_pos) -> None:
        if self._last_x is None or len(self._last_x) == 0:
            return

        for gi, plot in enumerate(self._plot_items):
            vb = plot.getViewBox()
            if not vb.sceneBoundingRect().contains(scene_pos):
                continue

            mouse_pt = vb.mapSceneToView(scene_pos)
            mx = mouse_pt.x()

            # Snap to nearest sample
            idx = int(np.searchsorted(self._last_x, mx))
            idx = max(0, min(idx, len(self._last_x) - 1))

            # Move all crosshairs to the same x
            for ch in self._crosshairs:
                ch.setPos(self._last_x[idx])
                ch.setVisible(True)

            # Build tooltip text
            lines: list[str] = [f"t = {self._last_x[idx]:.1f} s"]
            for key in self._groups[gi].keys:
                chk = self._chk.get(key)
                if chk and not chk.isChecked():
                    continue
                y_arr = self._last_y.get(key)
                if y_arr is None or idx >= len(y_arr):
                    continue
                val = y_arr[idx]
                lbl = _SIG_BY_KEY.get(key, (key,))[0]
                color = SIGNAL_COLORS.get(key, WHITE)
                lines.append(
                    f'<span style="color:{color}">{lbl}: '
                    f'{_fmt_val(val, ".3f")}</span>'
                )

            if self._tooltip_lbl is not None:
                self._tooltip_lbl.setText("<br>".join(lines))
                self._tooltip_lbl.adjustSize()
                # Position tooltip near the mouse, keeping it within the widget
                sp = self._glw.mapFromScene(scene_pos)
                tx = int(sp.x()) + 14
                ty = int(sp.y()) - 10
                tw = self._tooltip_lbl.width()
                th = self._tooltip_lbl.height()
                gw = self._glw.width()
                gh = self._glw.height()
                if tx + tw > gw - 4:
                    tx = int(sp.x()) - tw - 14
                if ty + th > gh - 4:
                    ty = gh - th - 4
                self._tooltip_lbl.move(max(0, tx), max(0, ty))
                self._tooltip_lbl.setVisible(True)
            return

        # Mouse left all plots
        for ch in self._crosshairs:
            ch.setVisible(False)
        if self._tooltip_lbl is not None:
            self._tooltip_lbl.setVisible(False)

    # ------------------------------------------------------------------
    # Export CSV — visible signals only
    # ------------------------------------------------------------------

    def _on_export_csv(self) -> None:
        default_dir = self._cfg.get("recording", "default_dir", default="")
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Export Plot Data as CSV",
            str(pathlib.Path(default_dir) / "plot_export.csv") if default_dir else "",
            "CSV Files (*.csv);;All Files (*)",
        )
        if not filepath:
            return

        visible_keys = [
            k for k in self._all_keys
            if self._chk.get(k, QCheckBox()).isChecked()
        ]
        if not visible_keys:
            QMessageBox.warning(self, "Export", "No signals selected for export.")
            return

        tw_idx   = self._tw_combo.currentIndex()
        window_s = _TIME_WINDOW_SEC[tw_idx]
        n        = int(window_s / 0.1)
        n        = max(1, min(n, HISTORY_LEN))
        x        = np.linspace(-window_s, 0.0, n)

        try:
            with open(filepath, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                header = ["time_s"] + [
                    _SIG_BY_KEY.get(k, (k,))[0] for k in visible_keys
                ]
                writer.writerow(header)
                cols = {
                    k: np.asarray(self._state.get_hist_window(k, n), dtype=np.float64)
                    for k in visible_keys
                }
                for i in range(n):
                    row = [f"{x[i]:.3f}"]
                    for k in visible_keys:
                        v = cols[k][i]
                        row.append("" if math.isnan(v) else f"{v:.6g}")
                    writer.writerow(row)
        except Exception as exc:
            QMessageBox.critical(self, "Export Error", str(exc))
            return

        # Remember directory for next time
        self._cfg.set("recording", "default_dir",
                      str(pathlib.Path(filepath).parent))


# ===========================================================================
# Phase 4 — MainWindow + all 10 tabs
# ===========================================================================

# ---------------------------------------------------------------------------
# Shared small widgets
# ---------------------------------------------------------------------------

def _sep() -> QFrame:
    """Horizontal separator line."""
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFrameShadow(QFrame.Shadow.Sunken)
    f.setStyleSheet(f"background-color: {DIM}; max-height: 1px; border: none;")
    return f


def _hdg(text: str, color: str = ACCENT) -> QLabel:
    """Small heading label."""
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color: {color}; font-weight: bold; font-size: 12px;"
        " background: transparent;"
    )
    return lbl


def _val_label(text: str = "---") -> QLabel:
    """Monospace value display label."""
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color: {WHITE}; font-family: 'Consolas','Courier New',monospace;"
        " font-size: 11px; background: transparent;"
    )
    return lbl


def _health_dot(ok: bool | None = None) -> QLabel:
    """Coloured circle: green=ok, red=fail, grey=unknown."""
    if ok is None:
        color = DIM
    elif ok:
        color = OK
    else:
        color = ERROR
    lbl = QLabel("●")
    lbl.setStyleSheet(f"color: {color}; font-size: 14px; background: transparent;")
    return lbl


def _make_form_row(form: QFormLayout, key_lbl: str, val: QLabel) -> None:
    k = QLabel(key_lbl)
    k.setStyleSheet(f"color: {DIM}; background: transparent;")
    form.addRow(k, val)


# ---------------------------------------------------------------------------
# Tab 1 — Overview
# ---------------------------------------------------------------------------

class OverviewTab(QWidget):
    """
    Quick-glance health panel:
      • Connection + packet stats
      • Subsystem health dots (IMU0, IMU1, FPGA, Rails, EM)
      • FPGA state + STATUS0 / INPUTS register decode
      • Quick-fire command buttons (Ping, Force Telem, Buzz, LED Blink, CAN Loopback)
    """

    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        # ---- Row 1: Connection + packet stats ----------------------------
        top = QHBoxLayout()
        top.setSpacing(12)

        conn_gb = QGroupBox("Connection")
        conn_form = QFormLayout(conn_gb)
        conn_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_ip       = _val_label()
        self._lbl_conn     = _val_label()
        self._lbl_pps      = _val_label()
        self._lbl_telem_age = _val_label()
        self._lbl_seq_gaps = _val_label()
        _make_form_row(conn_form, "RCU IP:",        self._lbl_ip)
        _make_form_row(conn_form, "State:",         self._lbl_conn)
        _make_form_row(conn_form, "Pkts/s:",        self._lbl_pps)
        _make_form_row(conn_form, "Telem age:",     self._lbl_telem_age)
        _make_form_row(conn_form, "Seq gaps:",      self._lbl_seq_gaps)
        top.addWidget(conn_gb)

        # Health dots
        health_gb = QGroupBox("Subsystem Health")
        health_grid = QGridLayout(health_gb)
        health_grid.setSpacing(6)
        labels_  = ["IMU 0", "IMU 1", "FPGA", "Rails", "Energy Meter",
                    "CAN Loopback"]
        self._health_dots:  dict[str, QLabel] = {}
        self._health_texts: dict[str, QLabel] = {}
        for col, name in enumerate(labels_):
            dot = _health_dot(None)
            txt = QLabel(name)
            txt.setStyleSheet(f"color: {TEXT}; font-size: 10px; background: transparent;")
            txt.setAlignment(Qt.AlignmentFlag.AlignHCenter)
            health_grid.addWidget(dot, 0, col, Qt.AlignmentFlag.AlignHCenter)
            health_grid.addWidget(txt, 1, col, Qt.AlignmentFlag.AlignHCenter)
            self._health_dots[name]  = dot
            self._health_texts[name] = txt
        top.addWidget(health_gb)

        # RCU uptime / reset from ping reply
        ping_gb = QGroupBox("RCU Info (last ping)")
        ping_form = QFormLayout(ping_gb)
        ping_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_uptime = _val_label()
        self._lbl_reset  = _val_label()
        self._lbl_hbage  = _val_label()
        _make_form_row(ping_form, "Uptime:",       self._lbl_uptime)
        _make_form_row(ping_form, "Reset cause:",  self._lbl_reset)
        _make_form_row(ping_form, "HB age:",       self._lbl_hbage)
        top.addWidget(ping_gb)

        root.addLayout(top)
        root.addWidget(_sep())

        # ---- Row 2: FPGA state + register bits ---------------------------
        fpga_row = QHBoxLayout()
        fpga_row.setSpacing(12)

        fpga_state_gb = QGroupBox("FPGA State")
        fs_form = QFormLayout(fpga_state_gb)
        fs_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_fpga_state    = _val_label()
        self._lbl_fpga_fc       = _val_label()
        self._lbl_fpga_sc       = _val_label()
        self._lbl_pchg_ms       = _val_label()
        self._lbl_fpga_ver      = _val_label()
        _make_form_row(fs_form, "State:",      self._lbl_fpga_state)
        _make_form_row(fs_form, "Fault cnt:",  self._lbl_fpga_fc)
        _make_form_row(fs_form, "Supv cnt:",   self._lbl_fpga_sc)
        _make_form_row(fs_form, "Pchg ms:",    self._lbl_pchg_ms)
        _make_form_row(fs_form, "FPGA ver:",   self._lbl_fpga_ver)
        fpga_row.addWidget(fpga_state_gb)

        status_gb = QGroupBox("STATUS0 Register")
        status_form = QFormLayout(status_gb)
        status_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_s0_raw  = _val_label()
        self._lbl_s0_bits = _val_label()
        self._lbl_s0_bits.setWordWrap(True)
        _make_form_row(status_form, "Raw:",  self._lbl_s0_raw)
        _make_form_row(status_form, "Bits:", self._lbl_s0_bits)
        fpga_row.addWidget(status_gb)

        inputs_gb = QGroupBox("INPUTS Register")
        inputs_form = QFormLayout(inputs_gb)
        inputs_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_in_raw  = _val_label()
        self._lbl_in_bits = _val_label()
        self._lbl_in_bits.setWordWrap(True)
        _make_form_row(inputs_form, "Raw:",  self._lbl_in_raw)
        _make_form_row(inputs_form, "Bits:", self._lbl_in_bits)
        fpga_row.addWidget(inputs_gb)

        fpga_row.addStretch()
        root.addLayout(fpga_row)
        root.addWidget(_sep())

        # ---- Row 3: Quick commands ----------------------------------------
        cmd_gb = QGroupBox("Quick Commands")
        cmd_layout = QHBoxLayout(cmd_gb)
        cmd_layout.setSpacing(8)

        def _qb(text: str, cb, role: str = "") -> QPushButton:
            btn = QPushButton(text)
            if role:
                btn.setProperty("role", role)
            btn.clicked.connect(cb)
            return btn

        cmd_layout.addWidget(_qb("Ping",         self._cmd_ping))
        cmd_layout.addWidget(_qb("Force Telem",  self._cmd_force_telem))
        cmd_layout.addWidget(_qb("Buzz",         self._cmd_buzz))
        cmd_layout.addWidget(_qb("LED Blink",    self._cmd_led))
        cmd_layout.addWidget(_qb("CAN Loopback", self._cmd_can_lb))
        cmd_layout.addStretch()
        root.addWidget(cmd_gb)

        root.addStretch()

    # ------------------------------------------------------------------
    # Command callbacks
    # ------------------------------------------------------------------

    def _rcu_ip(self) -> str:
        return self._cfg.get("connection", "rcu_ip", default=RCU_IP_DEFAULT)

    def _cmd_ping(self)       -> None: send_debug_cmd(self._rcu_ip(), DBGCMD_PING)
    def _cmd_force_telem(self)-> None: send_debug_cmd(self._rcu_ip(), DBGCMD_FORCE_TELEM)
    def _cmd_buzz(self)       -> None: send_debug_cmd(self._rcu_ip(), DBGCMD_BUZZ)
    def _cmd_led(self)        -> None: send_debug_cmd(self._rcu_ip(), DBGCMD_LED_BLINK)
    def _cmd_can_lb(self)     -> None: send_debug_cmd(self._rcu_ip(), DBGCMD_CAN_LOOPBACK)

    # ------------------------------------------------------------------
    # Live update
    # ------------------------------------------------------------------

    def refresh(self) -> None:
        t    = self._state.get_telem()
        ping = self._state.get_ping()
        seq  = self._state.seq_tracker

        self._lbl_ip.setText(self._cfg.get("connection", "rcu_ip", default="---"))
        conn = self._state.is_connected()
        self._lbl_conn.setText(
            f'<span style="color:{OK}">Connected</span>' if conn
            else f'<span style="color:{ERROR}">No socket</span>'
        )
        self._lbl_conn.setTextFormat(Qt.TextFormat.RichText)
        self._lbl_pps.setText(str(self._state.pkts_per_sec()))
        age = self._state.telem_age_ms()
        age_color = OK if age < 500 else WARN if age < 2000 else ERROR
        self._lbl_telem_age.setText(
            f'<span style="color:{age_color}">{age} ms</span>'
        )
        self._lbl_telem_age.setTextFormat(Qt.TextFormat.RichText)
        self._lbl_seq_gaps.setText(f"{seq.gaps} / {seq.total}")

        # Health dots from last ping
        if ping:
            for key, field in (
                ("IMU 0",         "imu0_ok"),
                ("IMU 1",         "imu1_ok"),
                ("FPGA",          "fpga_ok"),
                ("Rails",         "rails_ok"),
                ("Energy Meter",  "em_ok"),
            ):
                ok = ping.get(field)
                dot = self._health_dots[key]
                dot.setText("●")
                dot.setStyleSheet(
                    f"color: {OK if ok else ERROR};"
                    " font-size: 14px; background: transparent;"
                )
            can_ok = ping.get("can_lb", 0) == 3
            dot = self._health_dots["CAN Loopback"]
            dot.setText("●")
            dot.setStyleSheet(
                f"color: {OK if can_ok else (WARN if ping.get('can_lb',0)==0 else ERROR)};"
                " font-size: 14px; background: transparent;"
            )
            uptime_s = ping.get("uptime_ms", 0) / 1000.0
            h  = int(uptime_s // 3600)
            m  = int((uptime_s % 3600) // 60)
            s  = int(uptime_s % 60)
            self._lbl_uptime.setText(f"{h:02d}:{m:02d}:{s:02d}")
            self._lbl_reset.setText(", ".join(ping.get("reset_causes", ["---"])))
            self._lbl_hbage.setText(f"{ping.get('hb_age_ms', '---')} ms")

        # FPGA
        if t:
            state_num = t.get("fpga_act", 0)
            self._lbl_fpga_state.setText(
                FPGA_STATES.get(state_num, f"UNKNOWN ({state_num})")
            )
            self._lbl_fpga_fc.setText(str(t.get("fpga_fc", "---")))
            self._lbl_fpga_sc.setText(str(t.get("fpga_sc", "---")))
            self._lbl_pchg_ms.setText(str(t.get("fpga_pchg_ms", "---")))
            self._lbl_fpga_ver.setText(str(t.get("fpga_version", "---")))

            s0  = t.get("fpga_sts0",   0)
            inp = t.get("fpga_inputs", 0)
            self._lbl_s0_raw.setText(f"0x{s0:02X}  ({s0:08b}b)")
            self._lbl_s0_bits.setText(decode_bits(s0, S0_BITS))
            self._lbl_in_raw.setText(f"0x{inp:02X}  ({inp:08b}b)")
            self._lbl_in_bits.setText(decode_bits(inp, IN_BITS))


# ---------------------------------------------------------------------------
# Tab 2 — Power / Cross-Reference
# ---------------------------------------------------------------------------

_CROSS_REF_GROUPS: list[tuple[str, list[tuple[str, str]]]] = [
    # (group name, [(source label, telem key), ...])
    ("V_SOURCE (~53V)",   [("Ext ADC",    "v_vraw_v"),
                            ("Energy Meter","em_v_v"),
                            ("Local ADC",  "ladc_vsource_v")]),
    ("12V_SW",            [("Ext ADC",    "v_12v_v"),
                            ("Local ADC",  "ladc_vbus_v")]),
    ("24V_SW",            [("Ext ADC",    "v_24v_v")]),
    ("Source Current",    [("Ext ADC",    "i_vraw_sw_ma"),
                            ("Energy Meter","em_i_ma")]),
    ("12V_SW Current",    [("Ext ADC",    "i_12v_ma")]),
    ("24V_SW Current",    [("Ext ADC",    "i_24v_ma")]),
    ("Total Power",       [("Energy Meter","em_p_w")]),
    ("Board Temps (°C)",  [("PDU Therm 1","therm1_c"),
                            ("PDU Therm 2","therm2_c"),
                            ("PDU Board NTC","ladc_therm2_c"),
                            ("EM Temp",    "em_t_c"),
                            ("IMU0 Temp",  "imu0_temp_c"),
                            ("IMU1 Temp",  "imu1_temp_c"),
                            ("RCU NTC",    "ladc_therm0_c"),
                            ("PDU Ext Therm","ladc_therm1_c")]),
]


def _agreement_text(vals: list[float], is_temp: bool) -> tuple[str, str]:
    """
    Returns (text, colour) agreement indicator.

    Voltage agreement: (max-min)/max > 2% → WARN
    Temperature agreement: max-min > 5 °C → WARN
    """
    valid = [v for v in vals if not math.isnan(v)]
    if len(valid) < 2:
        return ("(insufficient data)", DIM)
    lo, hi = min(valid), max(valid)
    if is_temp:
        spread = hi - lo
        if spread > 5.0:
            return (f"⚠  Δ{spread:.1f}°C", WARN)
    else:
        if hi > 0:
            pct = (hi - lo) / hi * 100.0
            if pct > 2.0:
                return (f"⚠  {pct:.1f}%", WARN)
    return ("✓ agree", OK)


class PowerCrossRefTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._val_labels:  list[list[QLabel]] = []
        self._agree_labels: list[QLabel] = []
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(6)
        root.addWidget(_hdg("Power / Cross-Reference"))
        root.addWidget(QLabel(
            "Compares the same electrical quantity from multiple measurement sources. "
            "⚠ indicates significant disagreement.",
            styleSheet=f"color:{DIM}; background:transparent; font-size:10px;"
        ))
        root.addWidget(_sep())

        table = QTableWidget()
        table.setColumnCount(4)
        table.setHorizontalHeaderLabels(["Group", "Source", "Value", "Agreement"])
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setAlternatingRowColors(True)

        self._table = table
        row_idx = 0
        self._row_map: list[tuple[str, str, bool]] = []  # (group, key, is_first_in_group)

        for group_name, sources in _CROSS_REF_GROUPS:
            is_temp = "Temp" in group_name or "°C" in group_name
            for si, (src_lbl, key) in enumerate(sources):
                self._row_map.append((group_name, key, is_temp))
                row_idx += 1

        table.setRowCount(row_idx)

        row_idx = 0
        for group_name, sources in _CROSS_REF_GROUPS:
            is_temp = "Temp" in group_name or "°C" in group_name
            grp_val_labels: list[QLabel] = []
            agree_lbl = _val_label("---")
            span_start = row_idx

            for si, (src_lbl, key) in enumerate(sources):
                # Group cell (merged via span logic using first row of group)
                grp_item = QTableWidgetItem(group_name if si == 0 else "")
                grp_item.setForeground(pg.mkColor(ACCENT))
                table.setItem(row_idx, 0, grp_item)

                table.setItem(row_idx, 1, QTableWidgetItem(src_lbl))

                val_lbl = _val_label("---")
                table.setCellWidget(row_idx, 2, val_lbl)
                grp_val_labels.append(val_lbl)

                if si == 0:
                    table.setCellWidget(row_idx, 3, agree_lbl)
                    self._agree_labels.append(agree_lbl)
                    self._val_labels.append(grp_val_labels)

                row_idx += 1

        root.addWidget(table)
        root.addStretch()

        # Store for refresh
        self._cross_groups = _CROSS_REF_GROUPS

    def refresh(self) -> None:
        t = self._state.get_telem()
        if not t:
            return

        row_idx = 0
        ai = 0
        for group_name, sources in self._cross_groups:
            is_temp = "Temp" in group_name or "°C" in group_name
            vals: list[float] = []
            for si, (src_lbl, key) in enumerate(sources):
                raw = t.get(key, float('nan'))
                if math.isnan(raw) and key == "ladc_therm1_c":
                    # suppress no-sensor channel
                    raw = float('nan')
                vals.append(raw)

            # Update value cells
            vl_grp = self._val_labels[ai] if ai < len(self._val_labels) else []
            for vi, v in enumerate(vals):
                if vi < len(vl_grp):
                    vl_grp[vi].setText(_fmt_val(v, ".3f"))

            # Agreement indicator
            if ai < len(self._agree_labels):
                agree_text, agree_color = _agreement_text(vals, is_temp)
                lbl = self._agree_labels[ai]
                lbl.setText(agree_text)
                lbl.setStyleSheet(
                    f"color: {agree_color}; background: transparent;"
                    " font-family: 'Consolas','Courier New',monospace;"
                    " font-size: 11px;"
                )
            ai += 1
            row_idx += len(sources)


# ---------------------------------------------------------------------------
# Tab 3 — PDU
# ---------------------------------------------------------------------------

class PduTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _build(self) -> None:
        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(splitter)

        # ---- Left: PDU detail panel -----------------------------------------
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(10, 8, 6, 8)
        lv.setSpacing(8)
        lv.addWidget(_hdg("PDU Detail"))

        # Ext ADC
        ext_gb = QGroupBox("External ADC")
        ext_form = QFormLayout(ext_gb)
        ext_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbls: dict[str, QLabel] = {}

        def _add(form, key, name):
            lbl = _val_label()
            self._lbls[key] = lbl
            _make_form_row(form, name, lbl)

        _add(ext_form, "v_vraw_v",     "V_SRC:")
        _add(ext_form, "v_12v_v",      "12V_SW:")
        _add(ext_form, "v_24v_v",      "24V_SW:")
        _add(ext_form, "i_vraw_sw_ma", "I_SRC (mA):")
        _add(ext_form, "i_12v_ma",     "I_12V_SW (mA):")
        _add(ext_form, "i_24v_ma",     "I_24V_SW (mA):")
        lv.addWidget(ext_gb)

        # Temperatures
        therm_gb = QGroupBox("Temperatures")
        therm_form = QFormLayout(therm_gb)
        therm_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        _add(therm_form, "therm1_c",       "PDU Therm 1:")
        _add(therm_form, "therm2_c",       "PDU Therm 2:")
        _add(therm_form, "ladc_therm2_c",  "PDU Board NTC:")
        _add(therm_form, "ladc_therm0_c",  "RCU Board NTC:")
        _add(therm_form, "ladc_therm1_c",  "PDU Ext Therm:")
        lv.addWidget(therm_gb)

        # Energy Meter
        em_gb = QGroupBox("Energy Meter  (5 Hz, RS485)")
        em_form = QFormLayout(em_gb)
        em_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        _add(em_form, "em_v_v",   "EM Voltage:")
        _add(em_form, "em_i_ma",  "EM Current (mA):")
        _add(em_form, "em_p_w",   "EM Power (W):")
        _add(em_form, "em_t_c",   "EM Temp (°C):")
        lv.addWidget(em_gb)

        # Local ADC
        ladc_gb = QGroupBox("Local STM32 ADC")
        ladc_form = QFormLayout(ladc_gb)
        ladc_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        _add(ladc_form, "ladc_vsource_v", "V_SRC (loc):")
        _add(ladc_form, "ladc_vbus_v",    "12V (loc):")
        _add(ladc_form, "ladc_icoil_ma",  "I_COIL (mA):")
        lv.addWidget(ladc_gb)
        lv.addStretch()

        splitter.addWidget(left)

        # ---- Right: plot panel -----------------------------------------------
        self._plot_panel = TelemetryPlotPanel(
            PDU_PLOT_GROUPS, self._state, self._cfg,
            cfg_key="pdu_signals_checked",
        )
        splitter.addWidget(self._plot_panel)
        splitter.setSizes([240, 700])

    def refresh(self) -> None:
        t = self._state.get_telem()
        if not t:
            return
        cfg = self._cfg.get

        alert = cfg("alerts")

        def _set(key, fmt, suffix="", lo_w=None, hi_w=None):
            v = t.get(key, float('nan'))
            if math.isnan(v):
                # PDU Ext Therm — no-sensor detection
                if key == "ladc_therm1_c" and _is_no_sensor(v):
                    self._lbls[key].setText("--- (no sensor)")
                    self._lbls[key].setStyleSheet(
                        f"color:{DIM}; background:transparent;"
                        " font-family:'Consolas','Courier New',monospace; font-size:11px;"
                    )
                    return
                self._lbls[key].setText("---")
                self._lbls[key].setStyleSheet(
                    f"color:{TEXT}; background:transparent;"
                    " font-family:'Consolas','Courier New',monospace; font-size:11px;"
                )
                return
            text = _fmt_val(v, fmt, suffix)
            if lo_w is not None and hi_w is not None:
                color = _color_label(v, lo_w, hi_w)
            else:
                color = TEXT
            self._lbls[key].setText(text)
            self._lbls[key].setStyleSheet(
                f"color:{color}; background:transparent;"
                " font-family:'Consolas','Courier New',monospace; font-size:11px;"
            )

        a = self._cfg.get("alerts")
        _set("v_vraw_v",     ".2f", " V",  a.get("v_src_warn_min_v", 45.0), a.get("v_src_warn_max_v", 58.0))
        _set("v_12v_v",      ".3f", " V",  a.get("v_12v_warn_min_v", 11.0), a.get("v_12v_warn_max_v", 13.0))
        _set("v_24v_v",      ".3f", " V",  a.get("v_24v_warn_min_v", 22.0), a.get("v_24v_warn_max_v", 26.0))
        _set("i_vraw_sw_ma", ".1f", " mA")
        _set("i_12v_ma",     ".1f", " mA")
        _set("i_24v_ma",     ".1f", " mA")
        tw = a.get("temp_warn_max_c", 55.0)
        te = a.get("temp_err_max_c",  70.0)
        for k in ("therm1_c", "therm2_c", "ladc_therm0_c",
                  "ladc_therm1_c", "ladc_therm2_c"):
            _set(k, ".1f", " °C", -40.0, tw)
        _set("em_v_v",   ".2f", " V",  a.get("v_src_warn_min_v", 45.0), a.get("v_src_warn_max_v", 58.0))
        _set("em_i_ma",  ".1f", " mA")
        _set("em_p_w",   ".1f", " W")
        _set("em_t_c",   ".1f", " °C", -40.0, tw)
        _set("ladc_vsource_v", ".2f", " V", a.get("v_src_warn_min_v", 45.0), a.get("v_src_warn_max_v", 58.0))
        _set("ladc_vbus_v",    ".3f", " V", a.get("v_12v_warn_min_v", 11.0), a.get("v_12v_warn_max_v", 13.0))
        _set("ladc_icoil_ma",  ".1f", " mA")

    def update_plots(self) -> None:
        self._plot_panel.update_plots()


# ---------------------------------------------------------------------------
# Tab 4 — FPGA / Health
# ---------------------------------------------------------------------------

class FpgaHealthTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(10)

        # ---- Left: register decode + fault log ----
        left = QVBoxLayout()
        left.setSpacing(8)
        left.addWidget(_hdg("FPGA / Health"))

        # FPGA live registers
        reg_gb = QGroupBox("FPGA Registers")
        reg_form = QFormLayout(reg_gb)
        reg_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_state   = _val_label()
        self._lbl_s0      = _val_label()
        self._lbl_in      = _val_label()
        self._lbl_fc      = _val_label()
        self._lbl_sc      = _val_label()
        self._lbl_act     = _val_label()
        self._lbl_pchg    = _val_label()
        self._lbl_ver     = _val_label()
        for nm, lbl in (
            ("State:",    self._lbl_state),
            ("STATUS0:",  self._lbl_s0),
            ("INPUTS:",   self._lbl_in),
            ("Fault cnt:",self._lbl_fc),
            ("Supv cnt:", self._lbl_sc),
            ("Act byte:", self._lbl_act),
            ("Pchg ms:",  self._lbl_pchg),
            ("FW ver:",   self._lbl_ver),
        ):
            _make_form_row(reg_form, nm, lbl)
        left.addWidget(reg_gb)

        # Supervision event log
        supv_gb = QGroupBox("Supervision Events (newest first)")
        supv_vl = QVBoxLayout(supv_gb)
        self._supv_list = QTextEdit()
        self._supv_list.setReadOnly(True)
        self._supv_list.setMaximumHeight(200)
        supv_vl.addWidget(self._supv_list)
        left.addWidget(supv_gb)
        left.addStretch()

        lw = QWidget()
        lw.setLayout(left)
        lw.setFixedWidth(340)
        root.addWidget(lw)

        # ---- Right: STATUS0 / INPUTS bit sparklines ----
        right = QVBoxLayout()
        right.setSpacing(6)
        right.addWidget(_hdg("STATUS0 / INPUTS Bit History (sparklines)"))

        self._spark_glw = pg.GraphicsLayoutWidget()
        self._spark_glw.setBackground(BG)
        self._spark_plots: dict[str, pg.PlotDataItem] = {}
        self._spark_items: list[pg.PlotItem] = []

        all_bits = [
            (f"S0:{name}", "s0", bit) for bit, name in S0_BITS
        ] + [
            (f"IN:{name}", "in", bit) for bit, name in IN_BITS
        ]

        for ri, (label, reg, bit) in enumerate(all_bits):
            p = self._spark_glw.addPlot(row=ri, col=0)
            p.hideAxis("bottom")
            p.setLabel("left", label, **{"color": TEXT, "font-size": "8pt"})
            p.setMaximumHeight(28)
            p.setYRange(-0.1, 1.1)
            p.getAxis("left").setStyle(showValues=False)
            p.showGrid(x=False, y=False)
            p.hideButtons()
            p.setMenuEnabled(False)
            if ri > 0:
                p.setXLink(self._spark_items[0])
            curve = p.plot(
                x=np.array([], dtype=np.float64),
                y=np.array([], dtype=np.float64),
                pen=pg.mkPen(ACCENT, width=1),
                connect="finite",
                stepMode="right",
            )
            self._spark_plots[f"{reg}_{bit}"] = curve
            self._spark_items.append(p)

        # Store bit list for refresh
        self._all_bits = all_bits

        right.addWidget(self._spark_glw)
        rw = QWidget()
        rw.setLayout(right)
        root.addWidget(rw, stretch=1)

    def refresh(self) -> None:
        t = self._state.get_telem()
        if not t:
            return

        state_num = t.get("fpga_act", 0)
        self._lbl_state.setText(FPGA_STATES.get(state_num, f"? ({state_num})"))
        s0  = t.get("fpga_sts0",   0)
        inp = t.get("fpga_inputs", 0)
        self._lbl_s0.setText(f"0x{s0:02X} — {decode_bits(s0, S0_BITS)}")
        self._lbl_in.setText(f"0x{inp:02X} — {decode_bits(inp, IN_BITS)}")
        self._lbl_fc.setText(str(t.get("fpga_fc", "---")))
        self._lbl_sc.setText(str(t.get("fpga_sc", "---")))
        self._lbl_act.setText(f"0x{t.get('fpga_act',0):02X}")
        self._lbl_pchg.setText(str(t.get("fpga_pchg_ms", "---")))
        self._lbl_ver.setText(str(t.get("fpga_version", "---")))

        # Supervision events
        events = self._state.get_supervision_events()
        if events:
            lines = []
            for ts, raw in reversed(list(events)[-30:]):
                age = time.monotonic() - ts
                lines.append(f"+{age:6.1f}s ago  {raw.hex()}")
            self._supv_list.setPlainText("\n".join(lines))

        # Update sparklines using history
        n = 300  # last 30 s at 10 Hz
        x = np.linspace(-30.0, 0.0, n)
        for label, reg, bit in self._all_bits:
            hist_key = "fpga_sts0" if reg == "s0" else "fpga_inputs"
            raw_hist = self._state.get_hist_window(hist_key, n)
            y = np.array([(1.0 if (int(v) >> bit & 1) else 0.0)
                          if not math.isnan(v) else float('nan')
                          for v in raw_hist], dtype=np.float64)
            self._spark_plots[f"{reg}_{bit}"].setData(x=x, y=y)


# ---------------------------------------------------------------------------
# Tab 5 — IMU
# ---------------------------------------------------------------------------

class ImuTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _build(self) -> None:
        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(splitter)

        # ---- Left: numeric IMU panel ----
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(10, 8, 6, 8)
        lv.setSpacing(8)
        lv.addWidget(_hdg("IMU Readings"))

        self._lbls: dict[str, QLabel] = {}

        def _add(form, key, name):
            lbl = _val_label()
            self._lbls[key] = lbl
            _make_form_row(form, name, lbl)

        for imu_idx in (0, 1):
            gb = QGroupBox(f"IMU {imu_idx}")
            form = QFormLayout(gb)
            form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
            p = f"imu{imu_idx}"
            _add(form, f"{p}_ax_g",    "Ax (g):")
            _add(form, f"{p}_ay_g",    "Ay (g):")
            _add(form, f"{p}_az_g",    "Az (g):")
            _add(form, f"{p}_mag",     "|a| (g):")
            _add(form, f"{p}_gx_dps",  "Gx (dps):")
            _add(form, f"{p}_gy_dps",  "Gy (dps):")
            _add(form, f"{p}_gz_dps",  "Gz (dps):")
            _add(form, f"{p}_temp_c",  "Temp (°C):")
            lv.addWidget(gb)

        lv.addStretch()
        left.setFixedWidth(240)
        splitter.addWidget(left)

        # ---- Right: IMU plot panel ----
        self._plot_panel = TelemetryPlotPanel(
            IMU_PLOT_GROUPS, self._state, self._cfg,
            cfg_key="imu_signals_checked",
        )
        splitter.addWidget(self._plot_panel)
        splitter.setSizes([240, 700])

    def refresh(self) -> None:
        t = self._state.get_telem()
        if not t:
            return
        for key, lbl in self._lbls.items():
            v = t.get(key, float('nan'))
            fmt = ".4f" if "_g" in key or "_dps" in key else ".1f"
            lbl.setText(_fmt_val(v, fmt))

    def update_plots(self) -> None:
        self._plot_panel.update_plots()


# ---------------------------------------------------------------------------
# Tab 6 — Motors
# ---------------------------------------------------------------------------

class MotorsTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(8)
        root.addWidget(_hdg("Motor Feedback & Commands"))

        splitter = QSplitter(Qt.Orientation.Vertical)
        root.addWidget(splitter, stretch=1)

        # ---- Top: feedback tree ----
        fb_gb = QGroupBox("Motor Feedback")
        fb_vl = QVBoxLayout(fb_gb)
        self._tree = QTreeWidget()
        self._tree.setColumnCount(6)
        self._tree.setHeaderLabels(["Bus/ID", "Pos (rad)", "Vel (rad/s)",
                                    "Torque (Nm)", "Error", ""])
        self._tree.header().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._tree.setAlternatingRowColors(True)
        fb_vl.addWidget(self._tree)
        splitter.addWidget(fb_gb)

        # ---- Bottom: command entry ----
        cmd_gb = QGroupBox("Motor Command")
        cmd_grid = QGridLayout(cmd_gb)
        cmd_grid.setSpacing(8)

        def _lbl(txt): return QLabel(txt)

        cmd_grid.addWidget(_lbl("Bus:"),         0, 0)
        self._bus_combo = QComboBox()
        self._bus_combo.addItems(["L (0)", "R (1)"])
        cmd_grid.addWidget(self._bus_combo,      0, 1)

        cmd_grid.addWidget(_lbl("Motor ID:"),    0, 2)
        self._mid_spin = QSpinBox()
        self._mid_spin.setRange(1, 8)
        cmd_grid.addWidget(self._mid_spin,       0, 3)

        for ri, (label, attr, lo, hi, dec) in enumerate([
            ("Pos (rad)",    "_pos_spin",  -12.57, 12.57, 4),
            ("Vel (rad/s)",  "_vel_spin",  -15.0,  15.0,  3),
            ("Torque (Nm)",  "_trq_spin",  -120.0, 120.0, 2),
            ("Kp",           "_kp_spin",   0.0,    5000.0, 1),
            ("Kd",           "_kd_spin",   0.0,    100.0,  2),
        ]):
            cmd_grid.addWidget(_lbl(f"{label}:"), ri + 1, 0)
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(dec)
            sb.setSingleStep(0.1)
            setattr(self, attr, sb)
            cmd_grid.addWidget(sb, ri + 1, 1, 1, 3)

        btn_row = QHBoxLayout()
        send_btn  = QPushButton("Send Command")
        estop_btn = QPushButton("E-STOP ALL")
        estop_btn.setProperty("role", "danger")
        send_btn.clicked.connect(self._send_cmd)
        estop_btn.clicked.connect(self._estop_all)
        btn_row.addWidget(send_btn)
        btn_row.addWidget(estop_btn)
        btn_row.addStretch()
        cmd_grid.addLayout(btn_row, 6, 0, 1, 4)
        splitter.addWidget(cmd_gb)
        splitter.setSizes([400, 260])

    def _rcu_ip(self) -> str:
        return self._cfg.get("connection", "rcu_ip", default=RCU_IP_DEFAULT)

    def _send_cmd(self) -> None:
        bus = self._bus_combo.currentIndex()
        send_motor_cmd(
            self._rcu_ip(), bus, self._mid_spin.value(),
            self._pos_spin.value(), self._vel_spin.value(),
            self._trq_spin.value(), self._kp_spin.value(),
            self._kd_spin.value(),
        )

    def _estop_all(self) -> None:
        slots = self._state.get_motor_slots()
        active_ids = [(s["bus"], s["motor_id"]) for s in slots] or None
        send_motor_estop(self._rcu_ip(), active_ids)

    def refresh(self) -> None:
        slots = self._state.get_motor_slots()
        self._tree.clear()
        buses: dict[int, QTreeWidgetItem] = {}
        for s in slots:
            bus = s["bus"]
            if bus not in buses:
                bitem = QTreeWidgetItem([f"Bus {'L' if bus == 0 else 'R'} ({bus})"])
                bitem.setForeground(0, pg.mkColor(ACCENT))
                self._tree.addTopLevelItem(bitem)
                buses[bus] = bitem
                bitem.setExpanded(True)
            err = s["error"]
            row = QTreeWidgetItem([
                f"ID {s['motor_id']}",
                f"{s['pos_rad']:.4f}",
                f"{s['vel_rads']:.3f}",
                f"{s['torque_nm']:.2f}",
                f"0x{err:02X}" if err else "OK",
                "",
            ])
            if err:
                row.setForeground(4, pg.mkColor(ERROR))
            buses[bus].addChild(row)


# ---------------------------------------------------------------------------
# Tab 7 — Commands
# ---------------------------------------------------------------------------

class CommandsTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _rcu_ip(self) -> str:
        return self._cfg.get("connection", "rcu_ip", default=RCU_IP_DEFAULT)

    def _log(self, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._status.append(f"[{ts}] {text}")

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(8)
        root.addWidget(_hdg("Commands"))

        note = QLabel(
            "Buttons marked  ⚠ (FW req)  require unreleased RCU firmware (eth_udp.c). "
            "Commands without this note use the currently released firmware."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"color:{WARN}; background:transparent; font-size:10px;")
        root.addWidget(note)
        root.addWidget(_sep())

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        iv = QVBoxLayout(inner)
        iv.setContentsMargins(0, 0, 0, 0)
        iv.setSpacing(10)
        scroll.setWidget(inner)
        root.addWidget(scroll, stretch=1)

        def _btn(label: str, callback, fw_req: bool = False,
                 role: str = "") -> QPushButton:
            text = f"⚠ {label} (FW req)" if fw_req else label
            b = QPushButton(text)
            if fw_req:
                b.setProperty("role", "warn")
            elif role:
                b.setProperty("role", role)
            b.clicked.connect(callback)
            b.style().unpolish(b)
            b.style().polish(b)
            return b

        # ---- Diagnostics ----
        diag_gb = QGroupBox("Diagnostics")
        diag_h  = QHBoxLayout(diag_gb)
        diag_h.setSpacing(8)
        diag_h.addWidget(_btn("Ping RCU",      self._cmd_ping))
        diag_h.addWidget(_btn("Force Telem",   self._cmd_force_telem))
        diag_h.addWidget(_btn("CAN Loopback",  self._cmd_can_lb))
        diag_h.addWidget(_btn("Reset Gap Counter (GUI)", self._cmd_reset_gaps))
        diag_h.addWidget(_btn("Request Status Dump", self._cmd_status_dump,
                              fw_req=True))
        diag_h.addStretch()
        iv.addWidget(diag_gb)

        # ---- Indicators ----
        ind_gb = QGroupBox("Indicators")
        ind_h  = QHBoxLayout(ind_gb)
        ind_h.setSpacing(8)
        ind_h.addWidget(_btn("Buzz",     self._cmd_buzz))
        ind_h.addWidget(_btn("LED Blink", self._cmd_led))
        ind_h.addStretch()
        iv.addWidget(ind_gb)

        # ---- FPGA / Power ----
        fpga_gb = QGroupBox("FPGA / Power")
        fpga_h  = QHBoxLayout(fpga_gb)
        fpga_h.setSpacing(8)
        fpga_h.addWidget(_btn("Clear PDU Fault Latch", self._cmd_clear_fault,
                              fw_req=True))
        fpga_h.addWidget(_btn("Assert PDU Fault (test)", self._cmd_assert_fault,
                              fw_req=True))
        fpga_h.addStretch()
        iv.addWidget(fpga_gb)

        # ---- Motor Bus ----
        bus_gb = QGroupBox("Motor Bus Control")
        bus_h  = QHBoxLayout(bus_gb)
        bus_h.setSpacing(8)
        bus_h.addWidget(_btn("Bus L → Enable",   self._cmd_busL_en,    fw_req=True))
        bus_h.addWidget(_btn("Bus L → Standby",  self._cmd_busL_stby,  fw_req=True))
        bus_h.addWidget(_btn("Bus R → Enable",   self._cmd_busR_en,    fw_req=True))
        bus_h.addWidget(_btn("Bus R → Standby",  self._cmd_busR_stby,  fw_req=True))
        estop_b = _btn("E-STOP ALL Motors", self._cmd_estop, role="danger")
        bus_h.addWidget(estop_b)
        bus_h.addStretch()
        iv.addWidget(bus_gb)

        # ---- Telem rate ----
        rate_gb = QGroupBox("Set Telemetry Rate")
        rate_h  = QHBoxLayout(rate_gb)
        rate_h.setSpacing(8)
        for hz in (5, 10, 20):
            btn = _btn(f"{hz} Hz",
                       lambda _chk=False, h=hz: self._cmd_set_rate(h),
                       fw_req=True)
            rate_h.addWidget(btn)
        rate_h.addStretch()
        iv.addWidget(rate_gb)

        # ---- System ----
        sys_gb = QGroupBox("System")
        sys_h  = QHBoxLayout(sys_gb)
        sys_h.setSpacing(8)
        sys_h.addWidget(_btn("Soft Reset RCU",
                             self._cmd_reset_rcu, fw_req=True))
        sys_h.addWidget(_btn("Soft Reset RCU + PDU",
                             self._cmd_reset_rcu_pdu, fw_req=True))
        sys_h.addStretch()
        iv.addWidget(sys_gb)

        iv.addStretch()

        # ---- Status log ----
        root.addWidget(_sep())
        root.addWidget(QLabel("Command log:", styleSheet=f"color:{DIM}; background:transparent;"))
        self._status = QTextEdit()
        self._status.setReadOnly(True)
        self._status.setMaximumHeight(140)
        root.addWidget(self._status)

    # ------------------------------------------------------------------
    # Command implementations
    # ------------------------------------------------------------------

    def _cmd_ping(self)        -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_PING)
        self._log("Ping sent")

    def _cmd_force_telem(self) -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_FORCE_TELEM)
        self._log("Force telem sent")

    def _cmd_can_lb(self)      -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_CAN_LOOPBACK)
        self._log("CAN loopback test sent")

    def _cmd_reset_gaps(self)  -> None:
        self._state.reset_gap_counter()
        self._log("Sequence gap counter reset (GUI only)")

    def _cmd_status_dump(self) -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_REQUEST_SUPV_DUMP)
        self._log("Status dump request sent")

    def _cmd_buzz(self)        -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_BUZZ)
        self._log("Buzz sent")

    def _cmd_led(self)         -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_LED_BLINK)
        self._log("LED blink sent")

    def _cmd_clear_fault(self) -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_CLEAR_PDU_FAULT)
        self._log("Clear PDU fault latch sent")

    def _cmd_assert_fault(self)-> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_ASSERT_PDU_FAULT)
        self._log("Assert PDU fault (test) sent")

    def _cmd_busL_en(self)     -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_MOTOR_BUS_CTRL, bytes([0b00]))
        self._log("Bus L → Enable")

    def _cmd_busL_stby(self)   -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_MOTOR_BUS_CTRL, bytes([0b01]))
        self._log("Bus L → Standby")

    def _cmd_busR_en(self)     -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_MOTOR_BUS_CTRL, bytes([0b00]))
        self._log("Bus R → Enable")

    def _cmd_busR_stby(self)   -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_MOTOR_BUS_CTRL, bytes([0b10]))
        self._log("Bus R → Standby")

    def _cmd_estop(self)       -> None:
        slots = self._state.get_motor_slots()
        active = [(s["bus"], s["motor_id"]) for s in slots] or None
        send_motor_estop(self._rcu_ip(), active)
        self._log("E-STOP ALL motors sent")

    def _cmd_set_rate(self, hz: int) -> None:
        send_debug_cmd(self._rcu_ip(), DBGCMD_SET_TELEM_RATE, bytes([hz]))
        self._log(f"Set telem rate → {hz} Hz")

    def _cmd_reset_rcu(self) -> None:
        r = QMessageBox.question(
            self, "Confirm Reset",
            "Send Soft Reset to RCU?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if r == QMessageBox.StandardButton.Yes:
            send_debug_cmd(self._rcu_ip(), DBGCMD_SOFT_RESET, bytes([0x00]))
            self._log("Soft reset RCU sent")

    def _cmd_reset_rcu_pdu(self) -> None:
        r = QMessageBox.question(
            self, "Confirm Reset",
            "Send Soft Reset to RCU and PDU?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if r == QMessageBox.StandardButton.Yes:
            send_debug_cmd(self._rcu_ip(), DBGCMD_SOFT_RESET, bytes([0x01]))
            self._log("Soft reset RCU + PDU sent")

    def refresh(self) -> None:
        pass  # status log is event-driven


# ---------------------------------------------------------------------------
# Tab 8 — Recording
# ---------------------------------------------------------------------------

class RecordingTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config,
                 recorder: TelemetryRecorder, parent=None) -> None:
        super().__init__(parent)
        self._state    = state
        self._cfg      = cfg
        self._recorder = recorder
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)
        root.addWidget(_hdg("CSV Recording"))

        # File path row
        path_row = QHBoxLayout()
        path_row.addWidget(QLabel("File:"))
        self._path_edit = QLineEdit()
        self._path_edit.setPlaceholderText("Click Browse… to choose output file")
        path_row.addWidget(self._path_edit, stretch=1)
        browse_btn = QPushButton("Browse…")
        browse_btn.clicked.connect(self._browse)
        path_row.addWidget(browse_btn)
        root.addLayout(path_row)

        # Signal selection
        root.addWidget(_hdg("Signals to record", color=DIM))
        sig_scroll = QScrollArea()
        sig_scroll.setWidgetResizable(True)
        sig_scroll.setFrameShape(QFrame.Shape.NoFrame)
        sig_scroll.setMaximumHeight(240)
        sig_inner = QWidget()
        sig_grid  = QGridLayout(sig_inner)
        sig_grid.setSpacing(3)
        self._sig_chks: dict[str, QCheckBox] = {}
        saved_sigs = set(self._cfg.get("recording", "signals") or [])
        for i, (lbl, key, dp, di) in enumerate(TELEM_SIGNALS):
            chk = QCheckBox(lbl)
            chk.setChecked(key in saved_sigs if saved_sigs else (dp or di))
            self._sig_chks[key] = chk
            sig_grid.addWidget(chk, i // 3, i % 3)
        sig_scroll.setWidget(sig_inner)
        root.addWidget(sig_scroll)

        sel_row = QHBoxLayout()
        sel_all  = QPushButton("Select All")
        sel_none = QPushButton("Select None")
        sel_all.clicked.connect(lambda: [c.setChecked(True)  for c in self._sig_chks.values()])
        sel_none.clicked.connect(lambda: [c.setChecked(False) for c in self._sig_chks.values()])
        sel_row.addWidget(sel_all)
        sel_row.addWidget(sel_none)
        sel_row.addStretch()
        root.addLayout(sel_row)

        root.addWidget(_sep())

        # Start / Stop
        ctrl_row = QHBoxLayout()
        self._start_btn = QPushButton("▶  Start Recording")
        self._start_btn.setProperty("role", "ok")
        self._stop_btn  = QPushButton("■  Stop Recording")
        self._stop_btn.setProperty("role", "danger")
        self._stop_btn.setEnabled(False)
        self._start_btn.clicked.connect(self._start)
        self._stop_btn.clicked.connect(self._stop)
        ctrl_row.addWidget(self._start_btn)
        ctrl_row.addWidget(self._stop_btn)
        ctrl_row.addStretch()
        root.addLayout(ctrl_row)

        # Status indicators
        status_form = QFormLayout()
        status_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lbl_status  = _val_label("Idle")
        self._lbl_rows    = _val_label("0")
        self._lbl_elapsed = _val_label("0 s")
        self._lbl_size    = _val_label("0 B")
        _make_form_row(status_form, "Status:",  self._lbl_status)
        _make_form_row(status_form, "Rows:",    self._lbl_rows)
        _make_form_row(status_form, "Elapsed:", self._lbl_elapsed)
        _make_form_row(status_form, "File size:", self._lbl_size)
        root.addLayout(status_form)
        root.addStretch()

    def _browse(self) -> None:
        default_dir = self._cfg.get("recording", "default_dir", default="")
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Choose Recording File",
            str(pathlib.Path(default_dir) / "telem_rec.csv") if default_dir else "",
            "CSV Files (*.csv);;All Files (*)",
        )
        if filepath:
            self._path_edit.setText(filepath)
            self._cfg.set("recording", "default_dir",
                          str(pathlib.Path(filepath).parent))

    def _start(self) -> None:
        filepath = self._path_edit.text().strip()
        if not filepath:
            QMessageBox.warning(self, "Recording", "Please choose an output file first.")
            return
        keys = [k for k, chk in self._sig_chks.items() if chk.isChecked()]
        if not keys:
            QMessageBox.warning(self, "Recording", "No signals selected.")
            return
        self._cfg.set("recording", "signals", keys)
        ok = self._recorder.start(filepath, keys)
        if not ok:
            QMessageBox.critical(self, "Recording", "Failed to open file for writing.")
            return
        self._state.attach_recorder(self._recorder)
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._lbl_status.setText("Recording…")
        self._lbl_status.setStyleSheet(
            f"color:{OK}; background:transparent;"
            " font-family:'Consolas','Courier New',monospace; font-size:11px;"
        )

    def _stop(self) -> None:
        self._recorder.stop()
        self._state.attach_recorder(None)
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._lbl_status.setText("Stopped")
        self._lbl_status.setStyleSheet(
            f"color:{TEXT}; background:transparent;"
            " font-family:'Consolas','Courier New',monospace; font-size:11px;"
        )

    def refresh(self) -> None:
        if self._recorder.is_active:
            rows    = self._recorder.row_count
            elapsed = self._recorder.elapsed_s
            size_b  = self._recorder.file_size_bytes
            self._lbl_rows.setText(str(rows))
            self._lbl_elapsed.setText(f"{elapsed:.1f} s")
            if size_b < 1024:
                self._lbl_size.setText(f"{size_b} B")
            elif size_b < 1024 * 1024:
                self._lbl_size.setText(f"{size_b/1024:.1f} KB")
            else:
                self._lbl_size.setText(f"{size_b/1048576:.2f} MB")


# ---------------------------------------------------------------------------
# Tab 9 — Settings
# ---------------------------------------------------------------------------

class SettingsTab(QWidget):
    def __init__(self, state: RcuState, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self._state = state
        self._cfg   = cfg
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(10)
        root.addWidget(_hdg("Settings"))

        # Connection
        conn_gb = QGroupBox("Connection")
        conn_form = QFormLayout(conn_gb)
        conn_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._rcu_ip_edit = QLineEdit(
            self._cfg.get("connection", "rcu_ip", default=RCU_IP_DEFAULT)
        )
        self._reconnect_chk = QCheckBox("Auto-reconnect on socket error")
        self._reconnect_chk.setChecked(
            self._cfg.get("connection", "auto_reconnect", default=True)
        )
        _make_form_row(conn_form, "RCU IP address:", self._rcu_ip_edit)
        conn_form.addRow("", self._reconnect_chk)
        root.addWidget(conn_gb)

        # Display
        disp_gb = QGroupBox("Display")
        disp_form = QFormLayout(disp_gb)
        disp_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._update_spin = QSpinBox()
        self._update_spin.setRange(50, 2000)
        self._update_spin.setSuffix(" ms")
        self._update_spin.setValue(
            self._cfg.get("display", "update_rate_ms", default=100)
        )
        self._tw_spin = QSpinBox()
        self._tw_spin.setRange(5, 600)
        self._tw_spin.setSuffix(" s")
        self._tw_spin.setValue(
            self._cfg.get("display", "default_time_window_s", default=30)
        )
        _make_form_row(disp_form, "Update interval:", self._update_spin)
        _make_form_row(disp_form, "Default time window:", self._tw_spin)
        root.addWidget(disp_gb)

        # Plots
        plot_gb = QGroupBox("Plots")
        plot_form = QFormLayout(plot_gb)
        plot_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._lw_spin = QDoubleSpinBox()
        self._lw_spin.setRange(0.5, 5.0)
        self._lw_spin.setSingleStep(0.25)
        self._lw_spin.setDecimals(2)
        self._lw_spin.setValue(
            self._cfg.get("plots", "line_width", default=1.5)
        )
        self._aa_chk = QCheckBox("Antialiasing")
        self._aa_chk.setChecked(
            self._cfg.get("plots", "antialiasing", default=True)
        )
        self._ogl_chk = QCheckBox("Use OpenGL (requires restart)")
        self._ogl_chk.setChecked(
            self._cfg.get("plots", "use_opengl", default=True)
        )
        _make_form_row(plot_form, "Line width:", self._lw_spin)
        plot_form.addRow("", self._aa_chk)
        plot_form.addRow("", self._ogl_chk)
        root.addWidget(plot_gb)

        # Alerts
        alert_gb = QGroupBox("Alert Thresholds")
        alert_form = QFormLayout(alert_gb)
        alert_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        a = self._cfg.get("alerts") or {}
        self._alert_spins: dict[str, QDoubleSpinBox] = {}
        for key, label, lo, hi in [
            ("v_src_warn_min_v",  "V_SRC min (V):",   0.0,  70.0),
            ("v_src_warn_max_v",  "V_SRC max (V):",   0.0,  70.0),
            ("v_12v_warn_min_v",  "12V min (V):",      0.0,  20.0),
            ("v_12v_warn_max_v",  "12V max (V):",      0.0,  20.0),
            ("v_24v_warn_min_v",  "24V min (V):",      0.0,  30.0),
            ("v_24v_warn_max_v",  "24V max (V):",      0.0,  30.0),
            ("temp_warn_max_c",   "Temp warn (°C):",   0.0,  120.0),
            ("temp_err_max_c",    "Temp error (°C):",  0.0,  120.0),
        ]:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(1)
            sb.setSingleStep(0.5)
            sb.setValue(a.get(key, _DEFAULTS["alerts"][key]))
            self._alert_spins[key] = sb
            _make_form_row(alert_form, label, sb)
        root.addWidget(alert_gb)

        # Save button
        save_btn = QPushButton("Save Settings")
        save_btn.setProperty("role", "ok")
        save_btn.clicked.connect(self._save)
        root.addWidget(save_btn)
        root.addStretch()

    def _save(self) -> None:
        self._cfg.set("connection", "rcu_ip",          self._rcu_ip_edit.text().strip())
        self._cfg.set("connection", "auto_reconnect",  self._reconnect_chk.isChecked())
        self._cfg.set("display",    "update_rate_ms",  self._update_spin.value())
        self._cfg.set("display",    "default_time_window_s", self._tw_spin.value())
        self._cfg.set("plots",      "line_width",       self._lw_spin.value())
        self._cfg.set("plots",      "antialiasing",     self._aa_chk.isChecked())
        self._cfg.set("plots",      "use_opengl",       self._ogl_chk.isChecked())
        for key, sb in self._alert_spins.items():
            self._cfg.set("alerts", key, sb.value())
        self._cfg.save()
        QMessageBox.information(self, "Settings", "Settings saved.")

    def refresh(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Tab 10 — Log
# ---------------------------------------------------------------------------

class LogTab(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(6)

        hdr_row = QHBoxLayout()
        hdr_row.addWidget(_hdg("Application Log"))
        clr_btn = QPushButton("Clear")
        clr_btn.setFixedWidth(70)
        clr_btn.clicked.connect(self._clear)
        hdr_row.addStretch()
        hdr_row.addWidget(clr_btn)
        root.addLayout(hdr_row)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFontFamily("Consolas")
        root.addWidget(self._log, stretch=1)

    def append(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._log.append(f"[{ts}] {msg}")

    def _clear(self) -> None:
        self._log.clear()

    def refresh(self) -> None:
        pass


# ---------------------------------------------------------------------------
# MainWindow
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self, state: RcuState, cfg: Config, receiver: RcuReceiver,
                 recorder: TelemetryRecorder) -> None:
        super().__init__()
        self._state    = state
        self._cfg      = cfg
        self._receiver = receiver
        self._recorder = recorder

        self.setWindowTitle("Plymouth Humanoid Bench Monitor")
        self.setMinimumSize(1280, 800)

        # Restore window geometry
        geo_str = cfg.get("ui", "window_geometry", default="")
        if geo_str:
            try:
                from PyQt6.QtCore import QByteArray
                self.restoreGeometry(QByteArray.fromBase64(geo_str.encode()))
            except Exception:
                pass

        # Build tabs
        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)
        self.setCentralWidget(self._tabs)

        self._overview_tab   = OverviewTab(state, cfg)
        self._xref_tab       = PowerCrossRefTab(state, cfg)
        self._pdu_tab        = PduTab(state, cfg)
        self._fpga_tab       = FpgaHealthTab(state, cfg)
        self._imu_tab        = ImuTab(state, cfg)
        self._motors_tab     = MotorsTab(state, cfg)
        self._commands_tab   = CommandsTab(state, cfg)
        self._recording_tab  = RecordingTab(state, cfg, recorder)
        self._settings_tab   = SettingsTab(state, cfg)
        self._log_tab        = LogTab()

        for label, widget in (
            ("Overview",        self._overview_tab),
            ("Power / X-Ref",   self._xref_tab),
            ("PDU",             self._pdu_tab),
            ("FPGA / Health",   self._fpga_tab),
            ("IMU",             self._imu_tab),
            ("Motors",          self._motors_tab),
            ("Commands",        self._commands_tab),
            ("Recording",       self._recording_tab),
            ("Settings",        self._settings_tab),
            ("Log",             self._log_tab),
        ):
            self._tabs.addTab(widget, label)

        # Restore last tab
        last_tab = cfg.get("ui", "last_tab_index", default=0)
        self._tabs.setCurrentIndex(max(0, min(last_tab, self._tabs.count() - 1)))

        # 100 ms update timer
        self._timer = QTimer(self)
        self._timer.setInterval(
            cfg.get("display", "update_rate_ms", default=100)
        )
        self._timer.timeout.connect(self._update_loop)
        self._timer.start()

    # ------------------------------------------------------------------
    # Update loop — fires every 100 ms
    # ------------------------------------------------------------------

    def _update_loop(self) -> None:
        idx = self._tabs.currentIndex()

        # Always update visible tab
        tab = self._tabs.currentWidget()
        if hasattr(tab, "refresh"):
            try:
                tab.refresh()
            except Exception as exc:
                self._log_tab.append(f"refresh error ({type(tab).__name__}): {exc}")

        # Plot panels — only update the currently visible tab
        if isinstance(tab, PduTab):
            try:
                tab.update_plots()
            except Exception as exc:
                self._log_tab.append(f"plot error: {exc}")
        elif isinstance(tab, ImuTab):
            try:
                tab.update_plots()
            except Exception as exc:
                self._log_tab.append(f"plot error: {exc}")

    # ------------------------------------------------------------------
    # Close event — save config
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        self._timer.stop()
        self._receiver.stop()
        self._recorder.stop()

        # Save last tab index
        self._cfg.set("ui", "last_tab_index", self._tabs.currentIndex())

        # Save window geometry
        try:
            geo_bytes = self.saveGeometry().toBase64().data().decode()
            self._cfg.set("ui", "window_geometry", geo_bytes)
        except Exception:
            pass

        self._cfg.save()
        event.accept()


# ===========================================================================
# Phase 5 — Entry point
# ===========================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plymouth Humanoid Bench Monitor"
    )
    parser.add_argument(
        "--rcu-ip",
        default=None,
        help=f"RCU IP address (default: from config or {RCU_IP_DEFAULT})",
    )
    args = parser.parse_args()

    # Config — must come first so we can read use_opengl before any Qt objects
    cfg = Config()

    if args.rcu_ip:
        cfg.set("connection", "rcu_ip", args.rcu_ip)

    # pyqtgraph global settings — must be called before QApplication
    use_opengl = cfg.get("plots", "use_opengl", default=True)
    antialias  = cfg.get("plots", "antialiasing", default=True)
    _configure_pyqtgraph(use_opengl=use_opengl, antialias=antialias)

    app = QApplication([])
    app.setStyleSheet(DARK_STYLESHEET)

    # Shared state
    state    = RcuState()
    state.rcu_ip = cfg.get("connection", "rcu_ip", default=RCU_IP_DEFAULT)
    recorder = TelemetryRecorder()

    # Receiver thread
    receiver = RcuReceiver(state)
    receiver.start()

    # Main window
    window = MainWindow(state, cfg, receiver, recorder)
    window.show()

    import sys
    sys.exit(app.exec())


if __name__ == "__main__":
    main()



