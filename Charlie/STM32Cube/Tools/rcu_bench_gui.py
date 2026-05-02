#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plymouth Humanoid Bench Monitor
Requires: pip install customtkinter matplotlib

Usage:
  python rcu_bench_gui.py [--rcu-ip 192.168.100.10]

Tabs: Overview | PDU | IMU | Motors | Power | Log
"""

import argparse
import collections
import math
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import customtkinter as ctk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ---------------------------------------------------------------------------
# One Dark palette
# ---------------------------------------------------------------------------
BG      = "#1e2229"
PANEL   = "#282c34"
ACCENT  = "#61afef"
OK      = "#98c379"
WARN    = "#e5c07b"
ERROR   = "#e06c75"
TEXT    = "#abb2bf"
DIM     = "#5c6370"
WHITE   = "#ffffff"
PURPLE  = "#c678dd"

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ---------------------------------------------------------------------------
# Protocol (must match rcu_pkt.h)
# ---------------------------------------------------------------------------
PKT_MAGIC       = 0x5243
HDR_FMT         = "<HBBH"
HDR_SIZE        = struct.calcsize(HDR_FMT)

PKT_TELEM       = 0x01
PKT_MOTOR_FB    = 0x02
PKT_DEBUG_CMD   = 0x20
PKT_DEBUG_REPLY = 0x21
PKT_MOTOR_CMD   = 0x10

TELEM_FMT  = "<6BH8h4h6h7h7h"
TELEM_SIZE = struct.calcsize(TELEM_FMT)

FB_HEADER_SIZE = 1
FB_SLOT_FMT    = "<BBHHHBx"
FB_SLOT_SIZE   = struct.calcsize(FB_SLOT_FMT)
FB_SLOTS       = 16

DBGCMD_PING         = 0x01
DBGCMD_BUZZ         = 0x02
DBGCMD_LED_BLINK    = 0x03
DBGCMD_CAN_LOOPBACK = 0x04
DBGCMD_FORCE_TELEM  = 0x05

PORT_TELEM  = 7700
PORT_CMD    = 7701

RCU_IP_DEFAULT = "192.168.100.10"

# IMU scale factors (LSM6DSOX, +-4g / +-500dps)
IMU_ACCEL_G_PER_LSB  = 0.000122
IMU_GYRO_DPS_PER_LSB = 0.0175
IMU_TEMP_SCALE       = 1.0 / 256.0
IMU_TEMP_OFFSET_C    = 25.0

RS04_POS_MAX = 12.57
RS04_VEL_MAX = 15.0
RS04_TRQ_MAX = 120.0
RS04_KP_MAX  = 5000.0
RS04_KD_MAX  = 100.0

HISTORY_LEN  = 300   # ~30 s at 10 Hz

FPGA_STATES  = {0: "IDLE", 1: "PRECHARGE", 2: "ARMED", 3: "COMPUTE"}
# STATUS0 register 0x00 — from pdu_glue_mxo2.sv
S0_BITS      = [(7,"FAULT_LATCH"),(6,"PCHG_LATCH"),(5,"MOTOR_EN"),
                (4,"COMPUTE_EN"),(3,"K_SEL"),(2,"K_EN"),(1,"OVUV_OK"),(0,"ARM_PERMIT")]
# INPUTS register 0x04 — from pdu_glue_mxo2.sv
IN_BITS      = [(7,"IS_ARMED"),(6,"ARM_LATCH"),(5,"ESTOP_OK"),
                (4,"MCU_ARM"),(3,"MCU_ALIVE"),(2,"FB_CLOSED"),(1,"PCHG_OK"),(0,"VBUS_OV")]

# Exponential moving average alpha (0 = no filtering, 1 = freeze)
EMA_ALPHA = 0.25


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def u16_to_f(v, lo, hi):
    return lo + (v / 65535.0) * (hi - lo)


def f_to_u16(v, lo, hi):
    v = max(lo, min(hi, v))
    return int((v - lo) / (hi - lo) * 65535 + 0.5)


def decode_bits(val, bit_defs):
    active = [name for (bit, name) in bit_defs if (val >> bit) & 1]
    return " | ".join(active) if active else "none"


def ema(prev, new_val, alpha=EMA_ALPHA):
    """Exponential moving average. Returns new_val on first call (NaN prev)."""
    if math.isnan(new_val):
        return prev
    if math.isnan(prev):
        return new_val
    return prev * (1.0 - alpha) + new_val * alpha


def _color_label(val, lo_warn, hi_warn, lo_err=None, hi_err=None):
    if lo_err is not None and val <= lo_err:
        return ERROR
    if hi_err is not None and val >= hi_err:
        return ERROR
    if val < lo_warn or val > hi_warn:
        return WARN
    return OK


# ---------------------------------------------------------------------------
# Thread-safe state
# ---------------------------------------------------------------------------
class RcuState:
    # All numeric telem keys that should have history deques.
    # (label, key, default_checked_pdu, default_checked_imu)
    TELEM_SIGNALS = [
        # Power / voltage / current
        ("V_RAW",      "v_vraw_v",       True,  False),
        ("12V",        "v_12v_v",        True,  False),
        ("24V",        "v_24v_v",        True,  False),
        ("I_RAW",      "i_vraw_sw_ma",   False, False),
        ("I_12V",      "i_12v_ma",       False, False),
        ("I_24V",      "i_24v_ma",       False, False),
        ("Therm1",     "therm1_c",       False, False),
        ("Therm2",     "therm2_c",       False, False),
        # Energy meter
        ("EM Volt",    "em_v_v",         False, False),
        ("EM Curr",    "em_i_ma",        False, False),
        ("EM Power",   "em_p_w",         False, False),
        ("EM Temp",    "em_t_c",         False, False),
        # Local ADC
        ("L Therm0",   "ladc_therm0_c",  False, False),
        ("L Therm1",   "ladc_therm1_c",  False, False),
        ("L Therm2",   "ladc_therm2_c",  False, False),
        ("V_SOURCE",   "ladc_vsource_v", False, False),
        ("V_BUS",      "ladc_vbus_v",    False, False),
        ("I_COIL",     "ladc_icoil_ma",  False, False),
        # IMU 0
        ("IMU0 Ax",    "imu0_ax_g",      False, True),
        ("IMU0 Ay",    "imu0_ay_g",      False, True),
        ("IMU0 Az",    "imu0_az_g",      False, True),
        ("IMU0 |a|",   "imu0_mag",       False, True),
        ("IMU0 Gx",    "imu0_gx_dps",    False, False),
        ("IMU0 Gy",    "imu0_gy_dps",    False, False),
        ("IMU0 Gz",    "imu0_gz_dps",    False, False),
        ("IMU0 Temp",  "imu0_temp_c",    False, False),
        # IMU 1
        ("IMU1 Ax",    "imu1_ax_g",      False, True),
        ("IMU1 Ay",    "imu1_ay_g",      False, True),
        ("IMU1 Az",    "imu1_az_g",      False, True),
        ("IMU1 |a|",   "imu1_mag",       False, True),
        ("IMU1 Gx",    "imu1_gx_dps",    False, False),
        ("IMU1 Gy",    "imu1_gy_dps",    False, False),
        ("IMU1 Gz",    "imu1_gz_dps",    False, False),
        ("IMU1 Temp",  "imu1_temp_c",    False, False),
    ]

    def __init__(self):
        self._lock = threading.Lock()

        self.connected     = False
        self.rcu_ip        = RCU_IP_DEFAULT
        self.pkt_count     = 0
        self.telem_count   = 0
        self.seq_gaps      = 0
        self._last_seq     = -1
        self._pkt_ts_deque = collections.deque(maxlen=50)

        self._last_telem_t = 0.0
        self.t             = {}
        self._ema          = {}

        self.motor_slots   = []
        self.last_ping     = {}

        # Generic per-signal history deques
        self._hist = {
            key: collections.deque([float('nan')] * HISTORY_LEN, maxlen=HISTORY_LEN)
            for (_, key, _, _) in self.TELEM_SIGNALS
        }
        # Aliases kept for legacy get_history() calls
        self.h_vraw     = self._hist["v_vraw_v"]
        self.h_v12      = self._hist["v_12v_v"]
        self.h_v24      = self._hist["v_24v_v"]
        self.h_imu0_mag = self._hist["imu0_mag"]
        self.h_imu1_mag = self._hist["imu1_mag"]

    def update_telem(self, t):
        with self._lock:
            self._last_telem_t = time.monotonic()
            self.telem_count  += 1
            self.t = dict(t)

            for key in ("v_vraw_v","v_12v_v","v_24v_v","em_v_v","em_i_ma",
                        "i_vraw_sw_ma","i_12v_ma","i_24v_ma","em_p_w"):
                raw = t.get(key, float('nan'))
                self._ema[key] = ema(self._ema.get(key, raw), raw)

            # Derived signals
            ax0, ay0, az0 = t.get("imu0_ax_g",0), t.get("imu0_ay_g",0), t.get("imu0_az_g",0)
            ax1, ay1, az1 = t.get("imu1_ax_g",0), t.get("imu1_ay_g",0), t.get("imu1_az_g",0)
            imu0_valid = (ax0 != 0 or ay0 != 0 or az0 != 0)
            imu1_valid = (ax1 != 0 or ay1 != 0 or az1 != 0)
            t["imu0_mag"] = math.sqrt(ax0**2 + ay0**2 + az0**2) if imu0_valid else float('nan')
            t["imu1_mag"] = math.sqrt(ax1**2 + ay1**2 + az1**2) if imu1_valid else float('nan')
            # If IMU invalid, push nan to all its axes too
            if not imu0_valid:
                for k in ("imu0_ax_g","imu0_ay_g","imu0_az_g",
                          "imu0_gx_dps","imu0_gy_dps","imu0_gz_dps"):
                    t[k] = float('nan')
            if not imu1_valid:
                for k in ("imu1_ax_g","imu1_ay_g","imu1_az_g",
                          "imu1_gx_dps","imu1_gy_dps","imu1_gz_dps"):
                    t[k] = float('nan')

            # Update all generic history deques
            for (_, key, _, _) in self.TELEM_SIGNALS:
                val = t.get(key, float('nan'))
                if key in self._hist:
                    self._hist[key].append(val)

    def get_ema(self, key, default=0.0):
        with self._lock:
            v = self._ema.get(key, default)
            return default if math.isnan(v) else v

    def bump_pkt(self, seq):
        with self._lock:
            self.pkt_count += 1
            self._pkt_ts_deque.append(time.monotonic())
            if self._last_seq >= 0:
                gap = (seq - self._last_seq - 1) & 0xFF
                self.seq_gaps += gap
            self._last_seq = seq

    def pkts_per_sec(self):
        with self._lock:
            now = time.monotonic()
            return len([x for x in self._pkt_ts_deque if now - x <= 1.0])

    def telem_age(self):
        with self._lock:
            if self._last_telem_t == 0.0:
                return 9999
            return int((time.monotonic() - self._last_telem_t) * 1000)

    def get_telem(self):
        with self._lock:
            return dict(self.t)

    def get_motor_slots(self):
        with self._lock:
            return list(self.motor_slots)

    def update_motor_fb(self, slots):
        with self._lock:
            self.motor_slots = slots

    def update_ping(self, reply):
        with self._lock:
            self.last_ping = dict(reply)

    def set_connected(self, v):
        with self._lock:
            self.connected = v

    def is_connected(self):
        with self._lock:
            return self.connected

    def get_history(self, name):
        with self._lock:
            d = getattr(self, name, None)
            return list(d) if d is not None else []

    def get_hist(self, key):
        """Get history deque for any telem signal key."""
        with self._lock:
            d = self._hist.get(key)
            return list(d) if d is not None else [float('nan')] * HISTORY_LEN


# ---------------------------------------------------------------------------
# UDP receiver thread
# ---------------------------------------------------------------------------
class RcuReceiver(threading.Thread):
    def __init__(self, state: RcuState):
        super().__init__(daemon=True)
        self.rcu   = state
        self._stop = threading.Event()
        self._sock = None

    def stop(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def run(self):
        while not self._stop.is_set():
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._sock.bind(("0.0.0.0", PORT_TELEM))
                self._sock.settimeout(1.0)
                self.rcu.set_connected(True)
                while not self._stop.is_set():
                    try:
                        data, _ = self._sock.recvfrom(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    self._process(data)
            except Exception:
                pass
            finally:
                self.rcu.set_connected(False)
                try:
                    self._sock.close()
                except Exception:
                    pass
            if not self._stop.is_set():
                time.sleep(1.0)

    def _process(self, data):
        if len(data) < HDR_SIZE:
            return
        magic, pkt_type, seq, plen = struct.unpack_from(HDR_FMT, data, 0)
        if magic != PKT_MAGIC:
            return
        payload = data[HDR_SIZE: HDR_SIZE + plen]
        self.rcu.bump_pkt(seq)
        if pkt_type == PKT_TELEM:
            t = _decode_telem(payload)
            if t:
                self.rcu.update_telem(t)
        elif pkt_type == PKT_MOTOR_FB:
            self.rcu.update_motor_fb(_decode_motor_fb(payload))
        elif pkt_type == PKT_DEBUG_REPLY:
            r = _decode_debug_reply(payload)
            if r:
                self.rcu.update_ping(r)


# ---------------------------------------------------------------------------
# Packet decoders
# ---------------------------------------------------------------------------
def _decode_telem(payload):
    if len(payload) < TELEM_SIZE:
        return None
    f = struct.unpack_from(TELEM_FMT, payload)
    (s0, fc, sc, act, inp, ver, pchg_ms,
     v_vraw_dv, v_12v, v_24v, i_vraw_sw, i_12v, i_24v, th1, th2,
     em_i, em_v_dv, em_p, em_t,
     lt0, lt1, lt2, lvsrc, lvbus, licoil,
     a0x, a0y, a0z, g0x, g0y, g0z, t0,
     a1x, a1y, a1z, g1x, g1y, g1z, t1) = f

    return dict(
        fpga_sts0=s0, fpga_fc=fc, fpga_sc=sc, fpga_act=act,
        fpga_inputs=inp, fpga_version=ver, fpga_pchg_ms=pchg_ms,
        # External ADC rails
        v_vraw_v=v_vraw_dv / 100.0,
        v_12v_v=v_12v / 1000.0,    v_24v_v=v_24v / 1000.0,
        i_vraw_sw_ma=i_vraw_sw,
        i_12v_ma=i_12v,             i_24v_ma=i_24v,
        therm1_c=th1 * 0.1,         therm2_c=th2 * 0.1,
        # Energy meter -- voltage in 10mV units (divide by 100 for V)
        em_i_ma=em_i,
        em_v_v=em_v_dv / 100.0,
        em_p_w=em_p * 0.1,
        em_t_c=em_t * 0.1,
        # Local STM32 ADC — vsource/vbus packed in 10mV units (x100), so divide by 100
        ladc_therm0_c=lt0 * 0.1,    ladc_therm1_c=lt1 * 0.1,
        ladc_therm2_c=lt2 * 0.1,
        ladc_vsource_v=lvsrc / 100.0,
        ladc_vbus_v=lvbus / 100.0,
        ladc_icoil_ma=licoil,
        # IMU0
        imu0_ax_g=a0x * IMU_ACCEL_G_PER_LSB,
        imu0_ay_g=a0y * IMU_ACCEL_G_PER_LSB,
        imu0_az_g=a0z * IMU_ACCEL_G_PER_LSB,
        imu0_gx_dps=g0x * IMU_GYRO_DPS_PER_LSB,
        imu0_gy_dps=g0y * IMU_GYRO_DPS_PER_LSB,
        imu0_gz_dps=g0z * IMU_GYRO_DPS_PER_LSB,
        imu0_temp_c=t0 * IMU_TEMP_SCALE + IMU_TEMP_OFFSET_C,
        # IMU1
        imu1_ax_g=a1x * IMU_ACCEL_G_PER_LSB,
        imu1_ay_g=a1y * IMU_ACCEL_G_PER_LSB,
        imu1_az_g=a1z * IMU_ACCEL_G_PER_LSB,
        imu1_gx_dps=g1x * IMU_GYRO_DPS_PER_LSB,
        imu1_gy_dps=g1y * IMU_GYRO_DPS_PER_LSB,
        imu1_gz_dps=g1z * IMU_GYRO_DPS_PER_LSB,
        imu1_temp_c=t1 * IMU_TEMP_SCALE + IMU_TEMP_OFFSET_C,
    )


def _decode_motor_fb(payload):
    if len(payload) < FB_HEADER_SIZE:
        return []
    (count,) = struct.unpack_from("<B", payload, 0)
    slots = []
    off   = FB_HEADER_SIZE
    for _ in range(min(count, FB_SLOTS)):
        if off + FB_SLOT_SIZE > len(payload):
            break
        bus, mid, pos_u16, vel_u16, trq_u16, err = struct.unpack_from(FB_SLOT_FMT, payload, off)
        off += FB_SLOT_SIZE
        slots.append(dict(
            bus=bus, motor_id=mid,
            pos_rad=u16_to_f(pos_u16, -RS04_POS_MAX, RS04_POS_MAX),
            vel_rads=u16_to_f(vel_u16, -RS04_VEL_MAX, RS04_VEL_MAX),
            torque_nm=u16_to_f(trq_u16, -RS04_TRQ_MAX, RS04_TRQ_MAX),
            error=err,
        ))
    return slots


DBG_REPLY_FMT  = "<II6Bxx I"
DBG_REPLY_SIZE = struct.calcsize(DBG_REPLY_FMT)


def _decode_debug_reply(payload):
    if len(payload) < DBG_REPLY_SIZE:
        return None
    (uptime, boot_rsr, imu0_v, imu1_v, fpga_v, rails_v, ssd_v, can_lb, hb_age) = \
        struct.unpack_from(DBG_REPLY_FMT, payload)
    return dict(
        uptime_ms=uptime, boot_rsr=boot_rsr,
        imu0_ok=bool(imu0_v), imu1_ok=bool(imu1_v),
        fpga_ok=bool(fpga_v), rails_ok=bool(rails_v), em_ok=bool(ssd_v),
        can_lb=can_lb, hb_age_ms=hb_age,
    )


# ---------------------------------------------------------------------------
# Command sender
# ---------------------------------------------------------------------------
_seq_lock = threading.Lock()
_seq_val  = 0


def _next_seq():
    global _seq_val
    with _seq_lock:
        v = _seq_val
        _seq_val = (_seq_val + 1) & 0xFF
        return v


def send_debug_cmd(rcu_ip, subcmd):
    seq  = _next_seq()
    hdr  = struct.pack(HDR_FMT, PKT_MAGIC, PKT_DEBUG_CMD, seq, 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(hdr + bytes([subcmd]), (rcu_ip, PORT_CMD))
    finally:
        sock.close()


def send_motor_cmd(rcu_ip, bus, motor_id, pos, vel, trq, kp, kd):
    pos_u16 = f_to_u16(pos, -RS04_POS_MAX, RS04_POS_MAX)
    vel_u16 = f_to_u16(vel, -RS04_VEL_MAX, RS04_VEL_MAX)
    trq_u16 = f_to_u16(trq, -RS04_TRQ_MAX, RS04_TRQ_MAX)
    kp_u8   = int(max(0, min(255, (kp / RS04_KP_MAX) * 255)))
    kd_u8   = int(max(0, min(255, (kd / RS04_KD_MAX) * 255)))
    entry   = struct.pack("<BBHHHBBxx", bus, motor_id, pos_u16, vel_u16, trq_u16, kp_u8, kd_u8)
    seq     = _next_seq()
    hdr     = struct.pack(HDR_FMT, PKT_MAGIC, PKT_MOTOR_CMD, seq, len(entry))
    sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(hdr + entry, (rcu_ip, PORT_CMD))
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Matplotlib helpers
# ---------------------------------------------------------------------------
def _make_fig(nrows=1, ncols=1, figsize=(7, 2.5)):
    fig = Figure(figsize=figsize, facecolor=BG)
    axs = []
    for i in range(nrows * ncols):
        ax = fig.add_subplot(nrows, ncols, i + 1)
        ax.set_facecolor(PANEL)
        ax.tick_params(colors=TEXT, labelsize=8)
        ax.xaxis.label.set_color(TEXT)
        ax.yaxis.label.set_color(TEXT)
        for spine in ax.spines.values():
            spine.set_edgecolor(DIM)
        ax.grid(True, color="#3d3f43", linewidth=0.5)
        axs.append(ax)
    fig.subplots_adjust(left=0.14, right=0.97, top=0.93, bottom=0.12)
    return fig, axs


def _embed_fig(fig, parent):
    canvas = FigureCanvasTkAgg(fig, master=parent)
    canvas.draw()
    canvas.get_tk_widget().pack(fill="both", expand=True)
    return canvas


# ---------------------------------------------------------------------------
# GUI label helpers
# ---------------------------------------------------------------------------
def _lv_row(parent, label, row, col=0, colspan=1, width=130,
            label_font=("Consolas", 11), val_font=("Consolas", 11)):
    lbl = ctk.CTkLabel(parent, text=label, anchor="w",
                       text_color=DIM, font=label_font)
    lbl.grid(row=row, column=col, sticky="w", padx=(8, 2), pady=1)
    val = ctk.CTkLabel(parent, text="\u2014\u2014\u2014", anchor="w",
                       text_color=TEXT, font=val_font, width=width)
    val.grid(row=row, column=col + 1, columnspan=colspan,
             sticky="w", padx=(0, 8), pady=1)
    return val


def _status_dot(parent, row, col=0, size=14):
    dot = ctk.CTkLabel(parent, text="\u25cf", font=("Consolas", size),
                       text_color=DIM)
    dot.grid(row=row, column=col, padx=(8, 2), pady=1, sticky="w")
    return dot


def _section_header(parent, text, row=0, cols=2):
    ctk.CTkLabel(parent, text=text, text_color=ACCENT,
                 font=("Consolas", 12, "bold")
                 ).grid(row=row, column=0, columnspan=cols,
                        padx=8, pady=(8, 4), sticky="w")


# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------
class App(ctk.CTk):
    def __init__(self, rcu_ip: str):
        super().__init__()
        self.title("Plymouth Humanoid Bench Monitor")
        self.geometry("1380x820")
        self.configure(fg_color=BG)

        self.rcu_ip = rcu_ip
        self.rcu    = RcuState()
        self.rcu.rcu_ip = rcu_ip
        self._recv  = None
        self._log_lines = collections.deque(maxlen=500)
        self._pdu_plot_vars = {}

        self._build_topbar()
        self._build_tabs()
        self._start_receiver()
        self.after(100, self._update_loop)

    # ------------------------------------------------------------------
    # Top bar
    # ------------------------------------------------------------------
    def _build_topbar(self):
        bar = ctk.CTkFrame(self, fg_color=PANEL, height=48, corner_radius=0)
        bar.pack(fill="x", side="top")
        bar.pack_propagate(False)

        ctk.CTkLabel(bar, text="Plymouth Humanoid Bench Monitor",
                     text_color=ACCENT, font=("Consolas", 13, "bold")
                     ).pack(side="left", padx=(14, 20))

        ctk.CTkLabel(bar, text="RCU IP:", text_color=DIM,
                     font=("Consolas", 11)).pack(side="left", padx=(0, 2))
        self._ip_var = ctk.StringVar(value=self.rcu_ip)
        ctk.CTkEntry(bar, textvariable=self._ip_var, width=140,
                     font=("Consolas", 12)).pack(side="left", padx=(0, 8))
        ctk.CTkButton(bar, text="Reconnect", width=90,
                      command=self._reconnect,
                      fg_color=ACCENT, hover_color="#4d9bd4",
                      font=("Consolas", 11)
                      ).pack(side="left", padx=(0, 14))

        self._conn_lbl = ctk.CTkLabel(bar, text="\u25cf  Connecting\u2026",
                                      text_color=WARN, font=("Consolas", 12))
        self._conn_lbl.pack(side="left", padx=(0, 20))
        self._pps_lbl = ctk.CTkLabel(bar, text="0 pkt/s",
                                     text_color=DIM, font=("Consolas", 11))
        self._pps_lbl.pack(side="left", padx=(0, 10))
        self._gap_lbl = ctk.CTkLabel(bar, text="0 gaps",
                                     text_color=DIM, font=("Consolas", 11))
        self._gap_lbl.pack(side="left")
        self._age_lbl = ctk.CTkLabel(bar, text="telem: ---ms",
                                     text_color=DIM, font=("Consolas", 11))
        self._age_lbl.pack(side="right", padx=12)

    # ------------------------------------------------------------------
    # Tabs
    # ------------------------------------------------------------------
    def _build_tabs(self):
        self._nb = ctk.CTkTabview(self, fg_color=BG,
                                  segmented_button_fg_color=PANEL,
                                  segmented_button_selected_color=ACCENT,
                                  segmented_button_unselected_color=PANEL,
                                  text_color=TEXT)
        self._nb.pack(fill="both", expand=True, padx=6, pady=4)

        for name in ("Overview", "PDU", "IMU", "Motors", "Log"):
            self._nb.add(name)

        self._build_overview(self._nb.tab("Overview"))
        self._build_pdu(self._nb.tab("PDU"))
        self._build_imu(self._nb.tab("IMU"))
        self._build_motors(self._nb.tab("Motors"))
        self._build_log(self._nb.tab("Log"))

    # ------------------------------------------------------------------
    # Overview tab
    # ------------------------------------------------------------------
    def _build_overview(self, tab):
        tab.configure(fg_color=BG)

        health = ctk.CTkFrame(tab, fg_color=PANEL, corner_radius=8)
        health.pack(side="left", fill="y", padx=(6, 4), pady=6)
        _section_header(health, "SUBSYSTEM HEALTH", row=0, cols=3)

        self._ov_dots = {}
        self._ov_vals = {}
        items = [
            ("IMU 0",        "imu0",   "telem"),
            ("IMU 1",        "imu1",   "telem"),
            ("PDU FPGA",     "fpga",   "telem"),
            ("V_RAW",        "vraw",   "telem"),
            ("12 V",         "v12",    "telem"),
            ("24 V",         "v24",    "telem"),
            ("Energy Mtr",   "em",     "telem"),
            ("PDU HB",       "pdu_hb", "ping"),
            ("CAN Loopback", "can_lb", "ping"),
        ]
        for i, (label, key, src) in enumerate(items, start=1):
            dot = _status_dot(health, row=i)
            ctk.CTkLabel(health, text=label, anchor="w",
                         text_color=TEXT, font=("Consolas", 11), width=96
                         ).grid(row=i, column=1, sticky="w", padx=(0, 4), pady=1)
            ctk.CTkLabel(health, text=f"[{src}]", anchor="w",
                         text_color=DIM, font=("Consolas", 9), width=44
                         ).grid(row=i, column=2, sticky="w", padx=(0, 2))
            val = ctk.CTkLabel(health, text="\u2014\u2014\u2014", anchor="w",
                               text_color=DIM, font=("Consolas", 10), width=110)
            val.grid(row=i, column=3, sticky="w", padx=(0, 8), pady=1)
            self._ov_dots[key] = dot
            self._ov_vals[key] = val

        stats = ctk.CTkFrame(health, fg_color=BG, corner_radius=6)
        stats.grid(row=len(items) + 1, column=0, columnspan=4,
                   padx=8, pady=8, sticky="ew")
        self._ov_uptime = _lv_row(stats, "Uptime",  0, width=90)
        self._ov_pkts   = _lv_row(stats, "Packets", 1, width=90)
        self._ov_vraw   = _lv_row(stats, "V_RAW",   2, width=90)

        # Quick commands
        cmds = ctk.CTkFrame(tab, fg_color=PANEL, corner_radius=8)
        cmds.pack(side="left", fill="none", padx=4, pady=6)
        _section_header(cmds, "QUICK COMMANDS", row=0, cols=1)

        btn_defs = [
            ("Ping (both)",      lambda: self._send_dbg(DBGCMD_PING)),
            ("Buzz (both)",      lambda: self._send_dbg(DBGCMD_BUZZ)),
            ("LED Blink (both)", lambda: self._send_dbg(DBGCMD_LED_BLINK)),
            ("CAN Loopback",     lambda: self._send_dbg(DBGCMD_CAN_LOOPBACK)),
            ("Force Telem",      lambda: self._send_dbg(DBGCMD_FORCE_TELEM)),
        ]
        for i, (label, cmd) in enumerate(btn_defs, start=1):
            ctk.CTkButton(cmds, text=label, command=cmd, width=160,
                          fg_color=ACCENT, hover_color="#4d9bd4",
                          font=("Consolas", 11)
                          ).grid(row=i, column=0, pady=4, padx=14, sticky="ew")

        pf = ctk.CTkFrame(cmds, fg_color=BG, corner_radius=6)
        pf.grid(row=len(btn_defs) + 1, column=0, padx=8, pady=8, sticky="ew")
        ctk.CTkLabel(pf, text="Last ping:", text_color=DIM,
                     font=("Consolas", 10)).pack(anchor="w", padx=6)
        self._ping_lbl = ctk.CTkLabel(pf, text="(none)", text_color=TEXT,
                                      font=("Consolas", 10), justify="left",
                                      anchor="w", wraplength=200)
        self._ping_lbl.pack(anchor="w", padx=6, pady=(0, 6))

        # FPGA state machine visual
        fpga_vis = ctk.CTkFrame(tab, fg_color=PANEL, corner_radius=8)
        fpga_vis.pack(side="left", fill="none", padx=4, pady=6)
        _section_header(fpga_vis, "FPGA STATE MACHINE", row=0, cols=1)

        self._state_btns = {}
        state_order = [
            ("IDLE",      DIM),
            ("PRECHARGE", WARN),
            ("ARMED",     ACCENT),
            ("COMPUTE",   OK),
        ]
        for i, (sname, scolor) in enumerate(state_order):
            btn = ctk.CTkLabel(fpga_vis, text=f"  {sname}  ",
                               font=("Consolas", 12, "bold"),
                               fg_color=BG, corner_radius=6,
                               text_color=DIM, width=120)
            btn.grid(row=i + 1, column=0, padx=14, pady=4, sticky="ew")
            self._state_btns[sname] = (btn, scolor)

        ctk.CTkLabel(fpga_vis, text="STATUS0", text_color=DIM,
                     font=("Consolas", 10)
                     ).grid(row=5, column=0, padx=8, pady=(8, 0), sticky="w")
        self._ov_sts0 = ctk.CTkLabel(fpga_vis, text="\u2014\u2014\u2014",
                                     text_color=TEXT, font=("Consolas", 10),
                                     justify="left", anchor="w", wraplength=180)
        self._ov_sts0.grid(row=6, column=0, padx=8, pady=(0, 4), sticky="w")

        ctk.CTkLabel(fpga_vis, text="INPUTS", text_color=DIM,
                     font=("Consolas", 10)
                     ).grid(row=7, column=0, padx=8, pady=(4, 0), sticky="w")
        self._ov_inp = ctk.CTkLabel(fpga_vis, text="\u2014\u2014\u2014",
                                    text_color=TEXT, font=("Consolas", 10),
                                    justify="left", anchor="w", wraplength=180)
        self._ov_inp.grid(row=8, column=0, padx=8, pady=(0, 8), sticky="w")

    # ------------------------------------------------------------------
    # PDU tab
    # ------------------------------------------------------------------
    def _build_pdu(self, tab):
        tab.configure(fg_color=BG)

        left = ctk.CTkScrollableFrame(tab, fg_color=BG, width=420)
        left.pack(side="left", fill="y", padx=(6, 2), pady=6)

        # FPGA status
        fpga_f = ctk.CTkFrame(left, fg_color=PANEL, corner_radius=8)
        fpga_f.pack(fill="x", pady=(0, 6))
        _section_header(fpga_f, "FPGA STATUS", row=0)
        self._pdu = {}
        for i, (lbl, key) in enumerate([
            ("State",      "fpga_state"),
            ("STATUS0",    "fpga_sts0"),
            ("INPUTS",     "fpga_inp"),
            ("Fault code", "fpga_fault"),
            ("Version",    "fpga_ver"),
            ("Pchg timer", "fpga_pchg"),
        ], start=1):
            self._pdu[key] = _lv_row(fpga_f, lbl, i, width=210)

        # Power rails -- V + I side by side
        rail_f = ctk.CTkFrame(left, fg_color=PANEL, corner_radius=8)
        rail_f.pack(fill="x", pady=(0, 6))
        _section_header(rail_f, "POWER RAILS (Ext ADC)", row=0, cols=4)
        for ci, txt in enumerate(("Rail", "Voltage", "Current"), start=0):
            ctk.CTkLabel(rail_f, text=txt, text_color=DIM,
                         font=("Consolas", 10, "bold"),
                         width=(80 if ci == 0 else 110), anchor="w"
                         ).grid(row=1, column=ci, padx=(8 if ci == 0 else 2, 2),
                                pady=(0, 2))

        self._rail_v = {}
        self._rail_i = {}
        for r, (name, vkey, ikey_lbl) in enumerate([
            ("V_RAW",  "v_vraw_v",  "i_vraw_sw_ma_lbl"),
            ("12V",    "v_12v_v",   "i_12v_ma_lbl"),
            ("24V",    "v_24v_v",   "i_24v_ma_lbl"),
        ], start=2):
            ctk.CTkLabel(rail_f, text=name, text_color=ACCENT,
                         font=("Consolas", 11, "bold"), width=80, anchor="w"
                         ).grid(row=r, column=0, padx=(8, 2), pady=2, sticky="w")
            vl = ctk.CTkLabel(rail_f, text="\u2014\u2014\u2014", anchor="w",
                              text_color=TEXT, font=("Consolas", 11), width=110)
            vl.grid(row=r, column=1, padx=2, pady=2, sticky="w")
            il = ctk.CTkLabel(rail_f, text="\u2014\u2014\u2014", anchor="w",
                              text_color=TEXT, font=("Consolas", 11), width=110)
            il.grid(row=r, column=2, padx=(2, 8), pady=2, sticky="w")
            self._rail_v[vkey]   = vl
            self._rail_i[ikey_lbl] = il

        for r, (lbl, key) in enumerate([("Therm1", "th1"), ("Therm2", "th2")], start=5):
            ctk.CTkLabel(rail_f, text=lbl, text_color=DIM,
                         font=("Consolas", 11), anchor="w", width=80
                         ).grid(row=r, column=0, padx=(8, 2), pady=2, sticky="w")
            self._pdu[key] = ctk.CTkLabel(rail_f, text="\u2014\u2014\u2014",
                                          anchor="w", text_color=TEXT,
                                          font=("Consolas", 11), width=110)
            self._pdu[key].grid(row=r, column=1, padx=2,
                                pady=(2, 8 if r == 6 else 2), sticky="w")

        # Energy meter
        em_f = ctk.CTkFrame(left, fg_color=PANEL, corner_radius=8)
        em_f.pack(fill="x", pady=(0, 6))
        _section_header(em_f, "ENERGY METER (RS485)", row=0, cols=4)
        for ci, txt in enumerate(("", "Voltage", "Current"), start=0):
            ctk.CTkLabel(em_f, text=txt, text_color=DIM,
                         font=("Consolas", 10, "bold"),
                         width=(80 if ci == 0 else 110), anchor="w"
                         ).grid(row=1, column=ci, padx=(8 if ci == 0 else 2, 2),
                                pady=(0, 2))
        ctk.CTkLabel(em_f, text="Source", text_color=ACCENT,
                     font=("Consolas", 11, "bold"), width=80, anchor="w"
                     ).grid(row=2, column=0, padx=(8, 2), pady=2, sticky="w")
        self._pdu["em_v"] = ctk.CTkLabel(em_f, text="\u2014\u2014\u2014",
                                         anchor="w", text_color=TEXT,
                                         font=("Consolas", 11), width=110)
        self._pdu["em_v"].grid(row=2, column=1, padx=2, pady=2, sticky="w")
        self._pdu["em_i"] = ctk.CTkLabel(em_f, text="\u2014\u2014\u2014",
                                         anchor="w", text_color=TEXT,
                                         font=("Consolas", 11), width=110)
        self._pdu["em_i"].grid(row=2, column=2, padx=(2, 8), pady=2, sticky="w")

        for r, (lbl, key) in enumerate([("Power", "em_p"), ("Temp", "em_t")], start=3):
            ctk.CTkLabel(em_f, text=lbl, text_color=DIM,
                         font=("Consolas", 11), anchor="w", width=80
                         ).grid(row=r, column=0, padx=(8, 2), pady=2, sticky="w")
            self._pdu[key] = ctk.CTkLabel(em_f, text="\u2014\u2014\u2014",
                                          anchor="w", text_color=TEXT,
                                          font=("Consolas", 11), width=110)
            self._pdu[key].grid(row=r, column=1, padx=2,
                                pady=(2, 8 if r == 4 else 2), sticky="w")

        # Board ADC (local STM32)
        ladc_f = ctk.CTkFrame(left, fg_color=PANEL, corner_radius=8)
        ladc_f.pack(fill="x", pady=(0, 6))
        _section_header(ladc_f, "BOARD ADC (STM32 LOCAL)", row=0, cols=4)
        for ci, txt in enumerate(("Signal", "Value"), start=0):
            ctk.CTkLabel(ladc_f, text=txt, text_color=DIM,
                         font=("Consolas", 10, "bold"),
                         width=(80 if ci == 0 else 130), anchor="w"
                         ).grid(row=1, column=ci, padx=(8 if ci == 0 else 2, 2),
                                pady=(0, 2))
        ladc_rows = [
            ("V_SOURCE", "lvsrc"),
            ("V_BUS",    "lvbus"),
            ("I_COIL",   "licoil"),
            ("Therm0",   "lt0"),
            ("Therm1",   "lt1"),
            ("Therm2",   "lt2"),
        ]
        for r, (name, key) in enumerate(ladc_rows, start=2):
            ctk.CTkLabel(ladc_f, text=name, text_color=ACCENT,
                         font=("Consolas", 11, "bold"), width=80, anchor="w"
                         ).grid(row=r, column=0, padx=(8, 2), pady=2, sticky="w")
            vl = ctk.CTkLabel(ladc_f, text="\u2014\u2014\u2014", anchor="w",
                              text_color=TEXT, font=("Consolas", 11), width=130)
            vl.grid(row=r, column=1, padx=(2, 8),
                    pady=(2, 8 if r == len(ladc_rows) + 1 else 2), sticky="w")
            self._pdu[key] = vl

        # Right panel: signal checkboxes + plot
        right = ctk.CTkFrame(tab, fg_color=BG)
        right.pack(side="left", fill="both", expand=True, padx=(2, 6), pady=6)

        # Color cycle for dynamically added lines
        self._pdu_color_cycle = [
            WARN, OK, ACCENT, ERROR, PURPLE, WHITE,
            "#ff9e64", "#56b6c2", "#d19a66", "#be5046",
        ]

        # Checkbox panel (scrollable, left side of right frame)
        chk_outer = ctk.CTkFrame(right, fg_color=PANEL, corner_radius=6, width=130)
        chk_outer.pack(side="left", fill="y", padx=(0, 4), pady=0)
        chk_outer.pack_propagate(False)
        ctk.CTkLabel(chk_outer, text="SIGNALS", text_color=ACCENT,
                     font=("Consolas", 10, "bold")
                     ).pack(anchor="w", padx=8, pady=(6, 2))
        chk_scroll = ctk.CTkScrollableFrame(chk_outer, fg_color=PANEL, width=118)
        chk_scroll.pack(fill="both", expand=True, padx=2, pady=2)

        # PDU-only signals (not IMU)
        pdu_sigs = [(lbl, key, dflt_pdu)
                    for (lbl, key, dflt_pdu, _) in RcuState.TELEM_SIGNALS
                    if not key.startswith("imu")]

        self._pdu_chk_vars = {}          # key → BooleanVar
        self._pdu_lines    = {}          # key → Line2D
        self._pdu_plot_vars = {}         # kept for compat

        for lbl, key, default in pdu_sigs:
            var = ctk.BooleanVar(value=default)
            self._pdu_chk_vars[key] = var
            color_idx = sum(1 for v in self._pdu_chk_vars.values() if v.get()) - 1
            cb = ctk.CTkCheckBox(chk_scroll, text=lbl, variable=var,
                                 font=("Consolas", 10),
                                 command=self._rebuild_pdu_lines)
            cb.pack(anchor="w", padx=4, pady=1)

        # Plot area
        plot_right = ctk.CTkFrame(right, fg_color=BG)
        plot_right.pack(side="left", fill="both", expand=True)

        ctk.CTkLabel(plot_right, text="Telemetry history (30 s)",
                     text_color=DIM, font=("Consolas", 10)
                     ).pack(anchor="w", padx=6)
        self._pdu_fig, pdu_axs = _make_fig(figsize=(5.5, 4.5))
        self._pdu_ax = pdu_axs[0]
        self._pdu_ax.set_ylabel("value", color=TEXT, fontsize=9)
        self._pdu_canvas = _embed_fig(self._pdu_fig, plot_right)
        # Build initial lines for defaults
        self._rebuild_pdu_lines()

    def _rebuild_pdu_lines(self):
        """Add/remove matplotlib lines based on checkbox state."""
        ax = self._pdu_ax
        xs = list(range(HISTORY_LEN))
        colors = self._pdu_color_cycle
        color_idx = 0
        for lbl, key, *_ in [s for s in RcuState.TELEM_SIGNALS if not s[1].startswith("imu")]:
            var = self._pdu_chk_vars.get(key)
            color = colors[color_idx % len(colors)]
            color_idx += 1
            if var and var.get():
                if key not in self._pdu_lines:
                    line, = ax.plot(xs, [float('nan')] * HISTORY_LEN,
                                    color=color, lw=1.2, label=lbl)
                    self._pdu_lines[key] = line
                else:
                    self._pdu_lines[key].set_visible(True)
            else:
                if key in self._pdu_lines:
                    self._pdu_lines[key].set_visible(False)
        # Rebuild legend from visible lines
        visible = [(l.get_label(), l) for l in self._pdu_lines.values()
                   if l.get_visible()]
        if visible:
            labels, handles = zip(*visible)
            ax.legend(handles, labels, loc="upper left", fontsize=7,
                      facecolor=PANEL, edgecolor=DIM, labelcolor=TEXT,
                      ncol=2)
        else:
            ax.get_legend() and ax.get_legend().remove()
        self._pdu_canvas.draw_idle()

    # ------------------------------------------------------------------
    # IMU tab
    # ------------------------------------------------------------------
    def _build_imu(self, tab):
        tab.configure(fg_color=BG)
        self._imu = {}

        # Top half: two data panels side-by-side
        top = ctk.CTkFrame(tab, fg_color=BG)
        top.pack(side="top", fill="x", padx=6, pady=(6, 2))

        for col_idx, (imu_name, prefix) in enumerate([
            ("IMU 0  (SPI4, CS=PC13)", "imu0"),
            ("IMU 1  (SPI3, CS=PA15)", "imu1"),
        ]):
            frame = ctk.CTkFrame(top, fg_color=PANEL, corner_radius=8)
            frame.pack(side="left", fill="both", expand=True,
                       padx=(0 if col_idx == 0 else 4, 0), pady=0)
            _section_header(frame, imu_name, row=0)
            fields = [
                ("Ax (g)",    f"{prefix}_ax"),
                ("Ay (g)",    f"{prefix}_ay"),
                ("Az (g)",    f"{prefix}_az"),
                ("|a| (g)",   f"{prefix}_mag"),
                ("Gx (dps)",  f"{prefix}_gx"),
                ("Gy (dps)",  f"{prefix}_gy"),
                ("Gz (dps)",  f"{prefix}_gz"),
                ("IMU Temp",  f"{prefix}_temp"),
            ]
            if prefix == "imu0":
                fields.append(("Board Therm", "imu0_btherm"))
            for i, (lbl, key) in enumerate(fields, start=1):
                self._imu[key] = _lv_row(frame, lbl, i, width=130)

        # Bottom half: combined dynamic graph
        bot = ctk.CTkFrame(tab, fg_color=BG)
        bot.pack(side="top", fill="both", expand=True, padx=6, pady=(2, 6))

        # IMU-only signals
        imu_sigs = [(lbl, key, dflt_imu)
                    for (lbl, key, _, dflt_imu) in RcuState.TELEM_SIGNALS
                    if key.startswith("imu")]

        self._imu_chk_vars = {}
        self._imu_lines    = {}

        # Checkbox strip (left)
        chk_outer = ctk.CTkFrame(bot, fg_color=PANEL, corner_radius=6, width=130)
        chk_outer.pack(side="left", fill="y", padx=(0, 4), pady=0)
        chk_outer.pack_propagate(False)
        ctk.CTkLabel(chk_outer, text="SIGNALS", text_color=ACCENT,
                     font=("Consolas", 10, "bold")
                     ).pack(anchor="w", padx=8, pady=(6, 2))
        chk_scroll = ctk.CTkScrollableFrame(chk_outer, fg_color=PANEL, width=118)
        chk_scroll.pack(fill="both", expand=True, padx=2, pady=2)

        for lbl, key, default in imu_sigs:
            var = ctk.BooleanVar(value=default)
            self._imu_chk_vars[key] = var
            cb = ctk.CTkCheckBox(chk_scroll, text=lbl, variable=var,
                                 font=("Consolas", 10),
                                 command=self._rebuild_imu_lines)
            cb.pack(anchor="w", padx=4, pady=1)

        # Graph (right of checkbox strip)
        plot_outer = ctk.CTkFrame(bot, fg_color=BG)
        plot_outer.pack(side="left", fill="both", expand=True)
        self._imu_fig, imu_axs = _make_fig(figsize=(7, 3.0))
        self._imu_ax = imu_axs[0]
        self._imu_ax.set_ylabel("value", color=TEXT, fontsize=9)
        self._imu_canvas = _embed_fig(self._imu_fig, plot_outer)
        self._imu_color_cycle = [
            ACCENT, WARN, OK, ERROR, PURPLE, WHITE,
            "#ff9e64", "#56b6c2", "#d19a66", "#be5046",
        ]
        self._rebuild_imu_lines()

    def _rebuild_imu_lines(self):
        ax = self._imu_ax
        xs = list(range(HISTORY_LEN))
        colors = self._imu_color_cycle
        color_idx = 0
        for lbl, key, *_ in [s for s in RcuState.TELEM_SIGNALS if s[1].startswith("imu")]:
            var = self._imu_chk_vars.get(key)
            color = colors[color_idx % len(colors)]
            color_idx += 1
            if var and var.get():
                if key not in self._imu_lines:
                    line, = ax.plot(xs, [float('nan')] * HISTORY_LEN,
                                    color=color, lw=1.2, label=lbl)
                    self._imu_lines[key] = line
                else:
                    self._imu_lines[key].set_visible(True)
            else:
                if key in self._imu_lines:
                    self._imu_lines[key].set_visible(False)
        visible = [(l.get_label(), l) for l in self._imu_lines.values()
                   if l.get_visible()]
        if visible:
            labels, handles = zip(*visible)
            ax.legend(handles, labels, loc="upper left", fontsize=7,
                      facecolor=PANEL, edgecolor=DIM, labelcolor=TEXT, ncol=2)
        else:
            leg = ax.get_legend()
            if leg:
                leg.remove()
        self._imu_canvas.draw_idle()

    # ------------------------------------------------------------------
    # Motors tab
    # ------------------------------------------------------------------
    def _build_motors(self, tab):
        tab.configure(fg_color=BG)
        table_f = ctk.CTkFrame(tab, fg_color=PANEL, corner_radius=8)
        table_f.pack(fill="both", expand=True, padx=6, pady=6, side="top")
        _section_header(table_f, "MOTOR FEEDBACK", row=0)

        cols = ("Bus", "ID", "Pos (rad)", "Vel (rad/s)", "Torque (Nm)", "Error")
        self._motor_tree = ttk.Treeview(table_f, columns=cols,
                                        show="headings", height=12)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background=PANEL, foreground=TEXT,
                        fieldbackground=PANEL, rowheight=22,
                        font=("Consolas", 10))
        style.configure("Treeview.Heading", background=BG, foreground=ACCENT,
                        font=("Consolas", 10, "bold"))
        for col in cols:
            self._motor_tree.heading(col, text=col)
            self._motor_tree.column(col, width=110, anchor="center")
        table_f.grid_rowconfigure(1, weight=1)
        table_f.grid_columnconfigure(0, weight=1)
        self._motor_tree.grid(row=1, column=0, padx=6, pady=4, sticky="nsew")

        cmd_f = ctk.CTkFrame(tab, fg_color=PANEL, corner_radius=8)
        cmd_f.pack(fill="x", padx=6, pady=(0, 6), side="bottom")
        _section_header(cmd_f, "MOTOR COMMAND", row=0, cols=8)
        field_defs = [
            ("Bus (0=R,1=L)", "_mc_bus",  "0"),
            ("Motor ID",      "_mc_id",   "1"),
            ("Pos (rad)",     "_mc_pos",  "0.0"),
            ("Vel (rad/s)",   "_mc_vel",  "0.0"),
            ("Trq (Nm)",      "_mc_trq",  "0.0"),
            ("Kp",            "_mc_kp",   "10.0"),
            ("Kd",            "_mc_kd",   "1.0"),
        ]
        for ci, (lbl, attr, default) in enumerate(field_defs):
            ctk.CTkLabel(cmd_f, text=lbl, text_color=DIM,
                         font=("Consolas", 10)
                         ).grid(row=1, column=ci, padx=4, pady=2)
            var = ctk.StringVar(value=default)
            ctk.CTkEntry(cmd_f, textvariable=var, width=78,
                         font=("Consolas", 11)
                         ).grid(row=2, column=ci, padx=4, pady=2)
            setattr(self, attr, var)
        ctk.CTkButton(cmd_f, text="Send", width=80, command=self._send_motor,
                      fg_color=OK, hover_color="#7aad5c", text_color=BG,
                      font=("Consolas", 11, "bold")
                      ).grid(row=2, column=len(field_defs), padx=8, pady=2)
        self._mc_status = ctk.CTkLabel(cmd_f, text="", text_color=DIM,
                                       font=("Consolas", 10))
        self._mc_status.grid(row=3, column=0, columnspan=8,
                             padx=8, pady=(0, 6), sticky="w")

    # ------------------------------------------------------------------
    # Power tab
    # ------------------------------------------------------------------
    def _build_power(self, tab):
        tab.configure(fg_color=BG)
        self._pw = {}

        ctk.CTkLabel(tab,
                     text="Cross-system view: all available readings per rail. "
                          "Values color-coded against nominal range.",
                     text_color=DIM, font=("Consolas", 10), justify="left"
                     ).pack(anchor="w", padx=10, pady=(6, 2))

        scroll = ctk.CTkScrollableFrame(tab, fg_color=BG)
        scroll.pack(fill="both", expand=True, padx=6, pady=4)

        # (rail_title, [(source_label, telem_key, unit, lo_warn, hi_warn)])
        rail_defs = [
            ("SOURCE VOLTAGE  (~55 V bus)", [
                ("Ext ADC (TLA2528)",    "v_vraw_v",       "V",  40.0, 62.0),
                ("Energy Meter",         "em_v_v",         "V",  40.0, 62.0),
                ("STM32 Local ADC",      "ladc_vsource_v", "V",  40.0, 62.0),
            ]),
            ("12 V RAIL", [
                ("Ext ADC (TLA2528)",    "v_12v_v",        "V",  11.0, 13.0),
                ("STM32 Local ADC",      "ladc_vbus_v",    "V",  11.0, 13.0),
            ]),
            ("24 V RAIL", [
                ("Ext ADC (TLA2528)",    "v_24v_v",        "V",  22.0, 26.0),
            ]),
            ("SOURCE CURRENT", [
                ("Ext ADC (I_RAW_SW)",   "i_vraw_sw_ma",   "mA",  0.0, 8000.0),
                ("Energy Meter",         "em_i_ma",        "mA",  0.0, 8000.0),
            ]),
            ("12V CURRENT", [
                ("Ext ADC",              "i_12v_ma",       "mA",  0.0, 5000.0),
            ]),
            ("24V CURRENT", [
                ("Ext ADC",              "i_24v_ma",       "mA",  0.0, 5000.0),
            ]),
            ("TOTAL POWER", [
                ("Energy Meter",         "em_p_w",         "W",   0.0, 500.0),
            ]),
            ("BOARD TEMPERATURE", [
                ("STM32 Therm0",         "ladc_therm0_c",  "\u00b0C", 5.0, 55.0),
                ("STM32 Therm1",         "ladc_therm1_c",  "\u00b0C", 5.0, 55.0),
                ("STM32 Therm2",         "ladc_therm2_c",  "\u00b0C", 5.0, 55.0),
                ("Ext ADC Therm1",       "therm1_c",       "\u00b0C", 5.0, 55.0),
                ("Ext ADC Therm2",       "therm2_c",       "\u00b0C", 5.0, 55.0),
                ("Energy Meter",         "em_t_c",         "\u00b0C", 5.0, 55.0),
                ("IMU0 on-die",          "imu0_temp_c",    "\u00b0C", 5.0, 85.0),
                ("IMU1 on-die",          "imu1_temp_c",    "\u00b0C", 5.0, 85.0),
            ]),
        ]

        for rail_name, readings in rail_defs:
            rf = ctk.CTkFrame(scroll, fg_color=PANEL, corner_radius=8)
            rf.pack(fill="x", pady=(0, 8))
            ctk.CTkLabel(rf, text=rail_name, text_color=ACCENT,
                         font=("Consolas", 12, "bold")
                         ).grid(row=0, column=0, columnspan=3,
                                padx=8, pady=(8, 4), sticky="w")
            for hci, htxt in enumerate(("Source", "Value")):
                ctk.CTkLabel(rf, text=htxt, text_color=DIM,
                             font=("Consolas", 10, "bold"),
                             width=(230 if hci == 0 else 130), anchor="w"
                             ).grid(row=1, column=hci,
                                    padx=(8 if hci == 0 else 4, 4),
                                    pady=(0, 2))
            for ri, (src, key, unit, lo_w, hi_w) in enumerate(readings, start=2):
                ctk.CTkLabel(rf, text=src, text_color=TEXT,
                             font=("Consolas", 10), width=230, anchor="w"
                             ).grid(row=ri, column=0, padx=(8, 4), pady=2, sticky="w")
                vl = ctk.CTkLabel(rf, text="\u2014\u2014\u2014", anchor="w",
                                  text_color=TEXT,
                                  font=("Consolas", 11, "bold"), width=130)
                vl.grid(row=ri, column=1, padx=(4, 8), pady=2, sticky="w")
                # Store existing key only if not already there (multiple
                # rails can share the same telem key for display)
                if key not in self._pw:
                    self._pw[key] = []
                self._pw[key].append((vl, unit, lo_w, hi_w))

    # ------------------------------------------------------------------
    # Log tab
    # ------------------------------------------------------------------
    def _build_log(self, tab):
        tab.configure(fg_color=BG)
        self._log_text = ctk.CTkTextbox(tab, fg_color=PANEL,
                                        text_color=TEXT, font=("Consolas", 10),
                                        wrap="none", state="disabled")
        self._log_text.pack(fill="both", expand=True, padx=6, pady=6)

    # ------------------------------------------------------------------
    # Receiver lifecycle
    # ------------------------------------------------------------------
    def _start_receiver(self):
        self.rcu_ip = self._ip_var.get().strip() or RCU_IP_DEFAULT
        self.rcu.rcu_ip = self.rcu_ip
        if self._recv and self._recv.is_alive():
            self._recv.stop()
        self._recv = RcuReceiver(self.rcu)
        self._recv.start()

    def _reconnect(self):
        self._start_receiver()

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def _send_dbg(self, subcmd):
        ip = self._ip_var.get().strip() or RCU_IP_DEFAULT
        threading.Thread(target=send_debug_cmd, args=(ip, subcmd),
                         daemon=True).start()

    def _send_motor(self):
        try:
            bus = int(self._mc_bus.get())
            mid = int(self._mc_id.get())
            pos = float(self._mc_pos.get())
            vel = float(self._mc_vel.get())
            trq = float(self._mc_trq.get())
            kp  = float(self._mc_kp.get())
            kd  = float(self._mc_kd.get())
        except ValueError as e:
            self._mc_status.configure(text=f"Parse error: {e}",
                                      text_color=ERROR)
            return
        ip = self._ip_var.get().strip() or RCU_IP_DEFAULT
        threading.Thread(target=send_motor_cmd,
                         args=(ip, bus, mid, pos, vel, trq, kp, kd),
                         daemon=True).start()
        self._mc_status.configure(
            text=f"Sent bus={bus} id={mid} pos={pos:.3f} kp={kp:.1f} kd={kd:.2f}",
            text_color=OK)

    # ------------------------------------------------------------------
    # Main update loop (100 ms)
    # ------------------------------------------------------------------
    def _update_loop(self):
        try:
            self._update_topbar()
            t = self.rcu.get_telem()
            if t:
                self._update_overview(t)
                self._update_pdu(t)
                self._update_imu(t)
            self._update_motors()
        except Exception as e:
            self._log(f"UI update error: {e}")
        self.after(100, self._update_loop)

    def _update_topbar(self):
        connected = self.rcu.is_connected()
        age_ms    = self.rcu.telem_age()
        pps       = self.rcu.pkts_per_sec()
        gaps      = self.rcu.seq_gaps

        if connected:
            if age_ms < 500:
                self._conn_lbl.configure(text="\u25cf  Live", text_color=OK)
            else:
                self._conn_lbl.configure(text="\u25cf  Stale", text_color=WARN)
        else:
            self._conn_lbl.configure(text="\u25cf  Disconnected",
                                     text_color=ERROR)
        self._pps_lbl.configure(text=f"{pps} pkt/s")
        self._gap_lbl.configure(text=f"{gaps} gaps")
        age_str = f"{age_ms}ms" if age_ms < 9999 else "---"
        self._age_lbl.configure(text=f"telem: {age_str}")

    def _update_overview(self, t):
        ping = self.rcu.last_ping

        def _set(key, ok, txt=""):
            color = OK if ok else ERROR
            if key in self._ov_dots:
                self._ov_dots[key].configure(text_color=color)
            if key in self._ov_vals:
                self._ov_vals[key].configure(text=txt, text_color=color)

        vraw = t.get("v_vraw_v", 0.0)
        v12  = t.get("v_12v_v",  0.0)
        v24  = t.get("v_24v_v",  0.0)

        a0_mag = math.sqrt(t.get("imu0_ax_g",0)**2 +
                           t.get("imu0_ay_g",0)**2 +
                           t.get("imu0_az_g",0)**2)
        a1_mag = math.sqrt(t.get("imu1_ax_g",0)**2 +
                           t.get("imu1_ay_g",0)**2 +
                           t.get("imu1_az_g",0)**2)

        _set("imu0",  a0_mag > 0.1,  f"|a|={a0_mag:.3f}g")
        _set("imu1",  a1_mag > 0.1,  f"|a|={a1_mag:.3f}g")

        sc = t.get("fpga_sc", 255)
        fpga_ok = sc in (1, 2, 3)
        _set("fpga",  fpga_ok, FPGA_STATES.get(sc, f"?{sc}"))

        _set("vraw", vraw > 20.0,
             f"{vraw:.2f} V",)
        _set("v12",  10.5 < v12 < 13.5, f"{v12:.2f} V")
        _set("v24",  21.0 < v24 < 27.0, f"{v24:.1f} V")

        em_v = t.get("em_v_v", 0.0)
        _set("em",    em_v > 20.0,   f"V={em_v:.1f}V")

        pdu_hb_ok = ping.get("hb_age_ms", 0xFFFFFFFF) < 2000
        hb_txt = f"{ping['hb_age_ms']}ms" if "hb_age_ms" in ping else "---"
        _set("pdu_hb", pdu_hb_ok, hb_txt)

        can_lb = ping.get("can_lb", 0)
        can_str = {0:"untested",1:"right only",2:"left only",
                   3:"BOTH OK",0xFF:"FAIL"}.get(can_lb, f"0x{can_lb:02X}")
        _set("can_lb", can_lb == 3, can_str)

        uptime = ping.get("uptime_ms", 0)
        self._ov_uptime.configure(text=f"{uptime//1000}s" if uptime else "---")
        self._ov_pkts.configure(text=str(self.rcu.pkt_count))
        self._ov_vraw.configure(
            text=f"{vraw:.2f} V",
            text_color=_color_label(vraw, 40.0, 62.0, lo_err=20.0, hi_err=65.0))

        if ping:
            rsr = ping.get("boot_rsr", 0)
            rst = "clean" if rsr == 0 else f"RSR=0x{rsr:08X}"
            self._ping_lbl.configure(
                text=f"up={ping.get('uptime_ms',0)//1000}s  reset={rst}\n"
                     f"IMU0={'OK' if ping.get('imu0_ok') else 'FAIL'}  "
                     f"IMU1={'OK' if ping.get('imu1_ok') else 'FAIL'}")

        # State machine visual
        state_name = FPGA_STATES.get(sc, "")
        for sname, (btn, scolor) in self._state_btns.items():
            if sname == state_name:
                btn.configure(fg_color=scolor, text_color=BG)
            else:
                btn.configure(fg_color=BG, text_color=DIM)

        self._ov_sts0.configure(
            text=decode_bits(t.get("fpga_sts0", 0), S0_BITS) or "none")
        self._ov_inp.configure(
            text=decode_bits(t.get("fpga_inputs", 0), IN_BITS) or "none")

    def _update_pdu(self, t):
        pdu = self._pdu

        sc = t.get("fpga_sc", 0)
        state_str = FPGA_STATES.get(sc, f"?{sc}")
        pdu["fpga_state"].configure(
            text=state_str,
            text_color=OK if state_str in ("ARMED", "COMPUTE") else WARN)
        pdu["fpga_sts0"].configure(
            text=decode_bits(t.get("fpga_sts0", 0), S0_BITS))
        pdu["fpga_inp"].configure(
            text=decode_bits(t.get("fpga_inputs", 0), IN_BITS))
        fc = t.get("fpga_fc", 0)
        pdu["fpga_fault"].configure(
            text=f"0x{fc:02X}" if fc else "none",
            text_color=ERROR if fc else OK)
        pdu["fpga_ver"].configure(
            text=f"0x{t.get('fpga_version', 0):02X}")
        pdu["fpga_pchg"].configure(
            text=f"{t.get('fpga_pchg_ms', 0)} ms")

        v_raw = self.rcu.get_ema("v_vraw_v")
        v12   = self.rcu.get_ema("v_12v_v")
        v24   = self.rcu.get_ema("v_24v_v")
        i_raw = self.rcu.get_ema("i_vraw_sw_ma")
        i12   = self.rcu.get_ema("i_12v_ma")
        i24   = self.rcu.get_ema("i_24v_ma")

        self._rail_v["v_vraw_v"].configure(
            text=f"{v_raw:.2f} V",
            text_color=_color_label(v_raw, 40.0, 62.0, lo_err=20.0, hi_err=65.0))
        self._rail_v["v_12v_v"].configure(
            text=f"{v12:.3f} V",
            text_color=_color_label(v12, 11.0, 13.0, lo_err=10.0, hi_err=14.0))
        self._rail_v["v_24v_v"].configure(
            text=f"{v24:.3f} V",
            text_color=_color_label(v24, 22.0, 26.0, lo_err=20.0, hi_err=28.0))
        self._rail_i["i_vraw_sw_ma_lbl"].configure(text=f"{i_raw:.0f} mA")
        self._rail_i["i_12v_ma_lbl"].configure(text=f"{i12:.0f} mA")
        self._rail_i["i_24v_ma_lbl"].configure(text=f"{i24:.0f} mA")

        pdu["th1"].configure(text=f"{t.get('therm1_c', 0):.1f} \u00b0C")
        pdu["th2"].configure(text=f"{t.get('therm2_c', 0):.1f} \u00b0C")

        em_v = self.rcu.get_ema("em_v_v")
        em_i = self.rcu.get_ema("em_i_ma")
        em_p = self.rcu.get_ema("em_p_w")
        pdu["em_v"].configure(
            text=f"{em_v:.2f} V",
            text_color=_color_label(em_v, 40.0, 62.0, lo_err=5.0, hi_err=65.0))
        pdu["em_i"].configure(text=f"{em_i:.0f} mA")
        pdu["em_p"].configure(text=f"{em_p:.1f} W")
        pdu["em_t"].configure(text=f"{t.get('em_t_c', 0):.1f} \u00b0C")

        pdu["lt0"].configure(text=f"{t.get('ladc_therm0_c', 0):.1f} \u00b0C")
        pdu["lt1"].configure(text=f"{t.get('ladc_therm1_c', 0):.1f} \u00b0C")
        pdu["lt2"].configure(text=f"{t.get('ladc_therm2_c', 0):.1f} \u00b0C")
        pdu["lvsrc"].configure(
            text=f"{t.get('ladc_vsource_v', 0):.3f} V",
            text_color=_color_label(t.get('ladc_vsource_v', 0),
                                    40.0, 62.0, lo_err=5.0, hi_err=65.0))
        pdu["lvbus"].configure(
            text=f"{t.get('ladc_vbus_v', 0):.3f} V",
            text_color=_color_label(t.get('ladc_vbus_v', 0),
                                    10.5, 13.5, lo_err=5.0, hi_err=15.0))
        pdu["licoil"].configure(text=f"{t.get('ladc_icoil_ma', 0):.0f} mA")

        # Generic plot update — only update visible lines
        xs = list(range(HISTORY_LEN))
        for key, line in self._pdu_lines.items():
            if line.get_visible():
                line.set_data(xs, self.rcu.get_hist(key))
        self._pdu_ax.relim()
        self._pdu_ax.autoscale_view()
        self._pdu_canvas.draw_idle()

    def _update_imu(self, t):
        # Update value labels for both IMUs
        for prefix in ("imu0", "imu1"):
            ax_g = t.get(f"{prefix}_ax_g", 0.0)
            ay_g = t.get(f"{prefix}_ay_g", 0.0)
            az_g = t.get(f"{prefix}_az_g", 0.0)
            mag  = math.sqrt(ax_g**2 + ay_g**2 + az_g**2) if (ax_g or ay_g or az_g) else float('nan')
            mag_ok = 0.95 < mag < 1.05 if not math.isnan(mag) else False

            ax_label = self._imu.get(f"{prefix}_ax")
            if ax_label:
                ax_label.configure(
                    text=("---" if math.isnan(ax_g) else f"{ax_g:+.4f} g"),
                    text_color=TEXT if not math.isnan(ax_g) else ERROR)
            for ax_key, ax_val in [(f"{prefix}_ay", ay_g), (f"{prefix}_az", az_g)]:
                lbl = self._imu.get(ax_key)
                if lbl:
                    lbl.configure(text=("---" if math.isnan(ax_val) else f"{ax_val:+.4f} g"))
            mag_lbl = self._imu.get(f"{prefix}_mag")
            if mag_lbl:
                mag_lbl.configure(
                    text=("---" if math.isnan(mag) else f"{mag:.4f} g"),
                    text_color=OK if mag_ok else (WARN if not math.isnan(mag) else ERROR))
            for g_key, g_tkey in [
                (f"{prefix}_gx", f"{prefix}_gx_dps"),
                (f"{prefix}_gy", f"{prefix}_gy_dps"),
                (f"{prefix}_gz", f"{prefix}_gz_dps"),
            ]:
                gval = t.get(g_tkey, float('nan'))
                lbl = self._imu.get(g_key)
                if lbl:
                    lbl.configure(text=("---" if math.isnan(gval) else f"{gval:+.1f} dps"))
            imu_t = t.get(f"{prefix}_temp_c", float('nan'))
            temp_lbl = self._imu.get(f"{prefix}_temp")
            if temp_lbl:
                temp_lbl.configure(
                    text=("---" if math.isnan(imu_t) else f"{imu_t:.1f} \u00b0C"),
                    text_color=_color_label(imu_t, 5.0, 70.0) if not math.isnan(imu_t) else DIM)

            if prefix == "imu0":
                bt = t.get("ladc_therm0_c", float('nan'))
                bt_lbl = self._imu.get("imu0_btherm")
                if bt_lbl:
                    bt_lbl.configure(
                        text=("---" if math.isnan(bt) else f"{bt:.1f} \u00b0C"),
                        text_color=_color_label(bt, 5.0, 55.0) if not math.isnan(bt) else DIM)

        # Combined graph update
        xs = list(range(HISTORY_LEN))
        for key, line in self._imu_lines.items():
            if line.get_visible():
                line.set_data(xs, self.rcu.get_hist(key))
        self._imu_ax.relim()
        self._imu_ax.autoscale_view()
        self._imu_canvas.draw_idle()

    def _update_power(self, t):
        for key, entries in self._pw.items():
            raw = t.get(key, None)
            for (lbl, unit, lo_w, hi_w) in entries:
                if raw is None:
                    lbl.configure(text="\u2014\u2014\u2014", text_color=DIM)
                    continue
                color = _color_label(raw, lo_w, hi_w)
                if unit == "\u00b0C":
                    lbl.configure(text=f"{raw:.1f} {unit}", text_color=color)
                elif unit == "V":
                    lbl.configure(text=f"{raw:.3f} {unit}", text_color=color)
                else:
                    lbl.configure(text=f"{raw:.0f} {unit}", text_color=color)

    def _update_motors(self):
        slots = self.rcu.get_motor_slots()
        tree  = self._motor_tree
        for row in tree.get_children():
            tree.delete(row)
        for s in slots:
            bus_name = {0: "R", 1: "L"}.get(s["bus"], s["bus"])
            tree.insert("", "end", values=(
                bus_name, s["motor_id"],
                f"{s['pos_rad']:+.4f}",
                f"{s['vel_rads']:+.3f}",
                f"{s['torque_nm']:+.2f}",
                f"0x{s['error']:02X}" if s["error"] else "OK",
            ))

    def _log(self, msg):
        ts   = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}\n"
        self._log_lines.append(line)
        try:
            self._log_text.configure(state="normal")
            self._log_text.insert("end", line)
            self._log_text.see("end")
            if len(self._log_lines) >= 500:
                self._log_text.delete("1.0", "2.0")
            self._log_text.configure(state="disabled")
        except Exception:
            pass

    def on_close(self):
        if self._recv:
            self._recv.stop()
        self.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Plymouth Humanoid Bench Monitor")
    ap.add_argument("--rcu-ip", default=RCU_IP_DEFAULT,
                    help=f"RCU board IP (default {RCU_IP_DEFAULT})")
    args = ap.parse_args()
    app = App(rcu_ip=args.rcu_ip)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()


if __name__ == "__main__":
    main()
