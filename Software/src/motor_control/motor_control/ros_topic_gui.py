#!/usr/bin/env python3
"""
ros_topic_gui.py — Live ROS2 topic monitor with per-motor dashboard.

Tabs:
  Motors   — 12 motors in 6 L/R joint pairs (pos, vel from /motor_feedback;
             commanded values from /robot_command)
  Topics   — All ROS2 topics discovered on the graph with type and live Hz
  IMU      — /imu0 and /imu0_remapped accel / gyro / orientation
  Command  — /robot_command per-joint table (q_des, qd_des, Kp, Kd, tau_ff)
  PDU Telem— /rcu_pdu_telem JSON key-value display

Usage:
  ros2 run motor_control ros_topic_gui
  ros2 run motor_control ros_topic_gui --ros-args -p names_file:=joint_limits_config.json
"""

import collections
import json
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt8MultiArray

import tkinter as tk
from tkinter import ttk

from motor_control.msg import MotorFeedback, RobotCommand, RobotObservation
from motor_control.common import load_motor_names

# ── Constants ─────────────────────────────────────────────────────────────────

REFRESH_MS   = 100   # GUI poll period in milliseconds
HZ_WINDOW_S  = 2.0   # rolling window used for Hz estimation
STALE_WARN_S = 0.5   # amber threshold: data older than this
STALE_ERR_S  = 2.0   # red   threshold: data older than this

# Joint group labels and (left_index, right_index) pairs.
# Motors are ordered by joint_limits_config.json: index 0=L yaw, 1=R yaw, …
JOINT_GROUPS = ["Hip Yaw", "Hip Pitch", "Hip Roll", "Knee", "Ankle", "Foot"]
JOINT_PAIRS  = [(0, 1), (2, 3), (4, 5), (6, 7), (8, 9), (10, 11)]

# Colour scheme (dark-theme)
C_BG    = "#1e1e1e"
C_BG2   = "#252526"
C_HDR   = "#2d2d2d"
C_FG    = "#d4d4d4"
C_BLUE  = "#9cdcfe"
C_OK    = "#4ec94e"
C_WARN  = "#ff9900"
C_ERR   = "#ff4444"
C_NONE  = "#666666"
C_LEFT  = "#4ec94e"
C_RIGHT = "#4fc1ff"
C_NAME  = "#ce9178"
C_NUM   = "#b5cea8"

FMT_F   = "{:+9.4f}"   # numeric float format
FMT_HZ  = "{:6.1f}"   # Hz format


# ── Hz tracker ────────────────────────────────────────────────────────────────

class HzTracker:
    """Rolling-window message-rate estimator."""

    def __init__(self, window_s: float = HZ_WINDOW_S):
        self._window = window_s
        self._stamps: collections.deque = collections.deque()
        self._last_t: Optional[float] = None

    def tick(self) -> None:
        now = time.monotonic()
        self._stamps.append(now)
        self._last_t = now
        cutoff = now - self._window
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    @property
    def hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return 0.0 if span <= 0.0 else (len(self._stamps) - 1) / span

    @property
    def age_s(self) -> float:
        return float("inf") if self._last_t is None else time.monotonic() - self._last_t


def _age_color(age_s: float) -> str:
    if age_s > STALE_ERR_S:
        return C_ERR
    if age_s > STALE_WARN_S:
        return C_WARN
    return C_OK


# ── ROS2 subscriber node ──────────────────────────────────────────────────────

class TopicGuiNode(Node):
    """Subscribes to the PROJ500 motor stack topics and stores latest data."""

    KNOWN_TOPICS = [
        "/motor_can_feedback",
        "/motor_feedback",
        "/imu0",
        "/imu0_remapped",
        "/robot_observation",
        "/robot_command",
        "/rcu_pdu_telem",
        "/motor_can_tx",
    ]

    def __init__(self):
        super().__init__("ros_topic_gui")
        self.declare_parameter("names_file", "joint_limits_config.json")
        names_file = self.get_parameter("names_file").value

        self._lock = threading.Lock()

        self.motor_names: List[str] = load_motor_names(
            names_file, 12, self.get_logger()
        )

        # Latest message store
        self.motor_feedback:    Optional[MotorFeedback]      = None
        self.imu0:              Optional[Imu]                = None
        self.imu0_remapped:     Optional[Imu]                = None
        self.robot_observation: Optional[RobotObservation]   = None
        self.robot_command:     Optional[RobotCommand]       = None
        self.pdu_telem_str:     str                          = ""

        # Per-topic Hz trackers
        self.hz: Dict[str, HzTracker] = {t: HzTracker() for t in self.KNOWN_TOPICS}

        # Subscriptions
        self.create_subscription(
            UInt8MultiArray, "motor_can_feedback", self._cb_can_rx, 10)
        self.create_subscription(
            MotorFeedback,   "motor_feedback",     self._cb_motor_feedback, 10)
        self.create_subscription(
            Imu,             "imu0",               self._cb_imu0, 10)
        self.create_subscription(
            Imu,             "imu0_remapped",      self._cb_imu0_remapped, 10)
        self.create_subscription(
            RobotObservation,"robot_observation",  self._cb_observation, 10)
        self.create_subscription(
            RobotCommand,    "robot_command",      self._cb_command, 10)
        self.create_subscription(
            String,          "rcu_pdu_telem",      self._cb_pdu_telem, 10)
        self.create_subscription(
            UInt8MultiArray, "motor_can_tx",       self._cb_can_tx, 10)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _tick(self, topic: str) -> None:
        if topic in self.hz:
            self.hz[topic].tick()

    def _cb_can_rx(self, _msg: UInt8MultiArray) -> None:
        self._tick("/motor_can_feedback")

    def _cb_motor_feedback(self, msg: MotorFeedback) -> None:
        self._tick("/motor_feedback")
        with self._lock:
            self.motor_feedback = msg

    def _cb_imu0(self, msg: Imu) -> None:
        self._tick("/imu0")
        with self._lock:
            self.imu0 = msg

    def _cb_imu0_remapped(self, msg: Imu) -> None:
        self._tick("/imu0_remapped")
        with self._lock:
            self.imu0_remapped = msg

    def _cb_observation(self, msg: RobotObservation) -> None:
        self._tick("/robot_observation")
        with self._lock:
            self.robot_observation = msg

    def _cb_command(self, msg: RobotCommand) -> None:
        self._tick("/robot_command")
        with self._lock:
            self.robot_command = msg

    def _cb_pdu_telem(self, msg: String) -> None:
        self._tick("/rcu_pdu_telem")
        with self._lock:
            self.pdu_telem_str = msg.data

    def _cb_can_tx(self, _msg: UInt8MultiArray) -> None:
        self._tick("/motor_can_tx")

    # ── Thread-safe snapshot ─────────────────────────────────────────────────

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "motor_feedback":    self.motor_feedback,
                "imu0":              self.imu0,
                "imu0_remapped":     self.imu0_remapped,
                "robot_observation": self.robot_observation,
                "robot_command":     self.robot_command,
                "pdu_telem_str":     self.pdu_telem_str,
            }


# ── GUI ───────────────────────────────────────────────────────────────────────

class RosTopicGui:

    def __init__(self, node: TopicGuiNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title("PROJ500 — ROS2 Topic Monitor")
        self.root.configure(bg=C_BG)
        self.root.geometry("1280x720")
        self.root.minsize(900, 500)

        self._apply_style()

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=6, pady=6)

        self._build_motors_tab()
        self._build_topics_tab()
        self._build_imu_tab()
        self._build_command_tab()
        self._build_pdu_tab()

        self.root.after(REFRESH_MS, self._refresh)

    # ── Styling ───────────────────────────────────────────────────────────────

    def _apply_style(self) -> None:
        s = ttk.Style(self.root)
        s.theme_use("clam")
        s.configure(".",
                     background=C_BG, foreground=C_FG,
                     fieldbackground=C_HDR, troughcolor=C_HDR,
                     bordercolor=C_HDR, darkcolor=C_BG, lightcolor=C_BG)
        s.configure("TNotebook", background=C_BG, borderwidth=0)
        s.configure("TNotebook.Tab",
                     background=C_HDR, foreground=C_FG,
                     padding=[12, 5], font=("Consolas", 9))
        s.map("TNotebook.Tab",
              background=[("selected", "#0078d4")],
              foreground=[("selected", "#ffffff")])
        s.configure("TFrame", background=C_BG)
        s.configure("TLabelframe", background=C_BG, bordercolor="#444444")
        s.configure("TLabelframe.Label",
                     background=C_BG, foreground=C_BLUE,
                     font=("Consolas", 9, "bold"))
        s.configure("Treeview",
                     background=C_HDR, foreground=C_FG,
                     fieldbackground=C_HDR, rowheight=22,
                     font=("Consolas", 9))
        s.configure("Treeview.Heading",
                     background=C_HDR, foreground=C_BLUE,
                     font=("Consolas", 9, "bold"))
        s.map("Treeview",
              background=[("selected", "#094771")],
              foreground=[("selected", "#ffffff")])
        s.configure("Vertical.TScrollbar",
                     background=C_HDR, troughcolor=C_BG, arrowcolor=C_FG)

    # ── Helper label factory ─────────────────────────────────────────────────

    @staticmethod
    def _lbl(parent, text, *, bg=C_BG, fg=C_FG,
             font=("Consolas", 9), anchor="center", **grid_kw) -> tk.Label:
        lbl = tk.Label(parent, text=text, bg=bg, fg=fg,
                       font=font, anchor=anchor, padx=4, pady=2)
        if grid_kw:
            lbl.grid(**grid_kw)
        return lbl

    # ── Motors tab ────────────────────────────────────────────────────────────

    def _build_motors_tab(self) -> None:
        outer = ttk.Frame(self.notebook)
        self.notebook.add(outer, text="  Motors  ")

        # Scrollable canvas
        canvas = tk.Canvas(outer, bg=C_BG, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        frame = tk.Frame(canvas, bg=C_BG)
        win_id = canvas.create_window((0, 0), window=frame, anchor="nw")

        def _on_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(win_id, width=canvas.winfo_width())

        frame.bind("<Configure>", _on_configure)
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win_id, width=e.width))

        # Mouse-wheel scrolling
        def _on_wheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_wheel)

        # Column headers
        col_defs = [
            ("Joint Group",     "#ce9178", 120),
            ("ID",              C_BLUE,    40),
            ("Side",            C_BLUE,    45),
            ("Name",            C_BLUE,    210),
            ("Pos (rad)",       C_BLUE,    105),
            ("Vel (rad/s)",     C_BLUE,    105),
            ("Cmd Pos (rad)",   C_BLUE,    110),
            ("Cmd Kp",          C_BLUE,    80),
            ("Cmd Kd",          C_BLUE,    80),
            ("Cmd tau_ff(Nm)",  C_BLUE,    105),
        ]
        for col, (hdr, fg, w) in enumerate(col_defs):
            lbl = tk.Label(frame, text=hdr, bg=C_HDR, fg=fg,
                           font=("Consolas", 9, "bold"), padx=4, pady=5,
                           anchor="center", width=w // 8)
            lbl.grid(row=0, column=col, sticky="ew", padx=1, pady=1)
            frame.grid_columnconfigure(col, weight=1, minsize=w)

        # Data rows — one per motor
        self._motor_cells: List[Dict[str, tk.Label]] = []

        for group_idx, (l_idx, r_idx) in enumerate(JOINT_PAIRS):
            for side_ord, motor_idx in enumerate([l_idx, r_idx]):
                grid_row = group_idx * 2 + side_ord + 1
                motor_id = motor_idx + 1
                side     = "L" if side_ord == 0 else "R"
                side_fg  = C_LEFT if side == "L" else C_RIGHT
                grp_text = JOINT_GROUPS[group_idx] if side_ord == 0 else ""
                bg       = C_BG if group_idx % 2 == 0 else C_BG2
                name_txt = (self.node.motor_names[motor_idx]
                            if motor_idx < len(self.node.motor_names)
                            else f"motor_{motor_id}")

                def _mk(txt, col, fg=C_FG, r=grid_row, b=bg):
                    lbl = tk.Label(frame, text=txt, bg=b, fg=fg,
                                   font=("Consolas", 9), padx=4, pady=3,
                                   anchor="center")
                    lbl.grid(row=r, column=col, sticky="ew", padx=1, pady=0)
                    return lbl

                grp_fg = C_NAME if side_ord == 0 else "#444444"
                _mk(grp_text, 0, fg=grp_fg)
                _mk(str(motor_id), 1, fg=C_NUM)
                _mk(side, 2, fg=side_fg)
                _mk(name_txt, 3, fg=C_FG)

                cells: Dict[str, tk.Label] = {
                    "motor_idx": motor_idx,
                    "pos":       _mk("–", 4),
                    "vel":       _mk("–", 5),
                    "cmd_pos":   _mk("–", 6),
                    "cmd_kp":    _mk("–", 7),
                    "cmd_kd":    _mk("–", 8),
                    "cmd_tau":   _mk("–", 9),
                }
                self._motor_cells.append(cells)

    # ── Topics tab ────────────────────────────────────────────────────────────

    def _build_topics_tab(self) -> None:
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="  Topics  ")

        cols = ("topic", "type", "hz", "age", "pub", "status")
        tree = ttk.Treeview(frame, columns=cols, show="headings", height=28)
        self._topics_tree = tree

        col_cfg = [
            ("topic",  "Topic",      310, "w"),
            ("type",   "Msg Type",   270, "w"),
            ("hz",     "Hz",          80, "center"),
            ("age",    "Age (s)",      75, "center"),
            ("pub",    "Publishers",   80, "center"),
            ("status", "",             30, "center"),
        ]
        for col, heading, width, anchor in col_cfg:
            tree.heading(col, text=heading)
            tree.column(col, width=width, anchor=anchor, minwidth=40)

        tree.tag_configure("ok",   foreground=C_OK)
        tree.tag_configure("warn", foreground=C_WARN)
        tree.tag_configure("err",  foreground=C_ERR)
        tree.tag_configure("none", foreground=C_NONE)

        vsb = ttk.Scrollbar(frame, orient="vertical",   command=tree.yview)
        hsb = ttk.Scrollbar(frame, orient="horizontal", command=tree.xview)
        tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)

        vsb.pack(side="right",  fill="y")
        hsb.pack(side="bottom", fill="x")
        tree.pack(fill="both",  expand=True)

        # Pre-insert known topics so they always appear first
        self._topic_iids: Dict[str, str] = {}
        for t in self.node.KNOWN_TOPICS:
            iid = tree.insert("", "end", values=(t, "–", "–", "–", "–", "○"),
                              tags=("none",))
            self._topic_iids[t] = iid

    # ── IMU tab ───────────────────────────────────────────────────────────────

    def _build_imu_tab(self) -> None:
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="  IMU  ")
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_rowconfigure(0, weight=1)

        self._imu_w: Dict[str, Dict[str, tk.Label]] = {}

        imu_configs = [
            ("imu0",          "/imu0          (raw from RCU)"),
            ("imu0_remapped", "/imu0_remapped  (policy frame)"),
        ]
        for col, (key, label) in enumerate(imu_configs):
            lf = ttk.LabelFrame(frame, text=label, padding=12)
            lf.grid(row=0, column=col, sticky="nsew", padx=10, pady=10)

            fields = [
                ("Hz",              "hz"),
                ("Age (s)",         "age"),
                ("",                "sep1"),
                ("Accel X  (g)",    "ax"),
                ("Accel Y  (g)",    "ay"),
                ("Accel Z  (g)",    "az"),
                ("",                "sep2"),
                ("Gyro X  (rad/s)", "gx"),
                ("Gyro Y  (rad/s)", "gy"),
                ("Gyro Z  (rad/s)", "gz"),
                ("",                "sep3"),
                ("Quat W",          "qw"),
                ("Quat X",          "qx"),
                ("Quat Y",          "qy"),
                ("Quat Z",          "qz"),
            ]
            widgets: Dict[str, tk.Label] = {}
            r = 0
            for lbl_txt, fkey in fields:
                if lbl_txt == "":
                    tk.Frame(lf, bg="#444444", height=1).grid(
                        row=r, column=0, columnspan=2, sticky="ew", pady=4)
                    r += 1
                    continue
                tk.Label(lf, text=lbl_txt + ":", bg=C_BG, fg=C_BLUE,
                         font=("Consolas", 9), anchor="w", width=16
                         ).grid(row=r, column=0, sticky="w", pady=1)
                val = tk.Label(lf, text="–", bg=C_BG, fg=C_FG,
                               font=("Consolas", 10, "bold"), anchor="w", width=14)
                val.grid(row=r, column=1, sticky="w", pady=1)
                widgets[fkey] = val
                r += 1
            self._imu_w[key] = widgets

    # ── Command tab ───────────────────────────────────────────────────────────

    def _build_command_tab(self) -> None:
        outer = ttk.Frame(self.notebook)
        self.notebook.add(outer, text="  Command  ")

        # Rate label at top
        top = tk.Frame(outer, bg=C_BG)
        top.pack(fill="x", padx=6, pady=4)
        tk.Label(top, text="/robot_command", bg=C_BG, fg=C_BLUE,
                 font=("Consolas", 10, "bold")).pack(side="left")
        self._cmd_hz_lbl = tk.Label(top, text="no data", bg=C_BG, fg=C_NONE,
                                    font=("Consolas", 10))
        self._cmd_hz_lbl.pack(side="right", padx=8)

        # Table
        frame = tk.Frame(outer, bg=C_BG)
        frame.pack(fill="both", expand=True, padx=6, pady=2)

        col_defs = [
            ("ID",          C_BLUE,  40),
            ("Joint Name",  C_BLUE, 230),
            ("q_des (rad)", C_BLUE, 110),
            ("qd_des (r/s)",C_BLUE, 110),
            ("Kp",          C_BLUE,  80),
            ("Kd",          C_BLUE,  80),
            ("tau_ff (Nm)", C_BLUE, 100),
        ]
        for col, (hdr, fg, w) in enumerate(col_defs):
            tk.Label(frame, text=hdr, bg=C_HDR, fg=fg,
                     font=("Consolas", 9, "bold"), padx=4, pady=5,
                     anchor="center").grid(row=0, column=col, sticky="ew",
                                           padx=1, pady=1)
            frame.grid_columnconfigure(col, weight=1, minsize=w)

        self._cmd_rows: List[Dict[str, tk.Label]] = []
        for i in range(12):
            bg = C_BG if i % 2 == 0 else C_BG2
            motor_id = i + 1
            name = (self.node.motor_names[i]
                    if i < len(self.node.motor_names)
                    else f"motor_{motor_id}")

            def _mk(txt, col, fg=C_FG, r=i + 1, b=bg):
                lbl = tk.Label(frame, text=txt, bg=b, fg=fg,
                               font=("Consolas", 9), padx=4, pady=3,
                               anchor="center")
                lbl.grid(row=r, column=col, sticky="ew", padx=1)
                return lbl

            _mk(str(motor_id), 0, C_NUM)
            _mk(name, 1, C_NAME)
            self._cmd_rows.append({
                "q_des":  _mk("–", 2),
                "qd_des": _mk("–", 3),
                "kp":     _mk("–", 4),
                "kd":     _mk("–", 5),
                "tau_ff": _mk("–", 6),
            })

    # ── PDU Telem tab ─────────────────────────────────────────────────────────

    def _build_pdu_tab(self) -> None:
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="  PDU Telem  ")

        top = tk.Frame(frame, bg=C_BG)
        top.pack(fill="x", padx=6, pady=4)
        tk.Label(top, text="/rcu_pdu_telem", bg=C_BG, fg=C_BLUE,
                 font=("Consolas", 10, "bold")).pack(side="left")
        self._pdu_hz_lbl = tk.Label(top, text="no data", bg=C_BG, fg=C_NONE,
                                    font=("Consolas", 10))
        self._pdu_hz_lbl.pack(side="right", padx=8)

        cols = ("key", "value")
        tree = ttk.Treeview(frame, columns=cols, show="headings", height=22)
        self._pdu_tree = tree
        tree.heading("key",   text="Field")
        tree.heading("value", text="Value")
        tree.column("key",   width=240, anchor="w")
        tree.column("value", width=380, anchor="w")

        vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        tree.pack(fill="both", expand=True, padx=6, pady=4)

        self._pdu_iids: Dict[str, str] = {}

    # ── Refresh cycle ─────────────────────────────────────────────────────────

    def _refresh(self) -> None:
        snap = self.node.snapshot()
        hz   = self.node.hz

        self._refresh_motors(snap, hz)
        self._refresh_topics(hz)
        self._refresh_imu(snap, hz)
        self._refresh_command(snap, hz)
        self._refresh_pdu(snap, hz)

        self.root.after(REFRESH_MS, self._refresh)

    # ── Motors refresh ────────────────────────────────────────────────────────

    def _refresh_motors(self, snap: dict, hz: Dict[str, HzTracker]) -> None:
        fb      = snap["motor_feedback"]
        cmd     = snap["robot_command"]
        fb_age  = hz["/motor_feedback"].age_s

        # Build lookup: joint_name → command values
        cmd_map: Dict[str, dict] = {}
        if cmd is not None:
            for i, name in enumerate(cmd.joint_names):
                cmd_map[name] = {
                    "q_des":  cmd.q_des[i]  if i < len(cmd.q_des)  else None,
                    "qd_des": cmd.qd_des[i] if i < len(cmd.qd_des) else None,
                    "kp":     cmd.kp[i]     if i < len(cmd.kp)     else None,
                    "kd":     cmd.kd[i]     if i < len(cmd.kd)     else None,
                    "tau_ff": cmd.tau_ff[i] if i < len(cmd.tau_ff) else None,
                }

        fb_color = _age_color(fb_age)

        for cells in self._motor_cells:
            midx = cells["motor_idx"]

            # Feedback
            if fb is not None and midx < len(fb.motors):
                m = fb.motors[midx]
                cells["pos"].config(text=FMT_F.format(float(m.q)),     fg=fb_color)
                cells["vel"].config(text=FMT_F.format(float(m.q_dot)), fg=fb_color)
            else:
                cells["pos"].config(text="–", fg=C_NONE)
                cells["vel"].config(text="–", fg=C_NONE)

            # Command
            name = (self.node.motor_names[midx]
                    if midx < len(self.node.motor_names) else "")
            c = cmd_map.get(name)
            if c:
                cmd_age = hz["/robot_command"].age_s
                c_fg = _age_color(cmd_age)
                cells["cmd_pos"].config(
                    text=FMT_F.format(c["q_des"])  if c["q_des"]  is not None else "–",
                    fg=c_fg)
                cells["cmd_kp"].config(
                    text=FMT_F.format(c["kp"])     if c["kp"]     is not None else "–",
                    fg=c_fg)
                cells["cmd_kd"].config(
                    text=FMT_F.format(c["kd"])     if c["kd"]     is not None else "–",
                    fg=c_fg)
                cells["cmd_tau"].config(
                    text=FMT_F.format(c["tau_ff"]) if c["tau_ff"] is not None else "–",
                    fg=c_fg)
            else:
                for k in ("cmd_pos", "cmd_kp", "cmd_kd", "cmd_tau"):
                    cells[k].config(text="–", fg=C_NONE)

    # ── Topics refresh ────────────────────────────────────────────────────────

    def _refresh_topics(self, hz: Dict[str, HzTracker]) -> None:
        # Query the ROS2 graph for all live topics
        try:
            graph_topics: Dict[str, List[str]] = {
                t: types
                for t, types in self.node.get_topic_names_and_types()
            }
        except Exception:
            graph_topics = {}

        # Count publishers per topic from the graph
        pub_counts: Dict[str, int] = {}
        try:
            for t in graph_topics:
                pub_counts[t] = len(self.node.get_publishers_info_by_topic(t))
        except Exception:
            pass

        def _type_str(topic: str) -> str:
            types = graph_topics.get(topic, [])
            if not types:
                return "–"
            # Format: "motor_control/msg/MotorFeedback"
            return types[0]

        def _tag(age_s: float, has_pub: bool) -> str:
            if not has_pub:
                return "none"
            if age_s > STALE_ERR_S:
                return "err"
            if age_s > STALE_WARN_S:
                return "warn"
            return "ok"

        # Update known topic rows
        for topic, iid in self._topic_iids.items():
            tracker  = hz.get(topic)
            age      = tracker.age_s if tracker else float("inf")
            hz_val   = tracker.hz    if tracker else 0.0
            type_s   = _type_str(topic)
            n_pub    = pub_counts.get(topic, 0)
            age_s    = f"{age:.1f}" if age < 9999 else "∞"
            hz_s     = f"{hz_val:.1f}" if hz_val > 0 else "0.0"
            status   = "●" if age < STALE_ERR_S else "○"
            tag      = _tag(age, n_pub > 0 or hz_val > 0)

            self._topics_tree.item(
                iid,
                values=(topic, type_s, hz_s, age_s, str(n_pub), status),
                tags=(tag,),
            )

        # Insert any newly discovered topics not already listed
        for topic in sorted(graph_topics.keys()):
            if topic in self._topic_iids:
                continue
            type_s  = _type_str(topic)
            n_pub   = pub_counts.get(topic, 0)
            iid = self._topics_tree.insert(
                "", "end",
                values=(topic, type_s, "?", "?", str(n_pub), "○"),
                tags=("none",),
            )
            self._topic_iids[topic] = iid

    # ── IMU refresh ───────────────────────────────────────────────────────────

    def _refresh_imu(self, snap: dict, hz: Dict[str, HzTracker]) -> None:
        for key, topic in [("imu0", "/imu0"), ("imu0_remapped", "/imu0_remapped")]:
            msg: Optional[Imu] = snap[key]
            tracker = hz[topic]
            w = self._imu_w[key]
            age = tracker.age_s
            fg  = _age_color(age)

            w["hz"].config(
                text=f"{tracker.hz:.1f} Hz",
                fg=fg if age < 9999 else C_NONE,
            )
            w["age"].config(
                text=f"{age:.2f} s" if age < 9999 else "∞",
                fg=fg if age < 9999 else C_NONE,
            )

            if msg is not None:
                a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
                for fkey, val in [
                    ("ax", a.x), ("ay", a.y), ("az", a.z),
                    ("gx", g.x), ("gy", g.y), ("gz", g.z),
                    ("qw", q.w), ("qx", q.x), ("qy", q.y), ("qz", q.z),
                ]:
                    w[fkey].config(text=f"{val:+.5f}", fg=fg)
            else:
                for fkey in ("ax", "ay", "az", "gx", "gy", "gz",
                              "qw", "qx", "qy", "qz"):
                    w[fkey].config(text="–", fg=C_NONE)

    # ── Command refresh ───────────────────────────────────────────────────────

    def _refresh_command(self, snap: dict, hz: Dict[str, HzTracker]) -> None:
        cmd     = snap["robot_command"]
        tracker = hz["/robot_command"]
        age     = tracker.age_s
        fg      = _age_color(age)

        self._cmd_hz_lbl.config(
            text=(f"{tracker.hz:.1f} Hz  age={age:.2f}s"
                  if age < 9999 else "no data"),
            fg=fg if age < 9999 else C_NONE,
        )

        cmd_by_name: Dict[str, dict] = {}
        if cmd is not None:
            for i, name in enumerate(cmd.joint_names):
                cmd_by_name[name] = {
                    "q_des":  cmd.q_des[i]  if i < len(cmd.q_des)  else 0.0,
                    "qd_des": cmd.qd_des[i] if i < len(cmd.qd_des) else 0.0,
                    "kp":     cmd.kp[i]     if i < len(cmd.kp)     else 0.0,
                    "kd":     cmd.kd[i]     if i < len(cmd.kd)     else 0.0,
                    "tau_ff": cmd.tau_ff[i] if i < len(cmd.tau_ff) else 0.0,
                }

        for i, row in enumerate(self._cmd_rows):
            name = (self.node.motor_names[i]
                    if i < len(self.node.motor_names) else f"motor_{i+1}")
            c = cmd_by_name.get(name)
            if c:
                row["q_des"].config( text=FMT_F.format(c["q_des"]),  fg=fg)
                row["qd_des"].config(text=FMT_F.format(c["qd_des"]), fg=fg)
                row["kp"].config(    text=FMT_F.format(c["kp"]),     fg=fg)
                row["kd"].config(    text=FMT_F.format(c["kd"]),     fg=fg)
                row["tau_ff"].config(text=FMT_F.format(c["tau_ff"]), fg=fg)
            else:
                for k in row:
                    row[k].config(text="–", fg=C_NONE)

    # ── PDU Telem refresh ─────────────────────────────────────────────────────

    def _refresh_pdu(self, snap: dict, hz: Dict[str, HzTracker]) -> None:
        tracker  = hz["/rcu_pdu_telem"]
        age      = tracker.age_s
        fg       = _age_color(age)

        self._pdu_hz_lbl.config(
            text=(f"{tracker.hz:.1f} Hz  age={age:.1f}s"
                  if age < 9999 else "no data"),
            fg=fg if age < 9999 else C_NONE,
        )

        raw = snap["pdu_telem_str"]
        if not raw:
            return

        try:
            data: dict = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            data = {"raw": raw}

        for k, v in data.items():
            v_str = f"{v:.4f}" if isinstance(v, float) else str(v)
            if k in self._pdu_iids:
                self._pdu_tree.item(self._pdu_iids[k], values=(k, v_str))
            else:
                iid = self._pdu_tree.insert("", "end", values=(k, v_str))
                self._pdu_iids[k] = iid

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self) -> None:
        self.root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TopicGuiNode()

    # Spin ROS2 in a daemon thread so it exits cleanly when the GUI closes
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        gui = RosTopicGui(node)
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
