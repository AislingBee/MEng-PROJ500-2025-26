from __future__ import annotations

"""Ros2RobotBridge – connects RobotHardwareInterface to the ROS2 topic network.

This module lives in the simulation package but depends on rclpy so it can
only be imported when a ROS2 environment is sourced.  It is intentionally NOT
imported by the Isaac-only code paths.

Usage
-----
In a ROS2-aware launch script or runner::

    import rclpy
    from simulation.isaac.rl.interface import RobotHardwareInterface, RobotInterfaceConfig
    from simulation.isaac.rl.interface.ros2_robot_bridge import Ros2RobotBridge

    bridge = Ros2RobotBridge(
        joint_names=JOINT_NAMES,
        command_topic='robot_command',
        observation_topic='robot_observation',
        node_name='rl_robot_bridge',
    )
    bridge.start()                  # starts background spin thread

    cfg = RobotInterfaceConfig(joint_names=JOINT_NAMES)
    hw  = RobotHardwareInterface(cfg, bridge.state_reader, bridge.command_writer)

    # … run RL policy loop …

    bridge.stop()

Wire-up
-------
The bridge publishes to the topic consumed by ``robot_command_bridge.py``
(Software/src/motor_test) and subscribes to the topic produced by
``robot_observation_bridge.py``.

Topic graph::

    RL policy
      │  write_control_packet(ControlPacket)
      ▼
    RobotHardwareInterface.command_writer  ──► /robot_command  ──► RobotCommandBridge
                                                                         │
                                              STM32 ◄──── EthernetCanBridge ◄── /motor_can_tx
                                              STM32 ────► EthernetCanBridge ──► /motor_can_feedback
                                                                         │
    RobotHardwareInterface.state_reader  ◄── /robot_observation ◄── RobotObservationBridge
      │
      ▼
    RL policy
      read_observation_packet() → ObservationPacket
"""

import math
import threading
import time
from typing import Sequence

import rclpy
from rclpy.node import Node

from .hardware_interface import BaseHardwareInterface, ControlPacket, ObservationPacket
from .robot_hardware_interface import (
    RobotCommandMessage,
    RobotHardwareInterface,
    RobotInterfaceConfig,
    RobotStateSample,
)

# Lazy import of ROS2 message types so the module can be imported before
# motor_test messages are sourced (useful in CI / bare Python tests).
try:
    from motor_test.msg import RobotCommand, RobotObservation  # type: ignore
    _MSGS_AVAILABLE = True
except ModuleNotFoundError:
    _MSGS_AVAILABLE = False


class Ros2RobotBridge:
    """Thin ROS2 node that wires ``RobotHardwareInterface`` to the topic bus.

    Parameters
    ----------
    joint_names:
        Ordered list of joint names.  Must match the order used by both the RL
        policy and the ``motor_names.json`` config in the motor_test package.
    command_topic:
        ROS2 topic on which ``RobotCommand`` messages are published.
    observation_topic:
        ROS2 topic on which ``RobotObservation`` messages are received.
    node_name:
        Name of the internal ROS2 node created by this bridge.
    encoder_cpr:
        Encoder counts per revolution used by ``RobotInterfaceConfig``.  The
        bridge uses this to convert radian positions received from the hardware
        back to synthetic encoder counts so that ``RobotHardwareInterface``
        can apply its calibrated sign/offset corrections.
    observation_timeout_s:
        ``state_reader()`` raises ``TimeoutError`` if no observation has arrived
        within this many seconds.  Set to 0 to disable.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        command_topic: str = 'robot_command',
        observation_topic: str = 'robot_observation',
        node_name: str = 'rl_robot_bridge',
        encoder_cpr: int = 16384,
        observation_timeout_s: float = 2.0,
    ) -> None:
        if not _MSGS_AVAILABLE:
            raise ImportError(
                'motor_test ROS2 messages not found.  '
                'Build and source the motor_test package before importing Ros2RobotBridge.'
            )

        self._joint_names = list(joint_names)
        self._encoder_cpr = encoder_cpr
        self._observation_timeout_s = observation_timeout_s

        self._lock = threading.Lock()
        self._latest_obs: RobotObservation | None = None
        self._obs_event = threading.Event()

        self._node: Node = rclpy.create_node(node_name)

        self._cmd_pub = self._node.create_publisher(
            RobotCommand, command_topic, 1
        )
        self._obs_sub = self._node.create_subscription(
            RobotObservation,
            observation_topic,
            self._observation_callback,
            1,
        )

        self._spin_thread: threading.Thread | None = None
        self._running = False

        self._node.get_logger().info(
            f'Ros2RobotBridge ready: '
            f'pub={command_topic!r}  sub={observation_topic!r}'
        )

    # ------------------------------------------------------------------
    # Background spin
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the ROS2 executor in a background thread."""
        if self._running:
            return
        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name='ros2_robot_bridge_spin'
        )
        self._spin_thread.start()

    def stop(self) -> None:
        """Stop the background spin thread and destroy the node."""
        self._running = False
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()

    def _spin_loop(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _observation_callback(self, msg: RobotObservation) -> None:
        with self._lock:
            self._latest_obs = msg
        self._obs_event.set()

    # ------------------------------------------------------------------
    # Callables for RobotHardwareInterface
    # ------------------------------------------------------------------

    def state_reader(self) -> RobotStateSample:
        """Return the latest robot state as a ``RobotStateSample``.

        Blocks up to ``observation_timeout_s`` for the first observation.
        Subsequent calls return the most recently cached message immediately.
        """
        if self._latest_obs is None:
            if self._observation_timeout_s > 0:
                arrived = self._obs_event.wait(timeout=self._observation_timeout_s)
                if not arrived:
                    raise TimeoutError(
                        f'No RobotObservation received within '
                        f'{self._observation_timeout_s:.1f} s.  '
                        f'Check that robot_observation_bridge is running.'
                    )
            else:
                # Busy-wait (degrades gracefully; prefer a positive timeout)
                while self._latest_obs is None:
                    time.sleep(0.005)

        with self._lock:
            obs = self._latest_obs

        # Convert joint_pos_rad → synthetic encoder counts so that
        # RobotHardwareInterface._counts_to_joint_pos_rad() round-trips
        # correctly (assuming sign=1, offset=0 defaults in RobotInterfaceConfig).
        #
        # Formula: counts = (q_rad / 2π) * encoder_cpr, restricted to [0, CPR).
        two_pi = 2.0 * math.pi
        encoder_counts = [
            int(((q % two_pi) / two_pi) * self._encoder_cpr) % self._encoder_cpr
            for q in obs.joint_pos_rad
        ]

        joint_vel = list(obs.joint_vel_rad_s) if obs.joint_vel_rad_s else None
        joint_effort = list(obs.joint_effort_nm) if obs.joint_effort_nm else None

        gravity = list(obs.projected_gravity_b) if len(obs.projected_gravity_b) == 3 \
            else [0.0, 0.0, -1.0]
        gyro = list(obs.imu_gyro_b) if len(obs.imu_gyro_b) == 3 \
            else [0.0, 0.0, 0.0]

        return RobotStateSample(
            encoder_counts=encoder_counts,
            projected_gravity_b=gravity,
            imu_gyro_b=gyro,
            joint_vel=joint_vel,
            joint_effort=joint_effort,
            timestamp_s=obs.timestamp_s if obs.timestamp_s > 0 else None,
        )

    def command_writer(self, msg: RobotCommandMessage) -> None:
        """Publish a ``RobotCommand`` message to the ROS2 topic bus."""
        ros_msg = RobotCommand()
        ros_msg.joint_names = list(msg.joint_names)
        ros_msg.q_des = [float(v) for v in msg.q_des]
        ros_msg.qd_des = [float(v) for v in msg.qd_des]
        ros_msg.kp = [float(v) for v in msg.kp]
        ros_msg.kd = [float(v) for v in msg.kd]
        ros_msg.tau_ff = [float(v) for v in msg.tau_ff]
        self._cmd_pub.publish(ros_msg)


# ---------------------------------------------------------------------------
# Convenience factory
# ---------------------------------------------------------------------------

def make_ros2_hardware_interface(
    joint_names: Sequence[str],
    cfg: RobotInterfaceConfig | None = None,
    command_topic: str = 'robot_command',
    observation_topic: str = 'robot_observation',
    node_name: str = 'rl_robot_bridge',
    device: str = 'cpu',
) -> tuple['BaseHardwareInterface', 'Ros2RobotBridge']:
    """Create a wired-up ``(RobotHardwareInterface, Ros2RobotBridge)`` pair.

    The bridge is started automatically.  Call ``bridge.stop()`` when done.

    Example::

        hw, bridge = make_ros2_hardware_interface(JOINT_NAMES)
        # … run RL loop …
        bridge.stop()
    """
    if cfg is None:
        cfg = RobotInterfaceConfig(joint_names=tuple(joint_names))

    bridge = Ros2RobotBridge(
        joint_names=joint_names,
        command_topic=command_topic,
        observation_topic=observation_topic,
        node_name=node_name,
        encoder_cpr=cfg.encoder_cpr,
    )
    bridge.start()

    hw = RobotHardwareInterface(
        cfg=cfg,
        state_reader=bridge.state_reader,
        command_writer=bridge.command_writer,
        device=device,
    )
    return hw, bridge
