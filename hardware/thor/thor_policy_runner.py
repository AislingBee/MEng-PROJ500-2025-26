# Notion Documenation for this script.
# https://josephandrews.notion.site/Thor-Policy-Runner-Sim-to-Real-Deployment-Standing-Task-3456b3c9bc7680e99d5af51064cacd0f?source=copy_link

from __future__ import annotations

import atexit
import os
import math
import time
import threading
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import torch

# Select policy contract.
# Use exactly one active import.
# Standing and walking use the same hardware interface.
# The selected contract defines the policy observation layout.
# from simulation.isaac.configuration.standing_s2r_policy_contract import (
#     CONTRACT,
#     build_fixed_gains,
#     build_standing_q,
# )

from simulation.isaac.configuration.walking_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)

from simulation.isaac.rl.interface.hardware_interface import ControlPacket
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotCommandMessage,
    RobotHardwareInterface,
    RobotInterfaceConfig,
    RobotStateSample,
)


Tensor = torch.Tensor


@dataclass
class ThorPolicyRunnerConfig:
    policy_path: str
    joint_names: tuple[str, ...] = CONTRACT.joint_names
    joint_lower_rad: tuple[float, ...] = CONTRACT.joint_lower_limits_rad
    joint_upper_rad: tuple[float, ...] = CONTRACT.joint_upper_limits_rad
    action_scale: tuple[float, ...] = CONTRACT.action_scale
    command_value: float = CONTRACT.default_command_value
    max_command_value: float = 0.50
    device: str = "cpu"
    loop_hz: float = CONTRACT.policy_loop_hz
    send_standing_pose_on_exit: bool = True

    # Debug print control
    debug_print: bool = False
    debug_print_every_n_steps: int = 50 # Set the frequency of DEBUG message.

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        if self.joint_names != CONTRACT.joint_names:
            raise ValueError("joint_names must match the selected S2R policy contract")
        if n != CONTRACT.action_dim:
            raise ValueError(f"Expected {CONTRACT.action_dim} joints, got {n}")
        if len(self.joint_lower_rad) != n:
            raise ValueError("joint_lower_rad length must match joint_names")
        if len(self.joint_upper_rad) != n:
            raise ValueError("joint_upper_rad length must match joint_names")
        if len(self.action_scale) != n:
            raise ValueError("action_scale length must match joint_names")
        if tuple(self.joint_lower_rad) != CONTRACT.joint_lower_limits_rad:
            raise ValueError("joint_lower_rad must match the selected S2R policy contract")
        if tuple(self.joint_upper_rad) != CONTRACT.joint_upper_limits_rad:
            raise ValueError("joint_upper_rad must match the selected S2R policy contract")
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.max_command_value < 0.0:
            raise ValueError("max_command_value must be non-negative")

        if self.debug_print_every_n_steps <= 0:
            raise ValueError("debug_print_every_n_steps must be positive")


class DeployablePolicy:
    """Loads a deployable actor module for inference.

    Supported forms:
      1) TorchScript module via torch.jit.load(...)
      2) torch.load(...) returning a callable torch.nn.Module

    This runner intentionally does not rebuild a training-time network from an
    RSL-RL checkpoint dict. Thor deployment should consume an exported actor,
    not a full training checkpoint.
    """

    def __init__(self, policy_path: str | Path, device: str | torch.device = "cpu"):
        self.device = torch.device(device)
        self.policy = self._load(policy_path)
        self.policy.eval()

    def _load(self, policy_path: str | Path):
        policy_path = str(policy_path)

        try:
            model = torch.jit.load(policy_path, map_location=self.device)
            return model
        except Exception:
            pass

        obj = torch.load(policy_path, map_location=self.device)
        if isinstance(obj, torch.nn.Module):
            obj.to(self.device)
            return obj

        if isinstance(obj, dict):
            raise RuntimeError(
                "Policy file loaded as a checkpoint dict, not a deployable actor module. "
                "Export the trained actor to TorchScript first, then use that .pt here."
            )

        raise RuntimeError(
            f"Unsupported policy file contents in '{policy_path}'. "
            "Expected TorchScript or a serialized torch.nn.Module."
        )

    @torch.inference_mode()
    def act(self, obs: Tensor) -> Tensor:
        out = self.policy({"policy": obs})

        if isinstance(out, dict):
            if "actions" in out:
                out = out["actions"]
            else:
                raise RuntimeError("Policy dict output does not contain 'actions'")
        elif isinstance(out, (tuple, list)):
            if not out:
                raise RuntimeError("Policy returned an empty tuple/list")
            out = out[0]

        out = torch.as_tensor(out, dtype=torch.float32, device=self.device)
        if out.ndim == 1:
            out = out.unsqueeze(0)
        expected_shape = (1, CONTRACT.action_dim)
        if out.shape != expected_shape:
            raise RuntimeError(f"Expected policy output shape {expected_shape}, got {tuple(out.shape)}")
        if torch.isnan(out).any():
            raise RuntimeError("NaN detected in policy output")
        return out


class ThorStandingPolicyRunner:
    def __init__(
        self,
        runner_cfg: ThorPolicyRunnerConfig,
        hardware_cfg: RobotInterfaceConfig,
        state_reader: Callable[[], RobotStateSample],
        command_writer: Callable[[RobotCommandMessage], None],
    ):
        self.cfg = runner_cfg
        self.device = torch.device(runner_cfg.device)

        self.hardware = RobotHardwareInterface(
            cfg=hardware_cfg,
            state_reader=state_reader,
            command_writer=command_writer,
            device=self.device,
        )
        self.policy = DeployablePolicy(runner_cfg.policy_path, device=self.device)

        self._joint_names = list(runner_cfg.joint_names)
        self._standing_q = build_standing_q(device=self.device)
        self._action_scale = torch.tensor(
            runner_cfg.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_lower = torch.tensor(
            runner_cfg.joint_lower_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_upper = torch.tensor(
            runner_cfg.joint_upper_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._commands = torch.zeros((1, 1), dtype=torch.float32, device=self.device)
        self._last_actions = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        self._joint_pos_targets = self._standing_q.unsqueeze(0).to(self.device).clone()
        self._tau_ff = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0)
        self._kd_fixed = kd_fixed.unsqueeze(0)

        # Per-joint PD gains for MIT control (conservative defaults: kp=30.0, kd=2.0)
        self._kp_gains = torch.full((1, CONTRACT.action_dim), 30.0, dtype=torch.float32, device=self.device)
        self._kd_gains = torch.full((1, CONTRACT.action_dim), 2.0, dtype=torch.float32, device=self.device)
        self.set_command_value(runner_cfg.command_value)

        self._step_count = 0
        self._last_obs: Tensor | None = None
        self._last_actions_debug: Tensor | None = None

    def _get_phase_clock(self) -> tuple[Tensor, Tensor]:
        gait_frequency_hz = getattr(CONTRACT, "default_gait_frequency_hz", None)
        if gait_frequency_hz is None:
            raise RuntimeError("Selected policy contract does not define a walking gait phase clock")

        phase = (self._step_count / self.cfg.loop_hz * gait_frequency_hz) % 1.0
        phase_angle = 2.0 * math.pi * phase
        phase_sin = torch.tensor([[math.sin(phase_angle)]], dtype=torch.float32, device=self.device)
        phase_cos = torch.tensor([[math.cos(phase_angle)]], dtype=torch.float32, device=self.device)
        return phase_sin, phase_cos

    def set_command_value(self, command_value: float) -> None:
        # command_value = 0.0 means stand
        # command_value > 0.0 means walking
        clamped_value = min(max(float(command_value), 0.0), self.cfg.max_command_value)
        self._commands[0, 0] = clamped_value

    def set_stand_mode(self) -> None:
        self.set_command_value(0.0)

    def set_walk_mode(self) -> None:
        self.set_command_value(0.05)

    def _build_observation_fields(self) -> tuple[tuple[str, Tensor], ...]:
        packet = self.hardware.read_observation_packet()

        q_rel = packet.joint_pos - self._standing_q.unsqueeze(0).to(self.device)
        standing_base_fields = (
            ("q_rel", q_rel),
            ("joint_vel", packet.joint_vel),
        )
        tail_fields = (
            ("joint_effort", packet.joint_effort),
            ("projected_gravity_b", packet.projected_gravity_b),
            ("imu_gyro_b", packet.imu_gyro_b),
            ("command", self._commands),
            ("last_actions", self._last_actions),
        )
        standing_fields = standing_base_fields + tail_fields
        if sum(field.shape[1] for _, field in standing_fields) == CONTRACT.obs_dim:
            return standing_fields

        q_target_err = self._joint_pos_targets - packet.joint_pos
        phase_sin, phase_cos = self._get_phase_clock()
        foot_pos_b = packet.foot_pos_b
        if foot_pos_b.shape[-1] != 6:
            raise RuntimeError(f"FK foot_pos_b must have trailing dim 6, got {foot_pos_b.shape[-1]}")
        walking_fields = (
            ("q_rel", q_rel),
            ("qd", packet.joint_vel),
            ("q_target_err", q_target_err),
            ("joint_effort", packet.joint_effort),
            ("projected_gravity_b", packet.projected_gravity_b),
            ("imu_gyro_b", packet.imu_gyro_b),
            ("command", self._commands),
            ("phase_sin", phase_sin),
            ("phase_cos", phase_cos),
            ("foot_pos_b", foot_pos_b),
            ("last_actions", self._last_actions),
        )

        if sum(field.shape[1] for _, field in walking_fields) == CONTRACT.obs_dim:
            return walking_fields

        raise RuntimeError(f"Selected policy contract has unsupported observation dim {CONTRACT.obs_dim}")

    def build_observation(self) -> Tensor:
        fields = self._build_observation_fields()
        obs = torch.cat(tuple(field for _, field in fields), dim=-1)

        obs = obs.to(self.device, dtype=torch.float32)
        expected_shape = (1, CONTRACT.obs_dim)
        if obs.shape != expected_shape:
            raise RuntimeError(f"Expected observation shape {expected_shape}, got {tuple(obs.shape)}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")
        return obs

    def generate_control_packet(self, actions: Tensor) -> ControlPacket:
        actions = torch.clamp(actions, -1.0, 1.0)
        q_des = self._standing_q.unsqueeze(0).to(self.device) + self._action_scale * actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)
        self._joint_pos_targets = q_des.detach().clone()

        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains.clone(),
            kd_gains=self._kd_gains.clone(),
        )

    def step(self) -> ControlPacket:
        obs = self.build_observation()
        actions = self.policy.act(obs)
        packet = self.generate_control_packet(actions)
        self.hardware.write_control_packet(packet)

        self._last_obs = obs.detach().clone()
        self._last_actions_debug = actions.detach().clone()

        self._last_actions = torch.clamp(actions, -1.0, 1.0).detach().clone()

        self._step_count += 1
        self._debug_print_step(packet)

        return packet

    def send_standing_pose(self) -> None:
        packet = ControlPacket(
            joint_names=self._joint_names,
            q_des=self._standing_q.unsqueeze(0).to(self.device).clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains.clone(),
            kd_gains=self._kd_gains.clone(),
        )
        self.hardware.write_control_packet(packet)

    def run(self, stop_event: threading.Event | None = None) -> None:
        period_s = 1.0 / self.cfg.loop_hz
        next_t = time.monotonic()

        try:
            while stop_event is None or not stop_event.is_set():
                self.step()
                next_t += period_s
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_t = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            if self.cfg.send_standing_pose_on_exit:
                self.send_standing_pose()


    def _debug_print_step(self, packet: ControlPacket) -> None:
        if not self.cfg.debug_print:
            return

        if self._step_count % self.cfg.debug_print_every_n_steps != 0:
            return

        if self._last_obs is None or self._last_actions_debug is None:
            return

        obs = self._last_obs.detach().cpu()
        actions = self._last_actions_debug.detach().cpu()
        q_des = packet.q_des.detach().cpu()
        kp = packet.kp.detach().cpu()
        kd = packet.kd.detach().cpu()
        tau_ff = packet.tau_ff.detach().cpu()

        if CONTRACT.obs_dim == 75:
            q_rel = obs[:, 0:12]
            joint_vel = obs[:, 12:24]
            q_target_err = obs[:, 24:36]
            joint_effort = obs[:, 36:48]
            gravity_b = obs[:, 48:51]
            gyro_b = obs[:, 51:54]
            command = obs[:, 54:55]
            phase_sin = obs[:, 55:56]
            phase_cos = obs[:, 56:57]
            foot_pos_b = obs[:, 57:63]
            last_actions = obs[:, 63:75]
        else:
            q_rel = obs[:, 0:12]
            joint_vel = obs[:, 12:24]
            q_target_err = None
            joint_effort = obs[:, 24:36]
            gravity_b = obs[:, 36:39]
            gyro_b = obs[:, 39:42]
            command = obs[:, 42:43]
            phase_sin = None
            phase_cos = None
            foot_pos_b = None
            last_actions = obs[:, 43:55]

        print("\n" + "=" * 90)
        print(f"[THOR DEBUG] step={self._step_count}")
        print("-" * 90)

        print("[RECEIVED / OBSERVATION INPUT]")
        print("q_rel:", q_rel)
        print("joint_vel:", joint_vel)
        if q_target_err is not None:
            print("q_target_err:", q_target_err)
        print("joint_effort:", joint_effort)
        print("projected_gravity_b:", gravity_b)
        print("imu_gyro_b:", gyro_b)
        print("command:", command)
        if phase_sin is not None and phase_cos is not None:
            print("phase_sin:", phase_sin)
            print("phase_cos:", phase_cos)
        if foot_pos_b is not None:
            print("foot_pos_b:", foot_pos_b)
        print("last_actions:", last_actions)

        print("\n[POLICY OUTPUT]")
        print("actions:", actions)
        print("actions min/max:", actions.min().item(), actions.max().item())

        print("\n[SENT / CONTROL OUTPUT]")
        print("joint order:")
        for i, name in enumerate(packet.joint_names):
            print(
                f"{i:02d} {name:40s} "
                f"q_des={q_des[0, i]:+.5f} rad | "
                f"kp={kp[0, i]:+.3f} | "
                f"kd={kd[0, i]:+.3f} | "
                f"tau_ff={tau_ff[0, i]:+.3f}"
            )

        print("=" * 90)

# -----------------------------------------------------------------------------
# Wiring example
# Replace these with the actual Thor ROS/CAN hooks.
# -----------------------------------------------------------------------------

_THOR_ROS_BRIDGE = None
_THOR_ROS_INIT_DONE_HERE = False


def _get_ros2_bridge():
    global _THOR_ROS_BRIDGE
    global _THOR_ROS_INIT_DONE_HERE

    if _THOR_ROS_BRIDGE is not None:
        return _THOR_ROS_BRIDGE

    try:
        import rclpy
        from simulation.isaac.rl.interface.ros2_robot_bridge import Ros2RobotBridge
    except Exception as exc:
        raise RuntimeError(
            "ROS2 bridge dependencies are unavailable. "
            "Source your ROS2 + workspace environment before running this script."
        ) from exc

    if not rclpy.ok():
        rclpy.init()
        _THOR_ROS_INIT_DONE_HERE = True

    _THOR_ROS_BRIDGE = Ros2RobotBridge(
        joint_names=CONTRACT.joint_names,
        command_topic=os.getenv("THOR_ROS_COMMAND_TOPIC", "robot_command"),
        observation_topic=os.getenv("THOR_ROS_OBSERVATION_TOPIC", "robot_observation"),
        node_name=os.getenv("THOR_ROS_NODE_NAME", "thor_policy_runner_bridge"),
        encoder_cpr=int(os.getenv("THOR_ENCODER_CPR", "16384")),
        observation_timeout_s=float(os.getenv("THOR_OBS_TIMEOUT_S", "2.0")),
    )
    _THOR_ROS_BRIDGE.start()
    return _THOR_ROS_BRIDGE


def _shutdown_ros2_bridge() -> None:
    global _THOR_ROS_BRIDGE
    global _THOR_ROS_INIT_DONE_HERE

    bridge = _THOR_ROS_BRIDGE
    _THOR_ROS_BRIDGE = None

    if bridge is not None:
        bridge.stop()

    if _THOR_ROS_INIT_DONE_HERE:
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    _THOR_ROS_INIT_DONE_HERE = False


atexit.register(_shutdown_ros2_bridge)

def ros2_state_reader() -> RobotStateSample:
    return _get_ros2_bridge().state_reader()



def ros2_command_writer(msg: RobotCommandMessage) -> None:
    _get_ros2_bridge().command_writer(msg)


def example_state_reader() -> RobotStateSample:
    return ros2_state_reader()


def example_command_writer(msg: RobotCommandMessage) -> None:
    ros2_command_writer(msg)



def main() -> None:
    import rclpy
    from simulation.isaac.rl.interface.ros2_robot_bridge import Ros2RobotBridge

    joint_names = CONTRACT.joint_names

    runner_cfg = ThorPolicyRunnerConfig(
        # policy_path=r"hardware\policy\standing_policy_200.pt",
        policy_path=r"hardware/policy/standing_policy_200.pt",
        joint_names=joint_names,
        joint_lower_rad=CONTRACT.joint_lower_limits_rad,
        joint_upper_rad=CONTRACT.joint_upper_limits_rad,
        device="cpu",
        loop_hz=CONTRACT.policy_loop_hz,
        command_value=CONTRACT.default_command_value,
        debug_print=True,
        debug_print_every_n_steps=50,
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=tuple(1.0 for _ in joint_names),
    )

    rclpy.init()
    bridge = Ros2RobotBridge(
        joint_names=joint_names,
        command_topic="robot_command",
        observation_topic="robot_observation",
        node_name="thor_policy_runner",
    )
    bridge.start()

    runner = ThorStandingPolicyRunner(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=bridge.state_reader,
        command_writer=bridge.command_writer,
    )
    stop_event = threading.Event()

    def keyboard_loop() -> None:
        while not stop_event.is_set():
            try:
                key = input().strip().lower()
            except EOFError:
                break

            if key == "s":
                runner.set_stand_mode()
            elif key == "w":
                runner.set_walk_mode()
            elif key == "x":
                runner.set_stand_mode()
                stop_event.set()
                break

    keyboard_thread = threading.Thread(target=keyboard_loop, daemon=True)
    keyboard_thread.start()

    try:
        runner.run(stop_event=stop_event)
    finally:
        bridge.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
