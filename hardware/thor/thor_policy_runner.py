# Notion Documenation for this script.
# https://josephandrews.notion.site/Thor-Policy-Runner-Sim-to-Real-Deployment-Standing-Task-3456b3c9bc7680e99d5af51064cacd0f?source=copy_link

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import torch

from simulation.isaac.configuration.standing_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
    get_thor_runner_defaults,
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
    device: str = "cpu"
    loop_hz: float = CONTRACT.policy_loop_hz
    send_standing_pose_on_exit: bool = True

    # Debug print control
    debug_print: bool = False
    debug_print_every_n_steps: int = 50

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        if self.joint_names != CONTRACT.joint_names:
            raise ValueError("joint_names must match the standing S2R policy contract")
        if n != CONTRACT.action_dim:
            raise ValueError(f"Expected {CONTRACT.action_dim} joints, got {n}")
        if len(self.joint_lower_rad) != n:
            raise ValueError("joint_lower_rad length must match joint_names")
        if len(self.joint_upper_rad) != n:
            raise ValueError("joint_upper_rad length must match joint_names")
        if len(self.action_scale) != n:
            raise ValueError("action_scale length must match joint_names")
        if tuple(self.joint_lower_rad) != CONTRACT.joint_lower_limits_rad:
            raise ValueError("joint_lower_rad must match the standing S2R policy contract")
        if tuple(self.joint_upper_rad) != CONTRACT.joint_upper_limits_rad:
            raise ValueError("joint_upper_rad must match the standing S2R policy contract")
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")

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
        self._commands = torch.tensor(
            [[runner_cfg.command_value]], dtype=torch.float32, device=self.device
        )
        self._last_actions = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        self._tau_ff = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0)
        self._kd_fixed = kd_fixed.unsqueeze(0)

        self._step_count = 0
        self._last_obs: Tensor | None = None
        self._last_actions_debug: Tensor | None = None

    def build_observation(self) -> Tensor:
        packet = self.hardware.read_observation_packet()

        q_rel = packet.joint_pos - self._standing_q.unsqueeze(0).to(self.device)
        obs = torch.cat(
            (
                q_rel,
                packet.joint_vel,
                packet.joint_effort,
                packet.projected_gravity_b,
                packet.imu_gyro_b,
                self._commands,
                self._last_actions,
            ),
            dim=-1,
        )

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

        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
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
        )
        self.hardware.write_control_packet(packet)

    def run(self) -> None:
        period_s = 1.0 / self.cfg.loop_hz
        next_t = time.monotonic()

        try:
            while True:
                self.step()
                next_t += period_s
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_t = time.monotonic()
        except KeyboardInterrupt:
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

        q_rel = obs[:, 0:12]
        joint_vel = obs[:, 12:24]
        joint_effort = obs[:, 24:36]
        gravity_b = obs[:, 36:39]
        gyro_b = obs[:, 39:42]
        command = obs[:, 42:43]
        last_actions = obs[:, 43:55]

        print("\n" + "=" * 90)
        print(f"[THOR DEBUG] step={self._step_count}")
        print("-" * 90)

        print("[RECEIVED / OBSERVATION INPUT]")
        print("q_rel:", q_rel)
        print("joint_vel:", joint_vel)
        print("joint_effort:", joint_effort)
        print("projected_gravity_b:", gravity_b)
        print("imu_gyro_b:", gyro_b)
        print("command:", command)
        print("last_actions:", last_actions)

        print("\n[POLICY OUTPUT]")
        print("actions:", actions)
        print("actions min/max:", actions.min().item(), actions.max().item())

        print("\n[SENT / CONTROL OUTPUT]")
        print("joint order:")
        for i, name in enumerate(packet.joint_names):
            print(
                f"{i:02d} {name:40s} "
                f"q_des={q_des[0, i]:+ .5f} rad | "
                f"kp={kp[0, i]:+ .3f} | "
                f"kd={kd[0, i]:+ .3f} | "
                f"tau_ff={tau_ff[0, i]:+ .3f}"
            )

        print("=" * 90)

# -----------------------------------------------------------------------------
# Wiring example
# Replace these with the actual Thor ROS/CAN hooks.
# -----------------------------------------------------------------------------

def example_state_reader() -> RobotStateSample:
    raise NotImplementedError("Inject your real Thor state reader here")



def example_command_writer(msg: RobotCommandMessage) -> None:
    raise NotImplementedError("Inject your real Thor command writer here")



def main() -> None:
    contract_defaults = get_thor_runner_defaults()
    joint_names = contract_defaults["joint_names"]

    runner_cfg = ThorPolicyRunnerConfig(
        policy_path="exports/standing_policy.pt",
        joint_names=joint_names,
        joint_lower_rad=contract_defaults["joint_lower_limits_rad"],
        joint_upper_rad=contract_defaults["joint_upper_limits_rad"],
        device="cpu",
        loop_hz=contract_defaults["loop_hz"],
        command_value=contract_defaults["command_value"],
        debug_print=True,
        debug_print_every_n_steps=50,
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=tuple(1.0 for _ in joint_names),
    )

    runner = ThorStandingPolicyRunner(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=example_state_reader,
        command_writer=example_command_writer,
    )
    runner.run()


if __name__ == "__main__":
    main()
