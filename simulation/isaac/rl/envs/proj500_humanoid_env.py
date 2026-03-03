import torch

from isaaclab.envs import DirectRLEnv
from isaaclab.assets import Articulation
from isaaclab.utils.math import quat_rotate_inverse

from .proj500_humanoid_cfg import Proj500HumanoidCfg


JOINT_NAMES = [
    "l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint", "l_knee_joint", "l_ankle_joint", "l_foot_joint",
    "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint", "r_knee_joint", "r_ankle_joint", "r_foot_joint",
    "torso_joint",
]


class Proj500HumanoidEnv(DirectRLEnv):
    cfg: Proj500HumanoidCfg

    def __init__(self, cfg: Proj500HumanoidCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        # Robot handle
        self.robot: Articulation = self.scene["robot"]

        # Resolve joint indices in the articulation
        self.joint_ids = [self.robot.find_joints(name)[0] for name in JOINT_NAMES]
        self.joint_ids = torch.tensor(self.joint_ids, device=self.device, dtype=torch.long)

        # Action: delta joint target (radians), applied to current targets
        self.num_actions = len(JOINT_NAMES)

        # Obs: q, qd, projected gravity, base ang vel  =>  n + n + 3 + 3
        self.num_obs = self.num_actions * 2 + 6

        self.cfg.action_space = self.num_actions
        self.cfg.observation_space = self.num_obs

        # Buffers
        self._last_actions = torch.zeros((self.num_envs, self.num_actions), device=self.device)

        # A simple nominal pose (zero) – you can replace with a better stand pose later
        self._default_q = torch.zeros((self.num_envs, self.num_actions), device=self.device)
        self._default_qd = torch.zeros((self.num_envs, self.num_actions), device=self.device)

    # ---------------- RL API ----------------

    def _setup_scene(self):
        super()._setup_scene()

        # spawn robot
        robot = Articulation(self.cfg.robot)
        self.scene.add("robot", robot)

        # ground plane is created by the base scene in most IsaacLab templates
        # if you don’t have one, tell me and I’ll add explicit ground spawn

    def _reset_idx(self, env_ids: torch.Tensor):
        # reset robot state
        self.robot.reset(env_ids)

        # set base pose upright; IsaacLab typically uses default root state on reset
        # set joint state
        q = self._default_q[env_ids]
        qd = self._default_qd[env_ids]
        self.robot.set_joint_position_target(q, joint_ids=self.joint_ids, env_ids=env_ids)
        self.robot.set_joint_positions(q, joint_ids=self.joint_ids, env_ids=env_ids)
        self.robot.set_joint_velocities(qd, joint_ids=self.joint_ids, env_ids=env_ids)

        self._last_actions[env_ids] = 0.0

    def _pre_physics_step(self, actions: torch.Tensor):
        # actions in [-1, 1] -> delta radians
        actions = torch.clamp(actions, -1.0, 1.0)
        delta = 0.25 * actions  # start small (0.25 rad max step)

        # current targets -> new targets
        # NOTE: In IsaacLab versions, getter names can differ; if this errors, tell me the attribute error text.
        current_targets = self.robot.data.joint_pos_target[:, self.joint_ids]
        new_targets = current_targets + delta

        # Optional: clamp to joint limits (recommended)
        lower = self.robot.data.joint_pos_limits[:, self.joint_ids, 0]
        upper = self.robot.data.joint_pos_limits[:, self.joint_ids, 1]
        new_targets = torch.max(torch.min(new_targets, upper), lower)

        self.robot.set_joint_position_target(new_targets, joint_ids=self.joint_ids)

        self._last_actions = actions

    def _get_observations(self):
        # joint states
        q = self.robot.data.joint_pos[:, self.joint_ids]
        qd = self.robot.data.joint_vel[:, self.joint_ids]

        # base orientation -> projected gravity in base frame
        quat_w = self.robot.data.root_quat_w  # (N, 4)
        g_world = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(self.num_envs, 1)
        g_base = quat_rotate_inverse(quat_w, g_world)

        # base angular velocity (in base frame)
        ang_vel = self.robot.data.root_ang_vel_b  # (N, 3)

        obs = torch.cat([q, qd, g_base, ang_vel], dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        # “stand / don’t fall” reward first (you’ll flip to walking later)
        alive = torch.full((self.num_envs,), self.cfg.alive_reward, device=self.device)

        # torso tilt penalty from projected gravity: if upright, g_base ~ [0,0,-1]
        g_base = self._get_observations()["policy"][:, (self.num_actions*2):(self.num_actions*2 + 3)]
        tilt_pen = 2.0 * torch.sum(g_base[:, :2] ** 2, dim=1)

        # joint vel penalty
        qd = self.robot.data.joint_vel[:, self.joint_ids]
        vel_pen = 0.01 * torch.sum(qd ** 2, dim=1)

        # action rate penalty
        act_pen = 0.005 * torch.sum(self._last_actions ** 2, dim=1)

        # fall check
        base_h = self.robot.data.root_pos_w[:, 2]
        fallen = base_h < (0.5 * self.cfg.target_base_height_m)
        fall_cost = torch.where(fallen, torch.full_like(alive, -alive + self.cfg.fall_penalty), torch.zeros_like(alive))

        rew = alive - tilt_pen - vel_pen - act_pen + fall_cost
        return rew

    def _get_dones(self):
        base_h = self.robot.data.root_pos_w[:, 2]
        fallen = base_h < (0.5 * self.cfg.target_base_height_m)

        time_out = self.episode_length_buf >= int(self.cfg.episode_length_s / self.cfg.sim.dt)
        return fallen, time_out