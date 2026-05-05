from isaaclab_rl.rsl_rl import (
    RslRlMLPModelCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
)
from dataclasses import dataclass

from simulation.isaac.configuration.humanoid_stand_smooth_policy_contract import CONTRACT


@dataclass(frozen=True)
class SmoothStandingDeploymentCfg:
    use_obs_normalization: bool = CONTRACT.use_obs_normalization
    obs_normalizer_required: bool = CONTRACT.obs_normalizer_required
    obs_normalizer_artifact_name: str = CONTRACT.obs_normalizer_artifact_name


SMOOTH_STAND_DEPLOYMENT_CFG = SmoothStandingDeploymentCfg()


def get_humanoid_stand_smooth_ppo_cfg():
    return RslRlOnPolicyRunnerCfg(
        seed=42,
        device="cuda:0",
        num_steps_per_env=32,
        max_iterations=1200,
        empirical_normalization=CONTRACT.use_obs_normalization,
        save_interval=50,
        experiment_name="humanoid_stand_smooth_s2r",
        run_name="",
        logger="tensorboard",
        # Keep PPO from hiding saturated raw outputs. The env applies the
        # deployment safety clamp and physical joint limits.
        clip_actions=100.0,
        actor=RslRlMLPModelCfg(
            hidden_dims=[256, 256, 128],
            activation="elu",
            obs_normalization=CONTRACT.use_obs_normalization,
            distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
                init_std=0.3,
                std_type="scalar",
            ),
        ),
        critic=RslRlMLPModelCfg(
            hidden_dims=[256, 256, 128],
            activation="elu",
            obs_normalization=CONTRACT.use_obs_normalization,
            distribution_cfg=None,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.12,
            entropy_coef=0.0005,
            num_learning_epochs=4,
            num_mini_batches=8,
            learning_rate=5.0e-5,
            schedule="adaptive",
            gamma=0.995,
            lam=0.95,
            desired_kl=0.006,
            max_grad_norm=0.5,
        ),
    )
