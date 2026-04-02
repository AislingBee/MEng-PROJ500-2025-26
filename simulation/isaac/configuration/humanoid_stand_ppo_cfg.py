from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
    RslRlMLPModelCfg,
)


def get_humanoid_stand_ppo_cfg():
    return RslRlOnPolicyRunnerCfg(
        seed=42,
        device="cuda:0",
        num_steps_per_env=8,
        max_iterations=1000,
        empirical_normalization=False,
        save_interval=50,
        experiment_name="humanoid_stand",
        run_name="",
        logger="tensorboard",
        clip_actions=1.0,
        actor=RslRlMLPModelCfg(
            hidden_dims=[256, 256, 128],
            activation="elu",
            obs_normalization=False,
            distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
                init_std=1.0,
                std_type="scalar",
            ),
        ),
        critic=RslRlMLPModelCfg(
            hidden_dims=[256, 256, 128],
            activation="elu",
            obs_normalization=False,
            distribution_cfg=None,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.01,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=1.0e-3,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
    )