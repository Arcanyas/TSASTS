"""
TSA hardware parameter optimisation config.

Passed to TSAOptimEnv and train_ppo.py.  Serialisable to/from JSON via
dataclasses.asdict() + TSAOptimConfig(**d) pattern.

Parameter vector (symmetric mode, 5-D):
    θ = [L, t0, t1, t2, t3]
    L   : untwisted string length [m]       bounds [L_min, L_max]
    t_i : motor i activation time [s]       bounds [t_min, t_max]
    constraint: t0 ≤ t1 ≤ t2 ≤ t3 (enforced by sorting in wrapper)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class TSAOptimConfig:
    """Top-level config for TSA PPO optimisation."""

    total_timesteps: int = 5000
    """Total PPO timesteps = total episodes (one env step = one full STS rollout)."""

    # ------------------------------------------------------------------

    @dataclass
    class EnvParams:
        symmetric: bool = True
        """If True, share activation times across legs → 5-D action.
        If False, per-leg → 9-D action."""

        max_steps: int = 600
        """Maximum sim steps per episode (~6 s at dt=0.01 s)."""

        L_min: float = 0.25
        """Minimum string length [m]."""

        L_max: float = 0.65
        """Maximum string length [m]. Beyond 0.65 m X_max is irrelevant
        within the normal STS knee-angle range."""

        t_min: float = 0.0
        """Earliest motor activation time [s]."""

        t_max: float = 0.65
        """Latest motor activation time [s]. Capped well within the STS window
        (~0.78 s) so all motors fire before phase 4 entry and actually affect
        the motion. Increasing beyond ~0.6 s allows activations that miss the
        episode entirely and game the torque reward."""

        num_envs: int = 1
        """Parallel environments. >1 requires SubprocVecEnv with separate
        gym.make('TorsoLegs') per worker."""

    # ------------------------------------------------------------------

    @dataclass
    class RewardParams:
        # ── Component weights ───────────────────────────────────────────
        w_torque: float = 0.0
        """Weight for torque-tracking term."""

        w_muscle: float = 1.0 # Must be tuned
        """Weight for quadriceps-relief term.
        Start at 0; add once torque signal converges."""

        w_time: float = 0.0
        """Weight for STS-speed term.
        Episodes terminate when phase 4 is entered (STS complete), so t_final
        varies and this term has a gradient. Increase to reward faster STS."""

        # ── Torque tracking ─────────────────────────────────────────────
        support_fraction: float = 0.1
        """α — TSA target = α × knee demand.  0.3 means the exoskeleton
        should supply 30% of the total knee extension moment."""

        torque_ref: float = 20.0
        """Reference torque [N·m] for normalising the error term to ~[0, 1]."""

        # ── Quadriceps relief ───────────────────────────────────────────
        quad_muscle_names: List[str] = field(default_factory=lambda: [
            "vaslat_r", "vasmed_r", "vasint_r",
            "vaslat_l", "vasmed_l", "vasint_l",
            "recfem_r", "recfem_l",
        ])
        """MuJoCo actuator names for VL, VM, RF muscles.
        Verify against model.actuator(i).name list before first run."""

        # ── STS speed ───────────────────────────────────────────────────
        t_max_episode: float = 6.0
        """Episode duration cap [s] used to normalise the time penalty."""

        # ── Optional penalties (start at 0) ─────────────────────────────
        slack_penalty: float = 0.1
        """Penalise motor steps where cable is slack (X ≤ X_geometric)."""

        wall_penalty: float = 0.0
        """Penalise motors that hit X_max (max_contraction_ratio) early."""

    # ------------------------------------------------------------------

    @dataclass
    class PPOParams:
        learning_rate: float = 3e-4
        n_steps: int = 64
        """Episodes collected per env before each PPO gradient update.
        Must satisfy: n_steps * num_envs >= batch_size."""

        batch_size: int = 32
        n_epochs: int = 10
        gamma: float = 0.99
        """Discount factor.  Irrelevant for one-step MDP but SB3 requires it."""

        gae_lambda: float = 0.95
        ent_coef: float = 0.01
        """Entropy bonus coefficient — encourages exploration of θ-space."""

        clip_range: float = 0.2
        vf_coef: float = 0.5
        max_grad_norm: float = 0.5
        device: str = "cpu"

    # ------------------------------------------------------------------

    env_params: EnvParams = field(default_factory=EnvParams)
    reward_params: RewardParams = field(default_factory=RewardParams)
    ppo_params: PPOParams = field(default_factory=PPOParams)
