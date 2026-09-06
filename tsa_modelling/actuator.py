"""
TSAActuator: High-level wrapper for TSASimulator to apply knee torque.

Physics of the tension interface
---------------------------------
String tension T is not freely commandable — it is a *result* of the
motor torque acting against an external load. From the payload equation
(eq. 1.32 bottom row):

    T = m*Xddot + F_ext + b_X*Xdot

For T to be non-trivial, at least one of three things must be true:
  (a) The payload is accelerating (m*Xddot != 0)
  (b) There is a static external force opposing contraction (F_ext != 0)
  (c) There is viscous damping on the payload side (b_X*Xdot != 0)

For an exoskeleton knee actuator the cable is resisted by the knee joint
torque. The correct model is to set F_ext = joint_resistance_force, which
is the force the joint exerts back on the cable opposing contraction:

    F_ext = tau_knee_resistance / moment_arm            [N]

In practice the controller commands a desired tension T_des which
represents the assistive force it wants to inject. The motor must produce
enough torque to overcome F_ext AND accelerate the cable. At steady state
(Xddot -> 0) the achieved tension equals F_ext, not T_des, unless a
closed-loop tension controller is used.

Two ways to set up the exo simulator
--------------------------------------
Option A (recommended) — explicit external force per step:
    Set payload_mass to the effective cable-side inertia (shin/foot segment
    inertia / moment_arm^2), gravity_along_string=False, and pass
    joint_resistance_force to step() each call. This lets the resistance
    force vary with joint angle or gait phase.

Option B — equivalent static payload mass:
    Lump the expected average joint resistance into an equivalent mass
    m_eff = F_avg / g and use gravity_along_string=True. Simpler but
    cannot vary the load during the motion.

This file implements Option A.
"""

import numpy as np
from typing import Dict, Optional
from model_v2 import TSASimulator


class TSAActuator:
    """
    Single TSA unit acting as an external knee actuator.

    Command interface : desired_tension [N] and joint_resistance_force [N]
    Output           : knee_torque [N·m] = achieved_tension x moment_arm

    The motor tries to produce tau_cmd = T_des * J.
    Tension is actually achieved against the joint resistance force F_ext,
    which must be supplied by the controller each step (it varies with
    joint angle and gait phase).

    Parameters
    ----------
    tsa : TSASimulator
        Configured TSASimulator (gravity_along_string=False for exo use).
        payload_mass should be the cable-side inertia = I_leg / moment_arm^2.
    side : str
        'right' or 'left'.
    moment_arm : float
        Fixed moment arm d [m].  knee_torque = T * d.
    name : str, optional
        Label for logging.
    """

    def __init__(
        self,
        tsa: TSASimulator,
        side: str = 'right',
        moment_arm: float = 0.05,
        name: Optional[str] = None,
    ):
        self.tsa  = tsa
        self.side = side
        self.name = name or f"TSAActuator_{side}"
        self._d   = moment_arm

        # Integration state
        self._theta     = tsa.theta_pretension
        self._theta_dot = 0.0

        # Derived kinematic state
        self._X     = tsa._contraction(self._theta)
        self._J     = tsa._jacobian(self._X)
        self._X_dot = 0.0
        self._J_dot = 0.0

        self.last_tension     = 0.0
        self.last_knee_torque = 0.0
        self.torque_saturated = False
        self._was_at_wall     = False
        self._wall_tension    = 0.0

    # ------------------------------------------------------------------
    # Main step interface
    # ------------------------------------------------------------------

    def step(
        self,
        t: float,
        dt: float,
        desired_tension: float,
        joint_resistance_force: float = 0.0,
    ) -> Dict[str, float]:
        """
        Advance TSA by one timestep.

        Parameters
        ----------
        t : float
            Current sim time [s].
        dt : float
            Timestep [s].
        desired_tension : float
            Target string tension [N]. Converted to tau_cmd = T_des * J.
        joint_resistance_force : float
            External force [N] the joint exerts back on the cable opposing
            contraction. Equivalent to tau_knee_ext / moment_arm.
            This is what creates non-zero steady-state tension.
            Pass 0 if running a free-spinning bench test with no load.

        Returns
        -------
        dict
            torque                 – knee torque = T_dynamic * d  [N·m]
            tension                – dynamic string tension T  [N]
            tension_cmd            – tension implied by clamped tau_cmd  [N]
            tau_cmd                – motor torque commanded  [N·m]
            tau_available          – motor torque available at current speed  [N·m]
            X                     – string contraction  [m]
            X_dot                 – contraction velocity  [m/s]
            theta                 – motor angle  [rad]
            theta_dot             – motor angular velocity  [rad/s]
            J                     – task-space Jacobian  [m/rad]
            torque_saturated      – True if tau_cmd was clamped
        """
        # 1. Desired torque: tau = T_des * J  (virtual work)
        tau_desired = desired_tension * self._J

        # 2. Clamp to motor capability
        tau_avail = self.tsa._tau_available(self._theta_dot)
        tau_cmd   = float(np.clip(tau_desired, 0.0, tau_avail))
        self.torque_saturated = (tau_cmd < tau_desired)

        tension_cmd = tau_cmd / self._J if self._J > 1e-12 else 0.0

        # 3. RK4 — self.tsa.m is the actual cable-side inertia (set by
        #    tsa_integration.py from MuJoCo's M diagonal each step).
        #    F_ext is passed directly so it does not corrupt the inertia term.
        self._theta, self._theta_dot = self._rk4_step(
            self._theta, self._theta_dot, tau_cmd, dt,
            F_ext=joint_resistance_force,
        )

        # 4. Synchronise kinematic state
        self._X     = self.tsa._contraction(self._theta)
        self._J     = self.tsa._jacobian(self._X)
        self._X_dot = self._J * self._theta_dot
        self._J_dot = self.tsa._jacobian_dot(self._J, self._X, self._X_dot)

        # 5. Tension at new state — three cases:
        #
        #    (a) At hard wall (theta_dot clamped to 0 by RK4):
        #        String is geometrically wound; tension is held constant at the
        #        motor's stall-tension ceiling (τ_stall / J_max) as long as any
        #        demand exists.  Drops to 0 only when tau_desired = 0 outright.
        #        Real TSA: reducing motor command does not unwind the string —
        #        tension stays until active reverse drive or demand removed.
        #
        #    (b) Motor stalled before the wall (tau_cmd ≤ J·F_resist):
        #        T = tau_cmd / J  (motor holding the load).
        #
        #    (c) Motor advancing: EOM gives dynamic tension
        #        T = m·Ẍ + F_resist + b_X·Ẋ.
        at_wall     = self._X >= self.tsa.max_contraction - 1e-9
        tau_to_hold = self._J * joint_resistance_force

        # Load-stall: motor torque insufficient to advance against F_resist.
        # Distinct from torque_saturated (speed-limited): this fires when the
        # load itself exceeds what the motor can push against at any speed.
        load_stalled = (tau_avail <= tau_to_hold)

        if at_wall:
            # Motor is geometrically stalled at max contraction: RK4 clamps
            # theta_dot to 0, so tau_avail = full stall torque.
            # Tension = tau_stall / J — the motor's torque balance at standstill.
            # This avoids the pathological latch-at-zero case where the motor
            # reached the wall while the cable was slack (tiny F_resist → near-zero
            # tension during free-winding → wall_tension latched as 0 → zero torque
            # even when the cable is taut at the wall for the rest of the motion).
            tau_stall = self.tsa._tau_available(0.0)  # theta_dot = 0 at wall
            T_dynamic = (tau_stall / self._J if self._J > 1e-12 else 0.0) if tau_desired > 1e-12 else 0.0
            self._wall_tension = T_dynamic
        elif tau_cmd <= tau_to_hold + 1e-9:
            T_dynamic = tau_cmd / self._J if self._J > 1e-12 else 0.0
        else:
            tau_avail_new = self.tsa._tau_available(self._theta_dot)
            tau_eval      = float(np.clip(tau_cmd, 0.0, tau_avail_new))
            _, T_dynamic, _, _, _, _, _ = self.tsa._eom(
                self._theta, self._theta_dot, tau_eval,
                F_ext_override=joint_resistance_force,
            )
        T_dynamic = max(T_dynamic, 0.0)

        # 6. Knee torque
        knee_torque = T_dynamic * self.get_moment_arm()
        self.last_tension     = T_dynamic
        self.last_knee_torque = knee_torque
        self._was_at_wall     = at_wall

        return {
            'torque':           knee_torque,
            'tension':          T_dynamic,
            'tension_cmd':      tension_cmd,
            'tau_cmd':          tau_cmd,
            'tau_available':    tau_avail,
            'tau_to_hold':      tau_to_hold,
            'X':                self._X,
            'X_dot':            self._X_dot,
            'theta':            self._theta,
            'theta_dot':        self._theta_dot,
            'J':                self._J,
            'at_wall':          at_wall,
            'torque_saturated': self.torque_saturated,
            'load_stalled':     load_stalled,
        }

    def reset(self):
        """Reset to pretension initial state."""
        self._theta     = self.tsa.theta_pretension
        self._theta_dot = 0.0
        self._X         = self.tsa._contraction(self._theta)
        self._J         = self.tsa._jacobian(self._X)
        self._X_dot     = 0.0
        self._J_dot     = 0.0
        self.last_tension     = 0.0
        self.last_knee_torque = 0.0
        self.torque_saturated = False
        self._was_at_wall     = False
        self._wall_tension    = 0.0

    # ------------------------------------------------------------------
    # Moment arm
    # ------------------------------------------------------------------

    def get_moment_arm(
        self,
        X: Optional[float] = None,
        theta: Optional[float] = None,
    ) -> float:
        """Return moment arm [m]. Override for geometry-dependent d."""
        return self._d

    def set_moment_arm(self, d: float):
        self._d = d

    # ------------------------------------------------------------------
    # State properties
    # ------------------------------------------------------------------

    @property
    def theta(self)     -> float: return self._theta
    @property
    def theta_dot(self) -> float: return self._theta_dot
    @property
    def X(self)         -> float: return self._X
    @property
    def X_dot(self)     -> float: return self._X_dot
    @property
    def J(self)         -> float: return self._J

    # ------------------------------------------------------------------
    # RK4 integrator
    # ------------------------------------------------------------------

    def _rk4_step(
        self,
        theta: float,
        theta_dot: float,
        tau_cmd: float,
        dt: float,
        F_ext: float = 0.0,
    ) -> tuple[float, float]:
        pretension = self.tsa.theta_pretension

        max_contraction = self.tsa.max_contraction

        def derivs(th: float, thd: float) -> tuple[float, float]:
            th  = max(th,  pretension)
            thd = max(thd, 0.0)
            # Hard stop: motor cannot wind further once cable hits contraction limit
            if self.tsa._contraction(th) >= max_contraction:
                return 0.0, 0.0
            theta_ddot, _, _, _, _, _, _ = self.tsa._eom(th, thd, tau_cmd, F_ext_override=F_ext)
            return thd, theta_ddot

        k1_th, k1_thd = derivs(theta,                   theta_dot)
        k2_th, k2_thd = derivs(theta + 0.5*dt*k1_th,   theta_dot + 0.5*dt*k1_thd)
        k3_th, k3_thd = derivs(theta + 0.5*dt*k2_th,   theta_dot + 0.5*dt*k2_thd)
        k4_th, k4_thd = derivs(theta +     dt*k3_th,   theta_dot +     dt*k3_thd)

        theta_new     = theta     + (dt/6)*(k1_th  + 2*k2_th  + 2*k3_th  + k4_th)
        theta_dot_new = theta_dot + (dt/6)*(k1_thd + 2*k2_thd + 2*k3_thd + k4_thd)

        theta_new     = max(theta_new,     pretension)
        theta_dot_new = max(theta_dot_new, 0.0)

        # Clamp speed to zero at the contraction wall
        if self.tsa._contraction(theta_new) >= max_contraction:
            theta_dot_new = 0.0

        return theta_new, theta_dot_new


# ============================================================================
# Tests
# ============================================================================

def _make_exo_sim() -> TSASimulator:
    """
    Exo TSASimulator.

    payload_mass = I_leg / d^2.
    Shin+foot inertia ~0.25 kg.m2, d=0.05m => m_eff = 0.25/0.0025 = 100 kg.
    This is passed as the inertial term in D_theta. F_ext is supplied per-step
    via joint_resistance_force in step().

    gravity_along_string=False: F_ext is handled by step(), not baked in here.
    """
    return TSASimulator(
        id=0,
        L=0.5,
        radius=0.004,
        payload_mass=100.0,          # I_leg / d^2 = 0.25 / 0.05^2
        I_motor=5e-5,
        pretension_theta=2 * np.pi,
        max_motor_torque=0.1275,
        no_load_speed=520.0,
        b_theta=1e-4,
        b_X=0.0,
        gravity_along_string=False,  # F_ext supplied dynamically per step
        max_contraction_ratio=0.30,
    )


def test_exo_constant_resistance():
    """
    Constant 300 N joint resistance (= 15 Nm knee torque at d=0.05m).
    Motor should accelerate, contraction and tension should grow,
    J should increase over time.
    """
    print("\n" + "="*70)
    print("TEST: Exo — constant 300 N joint resistance, T_des=350 N")
    print("="*70)

    d = 0.05
    actuator = TSAActuator(_make_exo_sim(), side='right', moment_arm=d)

    F_joint = 300.0      # N — opposing contraction (= 15 Nm knee torque)
    T_des   = 350.0      # N — slightly above resistance to drive contraction

    dt, t_end = 0.001, 2.0
    n = int(t_end / dt)
    log = {k: [] for k in [
        't', 'tension', 'torque', 'X', 'theta_rot', 'theta_dot', 'J', 'sat'
    ]}

    for i in range(n):
        r = actuator.step(i*dt, dt, T_des, joint_resistance_force=F_joint)
        log['t'].append(i * dt)
        log['tension'].append(r['tension'])
        log['torque'].append(r['torque'])
        log['X'].append(r['X'] * 1e3)
        log['theta_rot'].append(r['theta'] / (2*np.pi))
        log['theta_dot'].append(r['theta_dot'])
        log['J'].append(r['J'] * 1e3)
        log['sat'].append(r['torque_saturated'])

    for k in log:
        log[k] = np.array(log[k])

    print(f"  F_joint = {F_joint} N  => target knee torque = {F_joint*d:.1f} Nm")
    print(f"  T_des   = {T_des} N  (commanding net pull above resistance)")
    print(f"  Final tension:       {log['tension'][-1]:.2f} N")
    print(f"  Final knee torque:   {log['torque'][-1]:.2f} N·m")
    print(f"  Final contraction:   {log['X'][-1]:.2f} mm")
    print(f"  Final motor angle:   {log['theta_rot'][-1]:.2f} rot")
    print(f"  Final motor speed:   {log['theta_dot'][-1]:.1f} rad/s")
    print(f"  J grew:              {log['J'][0]:.4f} -> {log['J'][-1]:.4f} mm/rad")
    print(f"  Torque saturated:    {log['sat'].any()}")

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle(
        f"TSAActuator Exo Test — F_joint={F_joint} N, T_des={T_des} N, d={d} m",
        fontsize=12, fontweight='bold'
    )

    axes[0,0].plot(log['t'], log['tension'], lw=1.8)
    axes[0,0].axhline(F_joint, color='r', ls='--', lw=1, label=f'F_joint={F_joint}N')
    axes[0,0].axhline(T_des,   color='g', ls='--', lw=1, label=f'T_des={T_des}N')
    axes[0,0].set_ylabel('Tension (N)'); axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_title('String Tension'); axes[0,0].legend(fontsize=8)
    axes[0,0].grid(True, alpha=0.4)

    axes[0,1].plot(log['t'], log['torque'], lw=1.8, color='darkorange')
    axes[0,1].axhline(F_joint*d, color='r', ls='--', lw=1, label=f'Target {F_joint*d:.0f} Nm')
    axes[0,1].set_ylabel('Knee torque (N·m)'); axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_title('Knee Torque'); axes[0,1].legend(fontsize=8)
    axes[0,1].grid(True, alpha=0.4)

    axes[0,2].plot(log['t'], log['X'], lw=1.8, color='steelblue')
    axes[0,2].set_ylabel('Contraction (mm)'); axes[0,2].set_xlabel('Time (s)')
    axes[0,2].set_title('Contraction'); axes[0,2].grid(True, alpha=0.4)

    axes[1,0].plot(log['t'], log['theta_dot'], lw=1.8, color='green')
    axes[1,0].set_ylabel('Motor speed (rad/s)'); axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_title('Motor Speed'); axes[1,0].grid(True, alpha=0.4)

    axes[1,1].plot(log['t'], log['J'], lw=1.8, color='purple')
    axes[1,1].set_ylabel('J (mm/rad)'); axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_title('Jacobian'); axes[1,1].grid(True, alpha=0.4)

    axes[1,2].plot(log['t'], log['sat'].astype(float), lw=1.8, color='crimson')
    axes[1,2].set_ylabel('Saturated'); axes[1,2].set_xlabel('Time (s)')
    axes[1,2].set_title('Torque Saturation'); axes[1,2].set_ylim(-0.1, 1.1)
    axes[1,2].grid(True, alpha=0.4)

    plt.tight_layout()
    fig.savefig("images/actuator_exo_test.png", dpi=150)
    plt.show()
    print("  Plot saved.")


def test_reset():
    print("\n" + "="*70)
    print("TEST: Reset")
    print("="*70)
    actuator = TSAActuator(_make_exo_sim(), moment_arm=0.05)
    theta_0, X_0 = actuator.theta, actuator.X
    for i in range(500):
        actuator.step(i*0.001, 0.001, 300.0, joint_resistance_force=200.0)
    actuator.reset()
    assert abs(actuator.theta - theta_0) < 1e-10
    assert abs(actuator.X     - X_0)     < 1e-10
    print(f"  theta OK: {actuator.theta:.6f} rad")
    print(f"  X OK:     {actuator.X*1e3:.6f} mm")


if __name__ == "__main__":
    test_exo_constant_resistance()
    test_reset()