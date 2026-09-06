import numpy as np
from typing import Callable, Dict
import matplotlib.pyplot as plt


class TSASimulator:
    """
    Single-unit Twisted String Actuator (TSA) simulator in task space.
    
    Models the kinematics and simplified dynamics of a TSA with motor torque input.
    Uses task-space formulation from Section 1.4 of the reference document.
    
    State variables:
        - theta: motor rotation angle (rad)
        - theta_dot: motor angular velocity (rad/s)
        - X: string contraction (m)
        - X_dot: string contraction velocity (m/s)
    
    Outputs:
        - Tension T in the string (N) - maps to task space force
        - Can be mapped to knee torque via moment arm in post-processing
    """
    
    def __init__(
        self,
        id: int,
        L: float,
        radius: float,
        I_motor: float = 1e-4,
        pretension_theta: float = 2 * np.pi,
        max_motor_torque: float = 0.1275,       # Polulu 9.7:1 LP 12V: 0.1275 Nm
        no_load_speed: float = 520.0,
        viscous_damping: float = 1e-4,
        required_tension: float = 120.0,
        max_contraction_ratio: float = 0.30,
    ):
        """
        Initialize TSA simulator.
        
        Parameters:
            L: Uncontracted string length (m)
            radius: String radius used in helical wrapping (m)
            I_motor: Motor moment of inertia (kg⋅m²) [placeholder: 1e-4]
            pretension_theta: Initial motor angle for pre-tensioning (rad) 
                            [default: 1 full rotation = 2π]
            max_motor_torque: Maximum allowable motor torque (Nm)
            no_load_speed: Approximate motor no-load speed (rad/s)
            viscous_damping: Motor-side viscous damping coefficient (N⋅m⋅s/rad)
            required_tension: Placeholder tension load used to compute motor demand (N)
            max_contraction_ratio: Maximum contraction as a fraction of L
        """
        self.id = id
        self.L = L
        self.radius = radius
        self.I = I_motor
        self.theta_pretension = pretension_theta
        self.max_motor_torque = max_motor_torque
        self.no_load_speed = no_load_speed
        self.viscous_damping = viscous_damping
        self.required_tension = required_tension
        self.max_contraction_ratio = max_contraction_ratio
        self.max_contraction = self.max_contraction_ratio * self.L
        
        # State (current snapshot)
        self.theta = pretension_theta
        self.theta_dot = 0.0
        self.X = 0.0
        self.X_dot = 0.0
        self.J = 1.0
        self.J_dot = 0.0

    def _clamp_contraction(self, X: np.ndarray) -> np.ndarray:
        """Clamp contraction to the configured physical limit."""
        return np.clip(X, 0.0, self.max_contraction)

    def _compute_required_motor_torque(self, J: np.ndarray) -> np.ndarray:
        """
        Required motor torque for the current contraction state.

        With the simplified task-space model, motor torque demand increases as
        the Jacobian grows. A placeholder tension load is used here so the
        actuator can stall when the available motor torque is insufficient.
        """
        return self.required_tension * J

    def _available_motor_torque(self, theta_dot: float) -> float:
        """Linear torque-speed curve with saturation at stall torque."""
        speed = abs(theta_dot)
        if self.no_load_speed <= 1e-10:
            return self.max_motor_torque
        torque = self.max_motor_torque * max(0.0, 1.0 - speed / self.no_load_speed)
        return torque

    def _compute_tension_from_torque(self, tau: np.ndarray, J: np.ndarray) -> np.ndarray:
        """Map motor torque to string tension using virtual work relation T = tau / J."""
        safe_J = np.where(np.abs(J) > 1e-10, J, np.nan)
        T = np.divide(tau, safe_J)
        return np.nan_to_num(T, nan=0.0, posinf=0.0, neginf=0.0)
    
    def _compute_contraction(self, theta: np.ndarray) -> np.ndarray:
        """
        Forward kinematics: map motor angle to string contraction.
        
        X(θ) = L - √(L² - (θr)²)  [Eq. from Section 1.4]
        """
        arg = self.L**2 - (theta * self.radius)**2
        # Guard against numerical issues at limits
        arg = np.clip(arg, 0, None)
        return self._clamp_contraction(self.L - np.sqrt(arg))
    
    def _compute_jacobian(self, X: np.ndarray) -> np.ndarray:
        """
        Compute task-space Jacobian dX/dθ (maps motor rotation to contraction).
        
        J(X) = r * √[(2L - X)X] / (L - X)  [Task space form, Section 1.4]
        
        Numerically stable via the form: J(θ) = θr² / √(L² - θ²r²)
        """
        denom = self.L - X
        denom = np.where(np.abs(denom) < 1e-10, 1e-10, denom)
        
        numer = self.radius * np.sqrt(np.clip((2 * self.L - X) * X, 0, None))
        return numer / denom
    
    def _compute_jacobian_dot(self, J: np.ndarray, X_dot: np.ndarray, X: np.ndarray) -> np.ndarray:
        """
        Compute time derivative of Jacobian.
        
        dJ/dt = [L²r² / (L - X)³] * J⁻¹ * dX/dt  [Section 1.4, Task space form]
        """
        denom = (self.L - X)**3
        denom = np.where(np.abs(denom) < 1e-10, 1e-10, denom)
        
        coeff = (self.L**2 * self.radius**2) / denom
        J_inv = np.where(np.abs(J) > 1e-10, 1.0 / J, 0.0)
        
        return coeff * J_inv * X_dot
    
    def simulate(
        self,
        tau_input: Callable[[float], float],
        t_end: float,
        dt: float = 0.001,
        verbose: bool = False
    ) -> Dict[str, np.ndarray]:
        """
        Simulate TSA response to motor torque input over time.
        
        Uses forward Euler integration for clarity. Motor dynamics include a
        linear torque-speed curve, viscous damping, and a task-space load torque.
        
        String tension is computed via virtual work principle:
            τ * dθ = T * dX  →  T = τ / J
        
        Parameters:
            tau_input: Callable τ(t) returning torque in Nm at time t
            t_end: End time for simulation (s)
            dt: Time step (s) [default: 1 ms]
            verbose: Print state summary during integration
        
        Returns:
            Dictionary with keys:
                - 't': Time array (s)
                - 'theta': Motor angle (rad)
                - 'theta_dot': Motor angular velocity (rad/s)
                - 'X': String contraction (m)
                - 'X_dot': Contraction velocity (m/s)
                - 'J': Jacobian (m/rad)
                - 'J_dot': Jacobian time derivative (m/s/rad)
                - 'tau': Input torque (Nm)
                - 'T': String tension (N)
                - 'tau_required': Required motor torque (Nm)
                - 'stalled': Boolean stall flag over time
        """
        n_steps = int(np.ceil(t_end / dt)) + 1
        t = np.linspace(0, t_end, n_steps)
        
        # Initialize state arrays
        theta = np.zeros(n_steps)
        theta_dot = np.zeros(n_steps)
        X = np.zeros(n_steps)
        X_dot = np.zeros(n_steps)
        J = np.zeros(n_steps)
        J_dot = np.zeros(n_steps)
        tau_log = np.zeros(n_steps)
        tau_available = np.zeros(n_steps)
        tau_load = np.zeros(n_steps)
        tau_required = np.zeros(n_steps)
        stalled = np.zeros(n_steps, dtype=bool)
        T = np.zeros(n_steps)
        
        # Initial conditions (at pretension)
        theta[0] = self.theta_pretension
        theta_dot[0] = 0.0
        X[0] = self._compute_contraction(np.array([theta[0]]))[0]
        J[0] = self._compute_jacobian(np.array([X[0]]))[0]
        tau_required[0] = self._compute_required_motor_torque(np.array([J[0]]))[0]
        tau_available[0] = self._available_motor_torque(theta_dot[0])
        tau_load[0] = tau_required[0]
        stalled[0] = tau_available[0] <= tau_load[0]
        T[0] = self._compute_tension_from_torque(
            np.array([min(tau_available[0], tau_load[0])]), np.array([J[0]])
        )[0]
        
        # Time integration loop
        for i in range(1, n_steps):
            t_now = t[i - 1]
            tau_now = tau_input(t_now)
            tau_log[i - 1] = tau_now

            tau_required[i - 1] = self._compute_required_motor_torque(np.array([J[i - 1]]))[0]
            tau_available[i - 1] = self._available_motor_torque(theta_dot[i - 1])
            tau_load[i - 1] = tau_required[i - 1]

            # Commanded torque is capped by what the motor can currently supply.
            tau_cmd = min(tau_now, tau_available[i - 1])

            # Motor dynamics with task-space load and viscous damping:
            # I * θ̈ = τ_cmd - τ_load - b * θ̇
            theta_ddot = (
                tau_cmd - tau_load[i - 1] - self.viscous_damping * theta_dot[i - 1]
            ) / self.I if self.I > 0 else 0.0
            
            # Euler step
            theta_dot[i] = theta_dot[i - 1] + theta_ddot * dt
            theta[i] = theta[i - 1] + theta_dot[i - 1] * dt + 0.5 * theta_ddot * dt**2

            # Prevent unwinding below the pretensioned state.
            theta[i] = max(theta[i], self.theta_pretension)
            
            # Update kinematics
            X[i] = self._compute_contraction(np.array([theta[i]]))[0]
            X[i] = min(X[i], self.max_contraction)
            J[i] = self._compute_jacobian(np.array([X[i]]))[0]
            X_dot[i] = J[i] * theta_dot[i]
            J_dot[i] = self._compute_jacobian_dot(
                np.array([J[i]]), np.array([X_dot[i]]), np.array([X[i]])
            )[0]

            tau_required[i] = self._compute_required_motor_torque(np.array([J[i]]))[0]
            tau_available[i] = self._available_motor_torque(theta_dot[i])
            tau_load[i] = tau_required[i]

            # Diagnostic flag only: this marks torque saturation, not a hard stop.
            stalled[i] = tau_available[i] <= tau_load[i]
            T[i] = self._compute_tension_from_torque(
                np.array([min(tau_available[i], tau_load[i])]), np.array([J[i]])
            )[0]

            if verbose and i % max(1, n_steps // 10) == 0:
                print(
                    f"t={t[i]:.3f}s | θ={theta[i]:.3f}rad | X={X[i]*1e3:.2f}mm | "
                    f"J={J[i]:.6f} | T={T[i]:.3f}N | τ_avail={tau_available[i]:.3f}Nm | "
                    f"τ_load={tau_load[i]:.3f}Nm | "
                    f"stalled={stalled[i]}"
                )
        
        tau_log[-1] = tau_input(t[-1])  # Final torque value
        tau_required[-1] = self._compute_required_motor_torque(np.array([J[-1]]))[0]
        tau_available[-1] = self._available_motor_torque(theta_dot[-1])
        tau_load[-1] = tau_required[-1]
        
        return {
            't': t,
            'theta': theta,
            'theta_dot': theta_dot,
            'X': X,
            'X_dot': X_dot,
            'J': J,
            'J_dot': J_dot,
            'tau': tau_log,
            'tau_available': tau_available,
            'tau_load': tau_load,
            'tau_required': tau_required,
            'stalled': stalled,
            'T': T,
        }

    def plot_results(self, results: Dict[str, np.ndarray], title_suffix: str = ""):
        """Plot TSA outputs as a function of motor angle."""
        theta_rad = results['theta']
        stalled_idx = np.where(results['stalled'])[0]

        fig, axes = plt.subplots(3, 2, figsize=(13, 12), sharex=True)
        fig.suptitle(f"TSA Simulation Results vs Motor Angle {title_suffix}", fontsize=14)

        axes[0, 0].plot(theta_rad, results['X'] * 1e3, linewidth=2)
        axes[0, 0].set_ylabel('Contraction (mm)')
        axes[0, 0].grid(True)

        axes[0, 1].plot(theta_rad, results['X_dot'] * 1e3, linewidth=2)
        axes[0, 1].set_ylabel('Contraction Velocity (mm/s)')
        axes[0, 1].grid(True)

        axes[1, 0].plot(theta_rad, results['theta_dot'], linewidth=2, color='tab:green')
        axes[1, 0].set_ylabel('Motor Angular Velocity (rad/s)')
        axes[1, 0].grid(True)

        axes[1, 1].plot(theta_rad, results['tau_required'], label='Required Torque', linewidth=2)
        if 'tau_available' in results:
            axes[1, 1].plot(theta_rad, results['tau_available'], label='Available Torque', linewidth=2)
        axes[1, 1].axhline(self.max_motor_torque, color='r', linestyle='--', label='Motor Max Torque')
        axes[1, 1].set_ylabel('Torque (Nm)')
        axes[1, 1].grid(True)
        axes[1, 1].legend()

        axes[2, 0].plot(theta_rad, results['T'], label='String Tension', linewidth=2)
        if stalled_idx.size > 0:
            axes[2, 0].scatter(theta_rad[stalled_idx], results['T'][stalled_idx], color='r', s=18, label='Stalled')
        axes[2, 0].set_ylabel('Tension (N)')
        axes[2, 0].grid(True)
        axes[2, 0].legend()

        axes[2, 1].axis('off')

        for axis in axes[2, :]:
            axis.set_xlabel('Motor Angle (rad)')

        plt.tight_layout()
        return fig


def test_constant_torque():
    """Test 1: Constant torque input."""
    print("\n" + "="*70)
    print("TEST 1: Constant Torque (0.05 Nm)")
    print("="*70)
    
    # TSA parameters (placeholder values)
    L = 0.5  # 50 cm uncontracted length
    radius = 0.004  # 4 mm string radius
    I_motor = 5e-5  # motor inertia (kg⋅m²)
    
    sim = TSASimulator(
        id=1,
        L=L,
        radius=radius,
        I_motor=I_motor,
        pretension_theta=2*np.pi,
        max_motor_torque=0.1275,
    )
    
    # Constant torque profile
    tau_const = lambda t: 0.1275
    
    results = sim.simulate(tau_const, t_end=3.0, dt=0.01, verbose=True)
    
    print(f"\nFinal state:")
    print(f"  Contraction: {results['X'][-1]*1e3:.2f} mm")
    print(f"  Max contraction allowed: {sim.max_contraction*1e3:.2f} mm")
    print(f"  Max required torque: {results['tau_required'].max():.3f} Nm")
    print(f"  Stalled: {bool(results['stalled'].any())}")
    print(f"  Final Jacobian: {results['J'][-1]:.6f} m/rad")
    sim.plot_results(results, "(Constant 0.05 Nm)")
    plt.show()


def test_ramp_torque():
    """Test 2: Ramping torque profile."""
    print("\n" + "="*70)
    print("TEST 2: Ramping Torque (0 → 0.1 Nm over 2s)")
    print("="*70)
    
    L = 0.5
    radius = 0.004
    I_motor = 5e-5
    
    sim = TSASimulator(
        id=1,
        L=L,
        radius=radius,
        I_motor=I_motor,
        pretension_theta=2*np.pi,
        max_motor_torque=0.1275,
    )
    
    # Ramp from 0 to 0.1 Nm over 2 seconds
    def tau_ramp(t):
        return 0.1275 * (t / 2.0) if t < 2.0 else 0.1275
    
    results = sim.simulate(tau_ramp, t_end=3.0, dt=0.01, verbose=True)
    
    print(f"\nFinal state:")
    print(f"  Contraction: {results['X'][-1]*1e3:.2f} mm")
    print(f"  Max required torque: {results['tau_required'].max():.3f} Nm")
    print(f"  Stalled: {bool(results['stalled'].any())}")
    sim.plot_results(results, "(Ramp 0.1275 Nm)")
    plt.show()


if __name__ == "__main__":
    test_constant_torque()
    # test_ramp_torque()