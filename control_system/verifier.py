# Non-oracle verifier for ENGDESIGN control-system tasks
import math
import numpy as np
from scipy import signal

# Public task parameters from ENGDESIGN XG_13
MASS = 1.0          # kg
DAMPING = 10.0      # N*s/m
SPRING = 20.0       # N/m
FORCE = 1.0         # N

# Performance requirements from public prompts
MAX_SETTLING_TIME = 0.2       # seconds
MAX_OVERSHOOT = 5.0           # percent
TARGET_STEADY_STATE_ERROR = 0.0

def validate_pid_gains(Kp, Ki, Kd):
    """Check that the PID gains are valid finite numbers."""

    gains = {
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd
    }

    for name, value in gains.items():

        if not isinstance(value, (int, float)):
            return {
                "verdict": "fail",
                "reason": f"{name} is not a number."
            }

        if not math.isfinite(value):
            return {
                "verdict": "fail",
                "reason": f"{name} is not finite."
            }

    return {
        "verdict": "pass",
        "reason": "Kp, Ki, and Kd are valid finite numbers."
    }
    
def closed_loop_characteristic(Kp, Ki, Kd):
    """
    Build the closed-loop characteristic polynomial
    for the PID-controlled mass-spring-damper system.

    Returns coefficients in descending powers of s:
    a3*s^3 + a2*s^2 + a1*s + a0
    """

    return [
        MASS,
        DAMPING + Kd,
        SPRING + Kp,
        Ki
    ]

def check_stability(Kp, Ki, Kd):
    """Check closed-loop stability using analytical control criteria."""

    a3, a2, a1, a0 = closed_loop_characteristic(Kp, Ki, Kd)

    # If Ki = 0, the integrator cancels and the effective
    # closed-loop characteristic equation is second order.
    if Ki == 0:
        stable = (
            MASS > 0 and
            (DAMPING + Kd) > 0 and
            (SPRING + Kp) > 0
        )

        return {
            "requirement": "closed_loop_stability",
            "operation": "Second-order analytical stability check",
            "observed": {
                "coefficients": [
                    MASS,
                    DAMPING + Kd,
                    SPRING + Kp
                ]
            },
            "expected": "all second-order coefficients > 0",
            "verdict": "pass" if stable else "fail"
        }

    # Full PID / PI case: cubic Routh-Hurwitz criterion
    coefficients_positive = (
        a3 > 0 and
        a2 > 0 and
        a1 > 0 and
        a0 > 0
    )

    routh_condition = (a2 * a1) > (a3 * a0)

    stable = coefficients_positive and routh_condition

    return {
        "requirement": "closed_loop_stability",
        "operation": "Cubic Routh-Hurwitz stability check",
        "observed": {
            "coefficients": [a3, a2, a1, a0],
            "routh_left": a2 * a1,
            "routh_right": a3 * a0
        },
        "expected": "positive coefficients and a2*a1 > a3*a0",
        "verdict": "pass" if stable else "fail"
    }

def simulate_closed_loop(Kp, Ki, Kd):
    """
    Simulate the PID-controlled mass-spring-damper system.

    Modeling assumption:
    - unity feedback
    - unit-step reference input
    """

    # Closed-loop transfer function:
    #
    #              Kd*s^2 + Kp*s + Ki
    # T(s) = --------------------------------
    #        m*s^3 + (b+Kd)*s^2 + (k+Kp)*s + Ki

    numerator = [Kd, Kp, Ki]

    denominator = [
        MASS,
        DAMPING + Kd,
        SPRING + Kp,
        Ki
    ]

    system = signal.TransferFunction(numerator, denominator)

    # Simulate from 0 to 2 seconds
    time = np.linspace(0, 2.0, 10000)

    time, response = signal.step(system, T=time)

    return time, response

def measure_performance(time, response):
    """
    Measure settling time, percent overshoot,
    and steady-state error from the simulated response.
    """

    final_value = response[-1]

    # Steady-state error for a unit-step reference
    steady_state_error = abs(1.0 - final_value)

    # Percent overshoot
    peak_value = np.max(response)

    if abs(final_value) > 1e-12:
        overshoot = max(
            0.0,
            ((peak_value - final_value) / abs(final_value)) * 100.0
        )
    else:
        overshoot = float("inf")

    # Settling time using a 2% band around the final value
    tolerance = 0.02 * abs(final_value)

    settling_time = None

    for i in range(len(response)):
        remaining_response = response[i:]

        if np.all(np.abs(remaining_response - final_value) <= tolerance):
            settling_time = time[i]
            break

    return {
        "settling_time": settling_time,
        "overshoot": overshoot,
        "steady_state_error": steady_state_error,
        "final_value": final_value
    }
