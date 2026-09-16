# Non-oracle verifier for ENGDESIGN control-system tasks
import math

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
