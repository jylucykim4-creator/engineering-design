from verifier import verify_controller

def print_evidence(name, result):
    print(f"\n--- {name} ---")
    print(f"Overall verdict: {result['overall_verdict']}\n")

    for record in result["evidence"]:
        print(f"Requirement: {record['requirement']}")
        print(f"Verdict:     {record['verdict']}")
        print(f"Observed:    {record['observed']}")
        print(f"Expected:    {record['expected']}")
        print()

# Fixture 1: Invalid candidate
invalid_controller = verify_controller(
    Kp=float("nan"),
    Ki=10.0,
    Kd=5.0
)

print_evidence(
    "TEST 1 - Invalid PID gains",
    invalid_controller
)


# Fixture 2: Intentionally unstable controller
unstable_controller = verify_controller(
    Kp=-30.0,
    Ki=10.0,
    Kd=0.0
)

print_evidence(
    "TEST 2 - Unstable controller",
    unstable_controller
)


# Fixture 3: Stable controller for exercising the full pipeline
stable_controller = verify_controller(
    Kp=100.0,
    Ki=200.0,
    Kd=20.0
)

print_evidence(
    "TEST 3 - Stable controller",
    stable_controller
)
