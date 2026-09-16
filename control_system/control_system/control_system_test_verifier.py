from verifier import verify_controller

def print_evidence(name, evidence):
    print(f"\n--- {name} ---")

    for record in evidence:
        print(f"Requirement: {record['requirement']}")
        print(f"Verdict:     {record['verdict']}")
        print(f"Observed:    {record['observed']}")
        print(f"Expected:    {record['expected']}")
        print()
