
"""
Hazard manager for risk-aware autonomous navigation.

Maintains hazard severity for individual SUMO road edges.

Severity scale:
    0 = No hazard
    1 = Low risk
    2 = Medium risk
    3 = High risk

A closed road should still be handled separately by obstacle_manager.py.
A high-risk road remains traversable but receives an additional cost.
"""

# ============================================================
# 1. HAZARD CONFIGURATION
# ============================================================

MAX_HAZARD_SEVERITY = 3

# Maximum additional cost for a severity-3 hazard.
# Cost is expressed in equivalent seconds for route comparison.
HAZARD_WEIGHT = 60.0


# ============================================================
# 2. HAZARD STORAGE
# ============================================================

# edge_id -> {
#     "severity": integer,
#     "description": string
# }

_hazards = {}


# ============================================================
# 3. SET HAZARD
# ============================================================

def set_hazard(edge_id, severity, description=""):
    """
    Assign a hazard severity to a SUMO road edge.

    Parameters
    ----------
    edge_id : str
        SUMO road edge ID.

    severity : int
        Integer between 0 and 3.

    description : str
        Optional explanation of the hazard.
    """

    if not isinstance(edge_id, str) or not edge_id.strip():
        raise ValueError("edge_id must be a non-empty string.")

    if isinstance(severity, bool) or not isinstance(severity, int):
        raise TypeError("Hazard severity must be an integer.")

    if not 0 <= severity <= MAX_HAZARD_SEVERITY:
        raise ValueError(
            f"Hazard severity must be between 0 and "
            f"{MAX_HAZARD_SEVERITY}."
        )

    if severity == 0:
        _hazards.pop(edge_id, None)
        print(f"[HAZARD] Cleared hazard on {edge_id}")
        return

    _hazards[edge_id] = {
        "severity": severity,
        "description": description
    }

    print(
        f"[HAZARD] {edge_id} | "
        f"Severity: {severity}/3 | "
        f"Description: {description}"
    )


# ============================================================
# 4. CLEAR HAZARD
# ============================================================

def clear_hazard(edge_id):
    """Remove the hazard assigned to an edge."""

    if edge_id in _hazards:
        del _hazards[edge_id]
        print(f"[HAZARD] Cleared hazard on {edge_id}")


def clear_all_hazards():
    """Remove all active hazards."""

    _hazards.clear()
    print("[HAZARD] All hazards cleared.")


# ============================================================
# 5. GET HAZARD SEVERITY
# ============================================================

def get_hazard_severity(edge_id):
    """Return the current severity of an edge; zero means no hazard."""

    hazard = _hazards.get(edge_id)

    if hazard is None:
        return 0

    return hazard["severity"]


# ============================================================
# 6. CALCULATE HAZARD COST
# ============================================================

def get_hazard_cost(edge_id):
    """
    Convert hazard severity into an additional route cost.

    Formula:

        H(e) = HAZARD_WEIGHT * severity / MAX_HAZARD_SEVERITY

    With HAZARD_WEIGHT=60.0 and MAX_HAZARD_SEVERITY=3:
        Severity 0 -> 0 cost units
        Severity 1 -> 20 cost units
        Severity 2 -> 40 cost units
        Severity 3 -> 60 cost units

    The cost is an additive planner penalty, expressed in the same
    effective units as the selected edge-cost function. For the current
    traffic-time model these units are treated as equivalent seconds.
    """

    severity = get_hazard_severity(edge_id)

    return (
        HAZARD_WEIGHT
        * severity
        / MAX_HAZARD_SEVERITY
    )


# ============================================================
# 7. GET ACTIVE HAZARDS
# ============================================================

def get_active_hazards():
    """Return a copy of all currently active hazards."""

    return {
        edge_id: details.copy()
        for edge_id, details in _hazards.items()
    }


# ============================================================
# 8. STANDALONE TEST
# ============================================================

if __name__ == "__main__":

    print("\n========== HAZARD MANAGER TEST ==========\n")

    set_hazard(
        "road_A",
        1,
        "Minor road obstruction"
    )

    set_hazard(
        "road_B",
        2,
        "Moderate traffic incident"
    )

    set_hazard(
        "road_C",
        3,
        "Major road hazard"
    )

    for edge_id in ["road_A", "road_B", "road_C"]:

        print(
            f"{edge_id} | "
            f"Severity: {get_hazard_severity(edge_id)} | "
            f"Hazard cost: {get_hazard_cost(edge_id):.2f}"
        )

    print("\nActive hazards:")
    print(get_active_hazards())

    assert abs(get_hazard_cost("road_A") - 20.0) < 1e-9
    assert abs(get_hazard_cost("road_B") - 40.0) < 1e-9
    assert abs(get_hazard_cost("road_C") - 60.0) < 1e-9

    clear_hazard("road_B")
    assert get_hazard_severity("road_B") == 0
    assert get_hazard_cost("road_B") == 0.0

    print("\nPASS: Hazard severity and cost calculations work.")
