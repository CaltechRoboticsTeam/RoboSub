"""Operator-specified zero-based motor order shared by both vehicles."""

THRUSTER_NAMES = ("FR", "FL", "BR", "BL", "RD", "LD")
THRUSTER_INDICES = {name: index for index, name in enumerate(THRUSTER_NAMES)}
THRUSTER_TOPIC_IDS = {
    name: f"t{index}" for name, index in THRUSTER_INDICES.items()
}
THRUSTERS = tuple(THRUSTER_TOPIC_IDS.values())
