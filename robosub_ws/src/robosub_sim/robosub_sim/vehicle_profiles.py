"""Select a fixed vehicle profile without modifying process-global defaults."""

from robosub_sim import constants, current_m5, future_t200

PROFILES = {profile.NAME: profile for profile in (current_m5, future_t200)}


def get_profile(name=constants.DEFAULT_VEHICLE_PROFILE):
    """Resolve only explicitly supported vehicles."""
    if name not in PROFILES:
        raise ValueError(f"Unknown vehicle profile: {name}")
    return PROFILES[name]


def profile_from_node(node):
    from robosub_sim.tuning import fixed_parameter

    return get_profile(
        fixed_parameter(
            node, "vehicle_profile", constants.DEFAULT_VEHICLE_PROFILE
        )
    )
