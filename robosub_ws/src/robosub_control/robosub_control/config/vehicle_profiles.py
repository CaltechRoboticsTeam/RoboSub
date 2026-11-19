"""Select the vehicle profile."""

from robosub_control.config import constants, current_m5, future_t200

PROFILES = {profile.NAME: profile for profile in (current_m5, future_t200)}


def get_profile(name=constants.DEFAULT_VEHICLE_PROFILE):
    return PROFILES[name]


def profile_from_node(node):
    from robosub_control.utils.tuning import fixed_parameter

    return get_profile(
        fixed_parameter(
            node, "vehicle_profile", constants.DEFAULT_VEHICLE_PROFILE
        )
    )
