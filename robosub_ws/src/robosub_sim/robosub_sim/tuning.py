"""Validated ROS parameters with cached values and committed-change logging."""

import json
import sys

from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    ParameterEvent,
    SetParametersResult,
)
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robosub_sim import constants
from robosub_sim.tunable_number import TunableNumber


def fixed_parameter(node, name, default):
    """Declare a startup-only setting; runtime changes would not take effect."""
    return node.declare_parameter(
        name, default, ParameterDescriptor(read_only=True)
    ).value


class TunableNumbers:
    """One tuning registry per node, with no per-control-tick network work.

    Validation callbacks have no side effects. Local caches change only when
    ROS publishes the event confirming that a parameter update committed.
    """

    def __init__(self, node):
        self.node = node
        self.numbers = {}
        requested = node.declare_parameter(
            "tuning_enabled",
            False,
            ParameterDescriptor(read_only=True),
            ignore_override=constants.LOCK_TUNING_DEFAULTS,
        ).value
        self.enabled = requested and not constants.LOCK_TUNING_DEFAULTS
        if self.enabled:
            node.add_on_set_parameters_callback(self._validate)
            qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.subscription = node.create_subscription(
                ParameterEvent, "/parameter_events", self._on_event, qos
            )

    def number(
        self, name, default, *, minimum=None, maximum=None, description=""
    ):
        """Register a typed number, bounds, and optional browser help text."""
        if name in self.numbers:
            raise ValueError(f"Duplicate tunable: {name}")
        number = TunableNumber(default, minimum=minimum, maximum=maximum)
        descriptor = ParameterDescriptor(
            description=description,
            read_only=not self.enabled,
            additional_constraints=json.dumps(
                {"robosub_tunable": True, "default": default}
            ),
        )
        if type(default) is float:
            descriptor.floating_point_range = [
                FloatingPointRange(
                    from_value=(
                        -sys.float_info.max
                        if minimum is None
                        else float(minimum)
                    ),
                    to_value=(
                        sys.float_info.max
                        if maximum is None
                        else float(maximum)
                    ),
                )
            ]
        else:
            descriptor.integer_range = [
                IntegerRange(
                    from_value=-(2**63) if minimum is None else minimum,
                    to_value=2**63 - 1 if maximum is None else maximum,
                )
            ]
        parameter = self.node.declare_parameter(
            name, default, descriptor, ignore_override=not self.enabled
        )
        number._commit(parameter.value)
        self.numbers[name] = number
        return number

    def _validate(self, parameters):
        try:
            for parameter in parameters:
                number = self.numbers.get(parameter.name)
                if number is not None:
                    number.validate(parameter.value)
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _on_event(self, event):
        if event.node != self.node.get_fully_qualified_name():
            return
        for message in [*event.new_parameters, *event.changed_parameters]:
            number = self.numbers.get(message.name)
            if number is None:
                continue
            value = Parameter.from_parameter_msg(message).value
            previous = number.get()
            number._commit(value)
            if previous != value:
                self.node.get_logger().info(
                    f"Tuning {message.name}: {previous} -> {value}"
                )
