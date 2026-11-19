"""Apply and log ROS parameter changes."""

import sys

from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node

from robosub_control.utils.tunable_number import TunableNumber


def fixed_parameter(node, name, default):
    """Declare a parameter that cannot change after startup."""
    return node.declare_parameter(
        name, default, ParameterDescriptor(read_only=True)
    ).value


class TunableNode(Node):
    """Sync accepted parameter changes before returning to the caller."""

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.tuning = TunableNumbers(self)

    def set_parameters(self, parameter_list):
        try:
            return super().set_parameters(parameter_list)
        finally:
            # Non-atomic sets may commit earlier values before a later error.
            tuning = getattr(self, "tuning", None)
            if tuning is not None:
                tuning.sync()

    def set_parameters_atomically(self, parameter_list):
        result = super().set_parameters_atomically(parameter_list)
        tuning = getattr(self, "tuning", None)
        if result.successful and tuning is not None:
            tuning.sync()
        return result


class TunableNumbers:
    """Apply numeric parameter changes to bound attributes."""

    def __init__(self, node):
        self.node = node
        self.numbers = {}
        node.add_on_set_parameters_callback(self._validate)

    def number(
        self, name, default, *, minimum=None, maximum=None, description=""
    ):
        """Register a typed number, bounds, and parameter help text."""
        number = TunableNumber(default, minimum=minimum, maximum=maximum)
        descriptor = ParameterDescriptor(
            description=description,
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
        parameter = self.node.declare_parameter(name, default, descriptor)
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

    def sync(self):
        for name, number in self.numbers.items():
            value = self.node.get_parameter(name).value
            previous = number.get()
            if previous != value:
                number._commit(value)
                self.node.get_logger().info(
                    f"Tuning {name}: {previous} -> {value}"
                )
