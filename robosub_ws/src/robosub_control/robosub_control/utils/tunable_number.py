"""Cached numeric settings, independent of ROS and network operations."""

import math


class TunableNumber:
    """A checked number whose reads only return a local cached value.

    The ROS adapter commits accepted changes and updates bound attributes.
    """

    def __init__(self, default, *, minimum=None, maximum=None):
        if type(default) not in (int, float):
            raise ValueError("A tunable default must be an int or float")
        self._type = type(default)
        self.minimum = minimum
        self.maximum = maximum
        if minimum is not None and maximum is not None and minimum > maximum:
            raise ValueError("minimum must not exceed maximum")
        self.default = self.validate(default)
        self._value = self.default
        self._bindings = []

    def validate(self, value):
        """Return a valid value without changing the cached setting."""
        if type(value) is not self._type or not math.isfinite(value):
            raise ValueError(f"Expected a finite {self._type.__name__}")
        if self.minimum is not None and value < self.minimum:
            raise ValueError(f"Value must be at least {self.minimum}")
        if self.maximum is not None and value > self.maximum:
            raise ValueError(f"Value must be at most {self.maximum}")
        return value

    def get(self):
        """Read the cache without checking parameters or the network."""
        return self._value

    def bind_to(self, target, attribute):
        """Keep an existing object's numeric attribute up to date."""
        if not hasattr(target, attribute):
            raise AttributeError(attribute)
        setattr(target, attribute, self._value)
        self._bindings.append((target, attribute))

    def _commit(self, value):
        self._value = self.validate(value)
        for target, attribute in self._bindings:
            setattr(target, attribute, self._value)
