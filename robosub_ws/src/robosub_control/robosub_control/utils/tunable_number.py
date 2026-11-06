"""Cache numeric settings."""

import math


class TunableNumber:
    """A numeric setting with limits and bound object attributes."""

    def __init__(self, default, *, minimum=None, maximum=None):
        self._type = type(default)
        self.minimum = minimum
        self.maximum = maximum
        self.default = default
        self._value = self.default
        self._bindings = []

    def validate(self, value):
        """Check a new value before applying it."""
        if type(value) is not self._type or not math.isfinite(value):
            raise ValueError(f"Expected a finite {self._type.__name__}")
        if self.minimum is not None and value < self.minimum:
            raise ValueError(f"Value must be at least {self.minimum}")
        if self.maximum is not None and value > self.maximum:
            raise ValueError(f"Value must be at most {self.maximum}")
        return value

    def get(self):
        return self._value

    def bind_to(self, target, attribute):
        """Update this attribute when the setting changes."""
        setattr(target, attribute, self._value)
        self._bindings.append((target, attribute))

    def _commit(self, value):
        self._value = self.validate(value)
        for target, attribute in self._bindings:
            setattr(target, attribute, self._value)
