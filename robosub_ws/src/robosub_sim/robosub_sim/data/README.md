# T200 measurements

Source: Blue Robotics, *T200 Public Performance Data 10-20V, September 2019*.
The source URL and SHA-256 are stored in `t200.json`.

Each row is `[PWM_us, thrust_N, current_A, RPM]`. The 1,206 samples preserve
the six 10/12/14/16/18/20 V source tables. Source kgf values were multiplied
by standard gravity, 9.80665 m/s2. Converted force is rounded to 8 decimals,
current to 6 decimals, and RPM to 4 decimals.

These are steady, static bollard measurements with a Basic ESC R3 and stock
BLHeli_S 16.6 firmware, measured at the ESC power input. They do not establish
thrust in moving water, wiring voltage losses or installed vehicle dynamics.

The runtime model interpolates between measured voltages. It enforces the
documented ESC neutral band, makes each directional force/current curve
monotone using cumulative maxima, and limits estimated current. Requests
below the first stable measured thrust map to neutral. Hardware changes or
different ESC settings require checking the model against measured response.
