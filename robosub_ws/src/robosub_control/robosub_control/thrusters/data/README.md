# Thruster data

`t200.json`: Blue Robotics T200 public performance tables, 10–20 V,
September 2019. Rows are `[PWM_us, thrust_N, current_A, RPM]`; source URL
and SHA-256 are embedded. The model interpolates voltage and limits current.

`m5.json`: approximate points digitized from VideoRay's thrust/power image;
source URL, image hash and digitization details are embedded. Simulation
assumes fixed 48 V, a 570 W limit, cubic command-to-power and mirrored
reverse thrust. Voltage and reverse performance are not specified by the
chart. Physical motor commands never use this model.

Both are static bollard models; installed moving-water performance needs
measurement. Neither estimates battery sag.
