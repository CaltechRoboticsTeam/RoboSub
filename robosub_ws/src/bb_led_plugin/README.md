# gz_led_plugin

A Gazebo (Harmonic / `gz-sim8`) system plugin for simulating LEDs using visuals and lights.
Define LED modes (blinking patterns, colors, intensity) in SDF and switch between them at
runtime over a Gazebo Transport topic (bridgeable to ROS 2).

## Features

- **Visual + Light LEDs**: define LEDs using visuals, lights, or both
- **Named modes**: multi-step sequences (idle, warning, emergency, …)
- **Selective activation**: a mode targets specific LEDs via `<active_leds>`, or all by default
- **Always-on support**: static LED states via `always_on="true"`
- **Runtime mode switching** over a Gazebo Transport topic (ROS 2 bridgeable)
- **Reset/Off**: send `"reset"` or `"off"` to return all LEDs to their default state
- **Case-insensitive** mode-name matching
- **Multi-instance safe**: LED names resolve relative to each model, so many identical models
  each carrying the plugin light up independently

## Build

```bash
cd your_workspace
colcon build --packages-select gz_led_plugin
source install/setup.bash   # env hook adds the plugin to GZ_SIM_SYSTEM_PLUGIN_PATH
```

The build produces **`libgz_led_plugin.so`** — this is the `filename=` an SDF `<plugin>` tag must
use. The env hook prepends the install `lib/` to `GZ_SIM_SYSTEM_PLUGIN_PATH` when the workspace is
sourced, so Gazebo can find it.

## Usage

### SDF example

```xml
<plugin name="led" filename="libgz_led_plugin.so">
  <led_group_name>tower_lamp</led_group_name>

  <led name="emergency_led">
    <visual_name>lamp_red::lamp_red_visual</visual_name>
    <light_name>lamp_red::emergency_light</light_name>
    <default_state>
      <color>0.3 0.0 0.0 1</color>
      <intensity>0.0</intensity>
    </default_state>
  </led>

  <led name="ready_led">
    <visual_name>lamp_green::lamp_green_visual</visual_name>
    <light_name>lamp_green::ready_light</light_name>
    <default_state>
      <color>0.0 0.3 0.0 1</color>
      <intensity>0.0</intensity>
    </default_state>
  </led>

  <startup_mode>ready</startup_mode>

  <mode name="ready">
    <active_leds>
      <led>ready_led</led>
    </active_leds>
    <step always_on="false">
      <color>0.0 1.0 0.0 1</color>
      <intensity>2.0</intensity>
      <on_time>1.0</on_time>
    </step>
    <step always_on="false">
      <color>0.0 0.3 0.0 1</color>
      <intensity>0.0</intensity>
      <on_time>1.0</on_time>
    </step>
  </mode>

  <mode name="emergency">
    <active_leds>
      <led>emergency_led</led>
    </active_leds>
    <step always_on="false">
      <color>1.0 0.0 0.0 1</color>
      <intensity>2.0</intensity>
      <on_time>1.0</on_time>
    </step>
    <step always_on="false">
      <color>0.3 0.0 0.0 1</color>
      <intensity>0.0</intensity>
      <on_time>1.0</on_time>
    </step>
  </mode>
</plugin>
```

### SDF reference

| Element | Description |
|---|---|
| `<led_group_name>` | Namespace for the mode-change topic. Defaults to `led_{model_name}` |
| `<led name="...">` | An LED with `<visual_name>`, `<light_name>`, and optional `<default_state>` |
| `<startup_mode>` | Mode to activate on startup. Defaults to the first defined mode |
| `<mode name="...">` | Named mode with optional `<active_leds>` and one or more `<step>` elements |
| `<step>` | `always_on` (attr), `<color>`, `<intensity>`, `<on_time>` (seconds) |

### Changing modes at runtime

**Gazebo CLI** — publish the mode name to `/<led_group_name>/change_led_mode`:
```bash
gz topic -t /tower_lamp/change_led_mode -m gz.msgs.StringMsg -p "data: 'emergency'"
gz topic -t /tower_lamp/change_led_mode -m gz.msgs.StringMsg -p "data: 'off'"   # reset
```

**ROS 2** — bridge the topic, then publish a `std_msgs/String`:
```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/led_robot/change_led_mode@std_msgs/msg/String]gz.msgs.StringMsg'],
)
```
```bash
ros2 topic pub --once /led_robot/change_led_mode std_msgs/msg/String "{data: 'emergency'}"
```

## Using with URDF

**Preserve fixed joints** — URDF→SDF merges fixed-joint links by default; keep the joint with:
```xml
<gazebo reference="top_to_front_led_joint">
  <preserveFixedJoint>true</preserveFixedJoint>
</gazebo>
```

**Name suffix** — URDF→SDF appends `_visual` / `_collision`, so a URDF visual `back_led_visual`
becomes `back_led_visual_visual` in SDF. Account for it in `<visual_name>`:
```xml
<!-- URDF: <visual name="back_led_visual"> -->
<led name="back_led">
  <visual_name>back_led::back_led_visual_visual</visual_name>
</led>
```

## Credits & License

Apache-2.0 — see [LICENSE](LICENSE). Original work by **Jasmeet Singh**
(jasmeet0915@gmail.com), [gz_sim_led_plugin](https://github.com/jasmeet0915/gz_sim_led_plugin).
