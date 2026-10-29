#!/usr/bin/env python3
"""Turn the simulated altimeter into a depth reading: /depth, metres, +down.

WHY THIS NODE EXISTS. The real sub has a pressure sensor (Bar30-class) whose
driver publishes depth in metres below the surface. Gazebo has no pressure
sensor, so the sim uses an *altimeter*, which reports something different:

    vertical_position   metres moved UP since the sensor was created
    vertical_reference  world z where the sensor was created (spawn height)
    vertical_velocity   dz/dt, metres/second, positive up

so the sub's world height is  z = vertical_reference + vertical_position
and its depth below the surface (world z = 0 in pool.sdf) is  depth = -z.

Everything above this node sees only /depth and never learns that it came
from an altimeter. On the vehicle, replace this node with the pressure
driver and nothing else changes.

Parameters
    surface_z   world z of the water surface (default 0.0, matches pool.sdf)
"""
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter
from std_msgs.msg import Float64

from robosub_control.config import constants


class DepthSensor(Node):
    def __init__(self):
        super().__init__('depth_sensor')
        self.declare_parameter('surface_z', constants.SIM_SURFACE_Z_M)
        self.surface_z = float(self.get_parameter('surface_z').value)

        self.pub = self.create_publisher(Float64, '/depth', 10)
        self.create_subscription(Altimeter, '/altimeter', self.on_alt, 10)
        self._logged_ref = False
        self.get_logger().info(
            f'depth_sensor up: /altimeter -> /depth (surface_z={self.surface_z})')

    def on_alt(self, msg: Altimeter):
        world_z = msg.vertical_reference + msg.vertical_position
        depth = self.surface_z - world_z
        if not self._logged_ref:
            self._logged_ref = True
            self.get_logger().info(
                f'altimeter reference z={msg.vertical_reference:.3f} m '
                f'-> first depth {depth:.3f} m')
        self.pub.publish(Float64(data=depth))


def main():
    rclpy.init()
    node = DepthSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():          # a real error; a SIGTERM mid-spin is not
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():          # a SIGTERM already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
