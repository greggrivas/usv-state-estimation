import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from ngc_interfaces.msg import GNSS, HeadingDevice
from ngc_utils.qos_profiles import default_qos_profile


class AdnavBridgeNode(Node):
    """
    Bridge between the Advanced Navigation INS/GNSS driver and the
    navigation message interface used by the control system.

    Subscribes to raw NavSatFix, Twist, and IMU from the AdNav driver
    and publishes GNSS and HeadingDevice messages at 10 Hz.
    """

    def __init__(self):
        super().__init__('adnav_bridge_node')

        self.create_subscription(NavSatFix, '/adnav_node/nav_sat_fix', self._fix_cb, default_qos_profile)
        self.create_subscription(Twist, '/adnav_node/twist', self._twist_cb, default_qos_profile)
        self.create_subscription(Imu, '/adnav_node/imu', self._imu_cb, default_qos_profile)

        self.gnss_pub = self.create_publisher(GNSS, 'gnss_sim_meas', default_qos_profile)
        self.heading_pub = self.create_publisher(HeadingDevice, 'gyrocompass_sim_meas', default_qos_profile)

        self.latest_fix = None
        self.latest_twist = None
        self.latest_imu = None

        self.step_size = 0.1  # 10 Hz
        self.create_timer(self.step_size, self._step)
        self.get_logger().info("AdNav bridge node started.")

    def _fix_cb(self, msg: NavSatFix):
        self.latest_fix = msg

    def _twist_cb(self, msg: Twist):
        self.latest_twist = msg

    def _imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def _step(self):
        self._publish_gnss()
        self._publish_heading()

    def _publish_gnss(self):
        gnss_msg = GNSS()
        gnss_msg.sensor_enabled = True

        if self.latest_fix is None:
            gnss_msg.sensor_healthy = False
            gnss_msg.signal_valid = False
            self.gnss_pub.publish(gnss_msg)
            return

        fix = self.latest_fix
        gnss_msg.sensor_healthy = True
        gnss_msg.signal_valid = fix.status.status >= 0
        gnss_msg.latitude = fix.latitude
        gnss_msg.longitude = fix.longitude
        gnss_msg.z = fix.altitude

        if self.latest_twist is not None:
            v_north = self.latest_twist.linear.x
            v_east = self.latest_twist.linear.y
            gnss_msg.sog = float(np.sqrt(v_north**2 + v_east**2))
            gnss_msg.cog = float(np.rad2deg(np.arctan2(v_east, v_north)) % 360.0)
            gnss_msg.sogcog_signal_valid = True
        else:
            gnss_msg.sog = 0.0
            gnss_msg.cog = 0.0
            gnss_msg.sogcog_signal_valid = False

        self.gnss_pub.publish(gnss_msg)
        self.latest_fix = None

    def _publish_heading(self):
        hd_msg = HeadingDevice()
        hd_msg.sensor_enabled = True

        if self.latest_imu is None:
            hd_msg.sensor_healthy = False
            hd_msg.signal_valid = False
            self.heading_pub.publish(hd_msg)
            return

        imu = self.latest_imu
        q = imu.orientation

        # Quaternion to yaw (NED frame, clockwise from north)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = np.arctan2(siny_cosp, cosy_cosp)
        heading = float((90.0 - np.rad2deg(yaw_rad)) % 360.0)

        hd_msg.sensor_healthy = True
        hd_msg.signal_valid = True
        hd_msg.heading_deg = heading
        hd_msg.rot_degsec = float(np.rad2deg(imu.angular_velocity.z))

        self.heading_pub.publish(hd_msg)
        self.latest_imu = None


def main(args=None):
    rclpy.init(args=args)
    node = AdnavBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
