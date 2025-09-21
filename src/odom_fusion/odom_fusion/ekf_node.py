#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
import tf2_ros

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_base', 0.20)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_cb, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_cb, 50)

        self._last_left = None
        self._last_right = None
        self._last_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.br = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info('Odom fusion node started')

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        imu_yaw = math.atan2(siny, cosy)
        self.yaw = 0.98 * self.yaw + 0.02 * imu_yaw

    def joint_cb(self, msg: JointState):
        now = self.get_clock().now()
        if self._last_time is None:
            if len(msg.position) >= 2:
                self._last_left = msg.position[0]
                self._last_right = msg.position[1]
            self._last_time = now
            return

        try:
            i_left = msg.name.index('wheel_left_joint')
            i_right = msg.name.index('wheel_right_joint')
            pos_left = msg.position[i_left]
            pos_right = msg.position[i_right]
        except Exception:
            if len(msg.position) >= 2:
                pos_left = msg.position[0]
                pos_right = msg.position[1]
            else:
                return

        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        dl = (pos_left - self._last_left) * self.wheel_radius
        dr = (pos_right - self._last_right) * self.wheel_radius

        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.wheel_base

        self.x += d_center * math.cos(self.yaw + d_theta/2.0)
        self.y += d_center * math.sin(self.yaw + d_theta/2.0)
        self.yaw += d_theta

        vx = d_center / dt
        vth = d_theta / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0,0,self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        self._last_left = pos_left
        self._last_right = pos_right
        self._last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
