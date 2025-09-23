#!/usr/bin/env python3
import time
from typing import Dict, Any, List, Tuple
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import argparse
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import Header

# your sim class
from .turtlebot4_sim import Turtlebot4Sim, HAS_VIEWER  
try:
    from mujoco import viewer
except Exception:
    viewer = None


def _to_float(x):
    # sim.read_sensors() returns numpy arrays; make scalars when needed
    return float(np.asarray(x).reshape(-1)[0])


def _xyzw_from_mjcf_quat(wxyz: np.ndarray) -> Tuple[float, float, float, float]:
    # MuJoCo quaternions are [w, x, y, z]; ROS uses [x, y, z, w]
    w, x, y, z = [float(v) for v in wxyz]
    return (x, y, z, w)


def _unwrap_sorted_angles(samples: List[Tuple[float, float]]) -> Tuple[np.ndarray, np.ndarray]:
    """
    samples: list of (angle_rad in [0, 2π), range_m)
    Returns sorted, monotonically increasing angles (possibly shifted by -2π) and matching ranges.
    """
    if not samples:
        return np.array([]), np.array([])

    a = np.array([s[0] for s in samples], dtype=np.float64)
    r = np.array([s[1] for s in samples], dtype=np.float32)

    # Sort by angle first
    idx = np.argsort(a)
    a = a[idx]
    r = r[idx]

    # If there is a wrap (big gap), shift the wrapped block by -2π so angles are monotonic
    gaps = np.diff(a)
    if gaps.size:
        k = int(np.argmax(gaps))
        if gaps[k] > np.pi:
            # split after k, shift the second block by -2π and concatenate
            a2 = np.concatenate([a[k+1:] - 2*np.pi, a[:k+1]])
            r2 = np.concatenate([r[k+1:], r[:k+1]])
            a, r = a2, r2

    return a, r


class TB4RosNode(Node):
    def __init__(self, sim: Turtlebot4Sim):
        super().__init__('tb4_node')
        self.sim = sim

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # Subscriber
        self.last_cmd = Twist()
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Cache ctrl ranges for clamping (if available)
        self.forward_min, self.forward_max = -1.0, 1.0
        self.turn_min, self.turn_max = -1.0, 1.0
        try:
            act = self.sim._act_id
            ctrlrng = self.sim.model.actuator_ctrlrange
            if 'forward' in act:
                lo, hi = ctrlrng[act['forward']]
                self.forward_min, self.forward_max = float(lo), float(hi)
            if 'turn' in act:
                lo, hi = ctrlrng[act['turn']]
                self.turn_min, self.turn_max = float(lo), float(hi)
        except Exception:
            pass

        # Laser model limits (ensure it matches your XML cutoff = 10)
        self.range_min = 0.02
        self.range_max = 10.0

        # Names for joint state (match MJCF)
        self.js_names = ['left_wheel_joint', 'right_wheel_joint']

    def _cmd_vel_cb(self, msg: Twist):
        self.last_cmd = msg

    # --- Builders for ROS messages ---

    def build_imu_msg(self, sensors: Dict[str, Any]) -> Imu:
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # orientation
        x, y, z, w = _xyzw_from_mjcf_quat(sensors['imu_quat'])
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        # angular velocity (rad/s)
        av = np.asarray(sensors['imu_ang']).reshape(3)
        msg.angular_velocity.x = float(av[0])
        msg.angular_velocity.y = float(av[1])
        msg.angular_velocity.z = float(av[2])

        # linear acceleration (m/s^2)
        aa = np.asarray(sensors['imu_acc']).reshape(3)
        msg.linear_acceleration.x = float(aa[0])
        msg.linear_acceleration.y = float(aa[1])
        msg.linear_acceleration.z = float(aa[2])

        # Covariances: unknown -> set diag to -1
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        return msg

    def build_joint_state_msg(self, sensors: Dict[str, Any]) -> JointState:
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()

        # Extract (these are arrays in your sim.read_sensors(); make floats)
        lp = _to_float(sensors['left_pos'])
        rp = _to_float(sensors['right_pos'])
        lv = _to_float(sensors['left_vel'])
        rv = _to_float(sensors['right_vel'])


        js.name = self.js_names
        js.position = [lp, rp]
        js.velocity = [lv, rv]
        return js

    def build_scan_msg(self, last_scan: List[Tuple[float, float]]) -> LaserScan:
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'rplidar_link'  # matches your MJCF body name

        # Time parameters (you collect 500/10 = 50 rays over 0.1 s)
        scan.scan_time = 1.0 / float(self.sim.control_hz)
        scan.time_increment = 1.0 / float(self.sim.sim_hz)

        a, r = _unwrap_sorted_angles(last_scan)
        n = int(a.size)

        if n == 0:
            scan.angle_min = 0.0
            scan.angle_max = 0.0
            scan.angle_increment = 0.0
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            scan.ranges = []
            scan.intensities = []
            return scan

        scan.angle_min = float(a[0])
        scan.angle_max = float(a[-1])

        # If there's only 1 sample, set an arbitrary small increment to avoid division by zero
        if n > 1:
            incs = np.diff(a)
            scan.angle_increment = float(np.mean(incs))
        else:
            scan.angle_increment = 0.0

        # Clamp ranges and convert to list
        r = np.clip(r, self.range_min, self.range_max)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float(x) for x in r.tolist()]
        scan.intensities = []  # not provided
        return scan

    def build_odom_msg(self, sensors: Dict[str, Any]) -> Any:
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # position
        pos = np.asarray(sensors['base_pos']).reshape(3)
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        # orientation
        x, y, z, w = _xyzw_from_mjcf_quat(sensors['imu_quat'])
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        # velocities
        lv = np.asarray(sensors['base_linvel']).reshape(3)
        av = np.asarray(sensors['imu_ang']).reshape(3)
        # TODO: transform to odom frame! Currently linear velocity is in world frame???
        odom.twist.twist.linear.x = float(lv[0])
        odom.twist.twist.linear.y = float(lv[1])    
        odom.twist.twist.linear.z = float(lv[2])
        odom.twist.twist.angular.x = float(av[0])
        odom.twist.twist.angular.y = float(av[1])
        odom.twist.twist.angular.z = float(av[2])
    
        return odom
    
    def apply_cmd(self):
        # Map /cmd_vel to sim controls; clamp to ctrlrange
        fwd = float(self.last_cmd.linear.x)
        turn = float(self.last_cmd.angular.z)
        fwd = max(self.forward_min, min(self.forward_max, fwd))
        turn = max(self.turn_min, min(self.turn_max, turn))
        self.sim.set_control(forward=fwd, turn=turn)


def publish_all(sim: Turtlebot4Sim, node: TB4RosNode):
    imu_msg = node.build_imu_msg(sim.sensors)
    node.pub_imu.publish(imu_msg)

    js_msg = node.build_joint_state_msg(sim.sensors)
    node.pub_js.publish(js_msg)

    odom_msg = node.build_odom_msg(sim.sensors)
    node.pub_odom.publish(odom_msg)

    scan_msg = node.build_scan_msg(sim.last_scan)
    node.pub_scan.publish(scan_msg)

    # Create broadcaster if not already present
    t = TransformStamped()
    t.header.stamp = odom_msg.header.stamp
    t.header.frame_id = odom_msg.header.frame_id
    t.child_frame_id = odom_msg.child_frame_id
    t.transform.translation.x = odom_msg.pose.pose.position.x
    t.transform.translation.y = odom_msg.pose.pose.position.y
    t.transform.translation.z = odom_msg.pose.pose.position.z
    t.transform.rotation = odom_msg.pose.pose.orientation
    node.tf_broadcaster.sendTransform(t)
    
def main():
    parser = argparse.ArgumentParser()

    package_share_dir = get_package_share_directory('turtlebot4_mujoco')
    default_xml_path = os.path.join(package_share_dir, 'scene.xml')

    parser.add_argument('--xml', default=default_xml_path, help='Path to MJCF scene.')
    parser.add_argument('--headless', action='store_true', help='Run MuJoCo without GUI.')
    parser.add_argument('--no_realtime', action='store_true', help='Run MuJoCo as fast as possible.')
    parser.add_argument('--sim_hz', type=int, default=500)
    parser.add_argument('--control_hz', type=int, default=20)
    args = parser.parse_args()

    # Init ROS 2
    rclpy.init()

    # Build sim
    sim = Turtlebot4Sim(args.xml, from_string=False, open_viewer=(not args.headless),
                        sim_hz=args.sim_hz, control_hz=args.control_hz)
    node = TB4RosNode(sim)

    node.get_logger().info(f"Sensors: {sim.sensor_names()}")
    node.get_logger().info(f"Actuators: {sim.actuator_names()}")

    try:
        if sim._use_viewer and HAS_VIEWER and viewer is not None:
            with viewer.launch_passive(sim.model, sim.data) as v:
                sim._viewer = v
                while True:
                    node.apply_cmd()
                    sim.iterate(realtime=(not args.no_realtime))
                    publish_all(sim, node)
                    rclpy.spin_once(node, timeout_sec=0.0)
        else:
            while True:
                node.apply_cmd()
                sim.iterate(realtime=(not args.no_realtime))
                publish_all(sim, node)
                rclpy.spin_once(node, timeout_sec=0.0)                

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
