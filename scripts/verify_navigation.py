#!/usr/bin/env python3
"""Bounded integration check of the actual Nav2 / Gazebo application."""
import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


class Verification(Node):
    def __init__(self):
        super().__init__('warehouse_setup_verification', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scans = 0
        self.valid_scans = 0
        self.clocks = []
        self.path = []
        self.pose = None
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl, pose_qos)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def scan(self, msg):
        self.scans += 1
        self.valid_scans += int(any(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges))

    def clock(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if not self.clocks or stamp != self.clocks[-1]:
            self.clocks.append(stamp)

    def odom(self, msg):
        p = msg.pose.pose.position
        self.path.append([p.x, p.y])

    def amcl(self, msg):
        p = msg.pose.pose.position
        self.pose = [p.x, p.y]

    def until(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(predicate())

    def lifecycle(self, name):
        client = self.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=5):
                return 'service_unavailable'
            future = client.call_async(GetState.Request())
            if not self.until(future.done, 5):
                return 'timeout'
            return future.result().current_state.label
        finally:
            self.destroy_client(client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='artifacts/verification.json')
    parser.add_argument('--initial-x', type=float, default=-2.0)
    parser.add_argument('--initial-y', type=float, default=-0.5)
    parser.add_argument('--goal-x', type=float, default=-2.0)
    parser.add_argument('--goal-y', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()
    report = {'passed': False, 'initial_pose': [args.initial_x, args.initial_y], 'goal': [args.goal_x, args.goal_y]}
    rclpy.init()
    node = Verification()
    start = time.monotonic()
    try:
        if not node.until(lambda: node.valid_scans >= 5 and len(node.clocks) >= 5 and len(node.path) >= 5, 60):
            raise RuntimeError('Timed out waiting for valid scan, odometry and simulation clock.')
        print('Sensor data and advancing simulation clock verified.', flush=True)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.position.x = args.initial_x
        initial.pose.pose.position.y = args.initial_y
        initial.pose.pose.orientation.w = 1.0
        initial.pose.covariance[0] = 0.25
        initial.pose.covariance[7] = 0.25
        initial.pose.covariance[35] = 0.0685
        for _ in range(10):
            initial.header.stamp = node.get_clock().now().to_msg()
            node.initial.publish(initial)
            node.until(lambda: False, 0.2)
        if not node.until(lambda: node.pose is not None, 30):
            raise RuntimeError('AMCL did not produce a localized pose.')
        lifecycle_deadline = time.monotonic() + 45
        while True:
            report['lifecycle_states'] = {name: node.lifecycle(name) for name in ['amcl', 'map_server', 'planner_server', 'controller_server', 'bt_navigator']}
            if all(state == 'active' for state in report['lifecycle_states'].values()):
                break
            if time.monotonic() > lifecycle_deadline:
                raise RuntimeError('Required Nav2 lifecycle nodes are not active.')
            node.until(lambda: False, 0.5)
        print('Localization and active Nav2 lifecycle nodes verified.', flush=True)
        if not node.navigate.wait_for_server(timeout_sec=20):
            raise RuntimeError('NavigateToPose action server is unavailable.')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.w = 1.0
        navigation_start = time.monotonic()
        future = node.navigate.send_goal_async(goal)
        if not node.until(future.done, 15):
            raise RuntimeError('Navigation goal acknowledgement timed out.')
        handle = future.result()
        report['goal_accepted'] = handle.accepted
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected the navigation goal.')
        print('Navigation goal accepted; waiting for physical movement and completion.', flush=True)
        result = handle.get_result_async()
        if not node.until(result.done, args.timeout):
            cancel = handle.cancel_goal_async()
            node.until(cancel.done, 5)
            raise RuntimeError('Navigation goal timed out and cancellation was requested.')
        report['navigation_wall_seconds'] = round(time.monotonic() - navigation_start, 3)
        report['action_status'] = result.result().status
        if report['action_status'] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Navigation did not succeed: status {report["action_status"]}.')
        node.until(lambda: False, 1)
        distance = sum(math.dist(a, b) for a, b in zip(node.path, node.path[1:]))
        report['odometry_path_metres'] = round(distance, 3)
        report['final_localized_pose'] = node.pose
        report['goal_error_metres'] = round(math.dist(node.pose, [args.goal_x, args.goal_y]), 3)
        if distance < 0.5 or report['goal_error_metres'] > 0.35:
            raise RuntimeError('Goal status succeeded, but movement / final pose validation failed.')
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        report['elapsed_wall_seconds'] = round(time.monotonic() - start, 3)
        report['scan_messages'] = node.scans
        report['valid_scan_messages'] = node.valid_scans
        report['clock_advanced_seconds'] = round(node.clocks[-1] - node.clocks[0], 3) if len(node.clocks) > 1 else 0
        report['odometry_samples'] = len(node.path)
        report['odometry_trace'] = node.path[::max(1, len(node.path) // 300)]
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'odometry_trace'}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
