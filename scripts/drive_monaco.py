#!/usr/bin/env python3
"""Request continuous upstream Nav2 NavigateThroughPoses; do not implement a controller."""
import argparse
import json
import math
from pathlib import Path
import time
import subprocess

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class Lap(Node):
    def __init__(self, scene):
        super().__init__('monaco_checkpoint_mission', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scene = scene
        self.scans = 0
        self.path, self.poses, self.feedback_indices = [], [], []
        self.pose = None
        self.pose_samples, self.target_events = [], []
        self.last_feedback = -1
        self.targets = []
        self.max_recoveries = 0
        self.velocity_samples = []
        self.ordered_targets_passed = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/monaco/markers', qos)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.action = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.show_markers(0)

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.pose = [p.x, p.y]
        self.poses.append(self.pose)
        self.pose_samples.append({'wall_time': time.time(), 'x': p.x, 'y': p.y})
        if self.targets and self.ordered_targets_passed < len(self.targets):
            target = self.targets[self.ordered_targets_passed]
            if math.dist(self.pose, [target['x'], target['y']]) <= 0.70:
                self.ordered_targets_passed += 1

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.path.append([p.x, p.y])
        self.velocity_samples.append({'wall_time': time.time(), 'vx': msg.twist.twist.linear.x, 'wz': msg.twist.twist.angular.z})

    def on_scan(self, msg):
        self.scans += int(any(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges))

    def until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(predicate())

    def lifecycle_active(self, name):
        client = self.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=2):
                return False
            future = client.call_async(GetState.Request())
            return self.until(future.done, 3) and future.result().current_state.label == 'active'
        finally:
            self.destroy_client(client)

    def show_markers(self, completed):
        array = MarkerArray()
        for cp in self.scene['checkpoints']:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'checkpoints', cp['id'], Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cp['x'], cp['y'], 0.8
            m.pose.orientation.w = 1.0
            m.scale.z = 0.48
            m.color.r, m.color.g, m.color.b, m.color.a = (0.2, 1.0, 0.4, 1.0) if cp['id'] <= completed else (1.0, 0.8, 0.15, 1.0)
            m.text = f'CP{cp["id"]:02d}'
            array.markers.append(m)
        for index, server in enumerate(self.scene['servers']):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'edge_locations', index, Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = server['x'], server['y'], 0.8
            m.pose.orientation.w = 1.0
            m.scale.z = 0.36
            m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.85, 1.0, 1.0
            m.text = server['id'].replace('edge_', 'E')
            array.markers.append(m)
        for index, (name, pose) in enumerate([('START', self.scene['start']), ('FINISH', self.scene['finish'])]):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'endpoints', index, Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pose['x'], pose['y'], 1.6
            m.pose.orientation.w = 1.0
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 1.0, 0.4, 1.0) if index == 0 else (1.0, 0.3, 0.3, 1.0)
            m.text = name
            array.markers.append(m)
        self.markers.publish(array)

    def feedback(self, msg):
        feedback = msg.feedback
        self.max_recoveries = max(self.max_recoveries, int(feedback.number_of_recoveries))
        completed = len(self.targets) - int(feedback.number_of_poses_remaining)
        index = min(len(self.targets) - 1, max(0, completed))
        if index > self.last_feedback:
            self.last_feedback = index
            self.feedback_indices.append(index)
            self.target_events.append({'wall_time': time.time(), 'index': index})
            absolute = self.targets[index].get('id', len(self.scene['checkpoints']) + 1) - 1
            self.show_markers(absolute)
            print(f'Continuous Nav2 target {index + 1}/{len(self.targets)}; recoveries: {self.max_recoveries}', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=1500)
    parser.add_argument('--output', type=Path, help='Optional per-experiment report destination.')
    parser.add_argument('--limit', type=int, default=0, help='Optional initial checkpoints only, for a short check.')
    parser.add_argument('--from-checkpoint', type=int, default=0, help='Simulation test only: teleport to this checkpoint and verify subsequent targets.')
    args = parser.parse_args()
    if not 0 <= args.from_checkpoint <= 19 or args.limit < 0:
        parser.error('Checkpoint must be between 0 and 19; limit must be nonnegative.')
    project = Path(__file__).resolve().parents[1]
    scene = json.loads((project / 'scenarios/monaco/scenario.json').read_text())
    output = args.output or project / ('artifacts/monaco-checkpoint-test.json' if args.from_checkpoint else 'artifacts/monaco-run.json')
    if args.from_checkpoint:
        scene['start'] = scene['checkpoints'][args.from_checkpoint - 1]
    rclpy.init()
    node = Lap(scene)
    start_wall_time = time.time()
    start = time.monotonic()
    handle = None
    report = {'passed': False, 'full_course': args.limit == 0 and args.from_checkpoint == 0, 'communication_enabled': False, 'scheduler': 'upstream_default', 'mission_type': 'continuous_navigate_through_poses', 'moving_vehicles': 1, 'static_edge_car_models': len(scene['servers'])}
    try:
        if not node.until(lambda: node.scans >= 5 and len(node.path) >= 5, 90):
            raise RuntimeError('No live LiDAR / odometry data.')
        if args.from_checkpoint:
            p = scene['start']
            request = f'name: "racecar" position {{ x: {p["x"]} y: {p["y"]} z: 0.01 }} orientation {{ z: {math.sin(p["yaw"] / 2)} w: {math.cos(p["yaw"] / 2)} }}'
            moved = subprocess.run(['gz', 'service', '-s', '/world/monaco/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request], capture_output=True, text=True, timeout=8)
            if moved.returncode or 'data: true' not in moved.stdout:
                raise RuntimeError('Gazebo test repositioning failed.')
            report['test_initial_checkpoint'] = args.from_checkpoint
            node.until(lambda: False, 1)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        p = scene['start']
        initial.pose.pose.position.x, initial.pose.pose.position.y = p['x'], p['y']
        initial.pose.pose.orientation.z, initial.pose.pose.orientation.w = math.sin(p['yaw'] / 2), math.cos(p['yaw'] / 2)
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.0025
        initial.pose.covariance[35] = 0.0025
        for _ in range(8):
            initial.header.stamp = node.get_clock().now().to_msg()
            node.initial.publish(initial)
            node.until(lambda: False, 0.2)
        if not node.until(lambda: node.pose is not None, 30):
            raise RuntimeError('AMCL localization is unavailable.')
        for name in ['controller_server', 'planner_server', 'bt_navigator']:
            if not node.until(lambda: node.lifecycle_active(name), 45):
                raise RuntimeError(f'{name} is not active.')
        if not node.action.wait_for_server(timeout_sec=15):
            raise RuntimeError('Upstream NavigateThroughPoses is unavailable.')
        targets = (scene['checkpoints'] + [scene['finish']])[args.from_checkpoint:]
        if args.limit:
            targets = targets[:args.limit]
        report['requested_targets'] = len(targets)
        node.targets = targets
        goal = NavigateThroughPoses.Goal()
        goal.behavior_tree = str(project / 'scenarios/monaco/navigate_through_poses.xml')
        for target in targets:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = node.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = target['x'], target['y']
            p.pose.orientation.z, p.pose.orientation.w = math.sin(target['yaw'] / 2), math.cos(target['yaw'] / 2)
            goal.poses.append(p)
        report['goal_send_mono_ns'] = time.monotonic_ns()
        report['goal_send_epoch_ns'] = time.time_ns()
        future = node.action.send_goal_async(goal, feedback_callback=node.feedback)
        if not node.until(future.done, 15):
            raise RuntimeError('NavigateThroughPoses acknowledgement timed out.')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('NavigateThroughPoses was rejected.')
        report['goal_accepted_mono_ns'] = time.monotonic_ns()
        report['goal_accepted_sim_ns'] = node.get_clock().now().nanoseconds
        print('Upstream Nav2 accepted the checkpoint mission.', flush=True)
        result = handle.get_result_async()
        if not node.until(result.done, args.timeout):
            raise RuntimeError('Course timeout.')
        report['action_result_mono_ns'] = time.monotonic_ns()
        report['action_result_sim_ns'] = node.get_clock().now().nanoseconds
        message = result.result()
        report['action_status'] = message.status
        report['nav2_error_code'] = int(message.result.error_code)
        report['nav2_error_message'] = message.result.error_msg
        if message.status != GoalStatus.STATUS_SUCCEEDED or message.result.error_code:
            raise RuntimeError(f'Nav2 course failed; status={message.status}, error={message.result.error_msg}')
        node.until(lambda: False, 1)
        report['final_estimated_pose'] = node.pose
        report['final_goal_error_m'] = math.dist(node.pose, [targets[-1]['x'], targets[-1]['y']])
        distances = [min(math.dist(p, [cp['x'], cp['y']]) for p in node.poses) for cp in targets]
        report['minimum_estimated_distance_to_targets_m'] = distances
        report['ordered_targets_passed'] = node.ordered_targets_passed
        report['checkpoint_pass_radius_m'] = 0.70
        if report['final_goal_error_m'] > 0.40 or any(distance > 0.70 for distance in distances) or node.ordered_targets_passed != len(targets):
            raise RuntimeError('Action succeeded but checkpoint proximity verification failed.')
        report['passed'] = True
        report['success_wall_time'] = time.time()
        node.show_markers(min(len(scene['checkpoints']), args.from_checkpoint + len(targets)))
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or 'Interrupted.'
        if handle is not None and rclpy.ok():
            cancel = handle.cancel_goal_async()
            node.until(cancel.done, 5)
    finally:
        report['start_wall_time'] = start_wall_time
        report['end_wall_time'] = time.time()
        report['end_mono_ns'] = time.monotonic_ns()
        report['elapsed_wall_seconds'] = round(time.monotonic() - start, 3)
        report['valid_lidar_messages'] = node.scans
        report['odometry_distance_m'] = sum(math.dist(a, b) for a, b in zip(node.path, node.path[1:]))
        report['feedback_target_indices'] = node.feedback_indices
        report['navigation_recoveries'] = node.max_recoveries
        report['velocity_samples'] = node.velocity_samples
        report['pose_samples'] = node.pose_samples
        report['target_events'] = node.target_events
        report['localized_trace'] = node.poses[::max(1, len(node.poses) // 1500)]
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ['localized_trace', 'pose_samples', 'target_events', 'velocity_samples']}, indent=2), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
