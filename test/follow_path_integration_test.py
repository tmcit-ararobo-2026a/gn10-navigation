"""Exercise Action cancellation, independent leases and TF faults with synthetic poses."""

import math
import os
import subprocess
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from gn10_navigation.action import FollowPath
from gn10_navigation.msg import ControlLease, PathVelocity


def test_follow_path_faults():
    """Verify rejected old goals, cancel, lease expiry, TF loss, heading and no resume."""
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('following_action_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = ActionClient(node, FollowPath, '/navigation/follow_path')
    leases = node.create_publisher(ControlLease, '/navigation/control_lease', 1)
    transforms = TransformBroadcaster(node)
    velocities, speed_paths = [], []
    node.create_subscription(PathVelocity, '/navigation/path_velocity', velocities.append, 10)
    node.create_subscription(Path, '/speed_path', speed_paths.append, 10)
    process = subprocess.Popen([os.environ['FOLLOWER_EXECUTABLE'], '--ros-args',
                                '-p', 'max_pose_age:=0.1'],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    pose_x, pose_y, pose_yaw = 0.0, 0.0, 0.0
    generation, lease_sequence = 0, 0
    publish_tf, publish_lease = True, False
    duplicate_lease = None

    def step():
        """Feed optional current TF and lease, then dispatch DDS callbacks."""
        nonlocal lease_sequence
        stamp = node.get_clock().now().to_msg()
        if publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = pose_x
            transform.transform.translation.y = pose_y
            transform.transform.rotation.z = math.sin(pose_yaw * 0.5)
            transform.transform.rotation.w = math.cos(pose_yaw * 0.5)
            transforms.sendTransform(transform)
        if publish_lease:
            lease_sequence += 1
            lease = ControlLease()
            lease.stamp = stamp
            lease.producer_session = 123
            lease.generation = generation
            lease.update_sequence = lease_sequence
            lease.enabled = True
            leases.publish(duplicate_lease or lease)
        executor.spin_once(timeout_sec=0.005)

    def wait_for(predicate, timeout=5):
        """Wait for a specific action or velocity observation."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert process.poll() is None
            step()
            if predicate():
                return
        raise AssertionError('follow-path outcome timed out')

    def await_future(future):
        """Resolve an asynchronous Action operation while feeding inputs."""
        wait_for(future.done)
        return future.result()

    def path(goal_x=2.0, goal_y=0.0, yaw=0.0):
        """Build a finite synthetic two-point path in map."""
        message = Path()
        message.header.frame_id = 'map'
        for x, y in ((pose_x, pose_y), (goal_x, goal_y)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            message.poses.append(pose)
        return message

    def goal(message=None, requested_generation=None):
        """Submit a goal under the selected producer and execution generation."""
        request = FollowPath.Goal()
        request.path = message or path()
        request.producer_session = 123
        request.generation = requested_generation or generation
        return await_future(client.send_goal_async(request))

    def start_next():
        """Start a fresh generation after observing lease delivery."""
        nonlocal generation, publish_lease
        generation += 1
        publish_lease = True
        # Give the independent permit topic time to arrive before the goal service.
        end = time.monotonic() + 0.06
        while time.monotonic() < end:
            step()
        handle = goal()
        assert handle.accepted
        wait_for(lambda: velocities and velocities[-1].generation == generation
                 and velocities[-1].valid and velocities[-1].velocity.linear.x > 0)
        return handle

    try:
        assert client.wait_for_server(timeout_sec=5)
        generation = 1
        assert not goal().accepted  # No lease.
        publish_lease = True
        wait_for(lambda: leases.get_subscription_count() == 1)
        end = time.monotonic() + 0.1
        while time.monotonic() < end:
            step()
        invalid = path()
        invalid.header.frame_id = 'odom'
        assert not goal(invalid).accepted
        handle = goal()
        assert handle.accepted
        wait_for(lambda: velocities and velocities[-1].valid
                 and velocities[-1].velocity.linear.x > 0)
        assert not goal().accepted  # Concurrent replacement must cancel first.
        cancel = await_future(handle.cancel_goal_async())
        assert cancel.goals_canceling
        outcome = await_future(handle.get_result_async())
        assert outcome.status == GoalStatus.STATUS_CANCELED
        assert outcome.result.error_code == FollowPath.Result.CANCELED
        wait_for(lambda: velocities and not velocities[-1].valid
                 and velocities[-1].velocity.linear.x == 0)
        assert not goal().accepted  # Same execution cannot be resurrected.

        handle = start_next()
        publish_lease = False
        outcome = await_future(handle.get_result_async())
        assert outcome.result.error_code == FollowPath.Result.LEASE_EXPIRED
        assert not outcome.result.success
        publish_lease = True
        end = time.monotonic() + 0.15
        while time.monotonic() < end:
            step()
        assert not velocities[-1].valid and velocities[-1].velocity.linear.x == 0

        handle = start_next()
        publish_tf = False
        outcome = await_future(handle.get_result_async())
        assert outcome.result.error_code == FollowPath.Result.TF_ERROR
        publish_tf = True
        end = time.monotonic() + 0.15
        while time.monotonic() < end:
            step()
        assert not velocities[-1].valid and velocities[-1].velocity.linear.x == 0
        assert not speed_paths[-1].poses

        handle = start_next()
        duplicate_lease = ControlLease()
        duplicate_lease.stamp = node.get_clock().now().to_msg()
        duplicate_lease.producer_session = 123
        duplicate_lease.generation = generation
        duplicate_lease.update_sequence = lease_sequence + 1
        duplicate_lease.enabled = True
        outcome = await_future(handle.get_result_async())
        assert outcome.result.error_code == FollowPath.Result.LEASE_EXPIRED
        duplicate_lease = None

        # Point goal: fixed heading during travel still aligns final orientation.
        generation += 1
        end = time.monotonic() + 0.06
        while time.monotonic() < end:
            step()
        aligned = goal(path(pose_x, pose_y, math.pi / 2))
        assert aligned.accepted
        wait_for(lambda: velocities[-1].valid and velocities[-1].velocity.angular.z > 0)
        assert velocities[-1].velocity.linear.x == 0
        pose_yaw = math.pi / 2
        outcome = await_future(aligned.get_result_async())
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.success and outcome.result.generation == generation
        wait_for(lambda: not velocities[-1].valid and velocities[-1].velocity.angular.z == 0)
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
