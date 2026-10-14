"""Verify the real planning Service against synthetic maps without robot hardware."""

import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from gn10_navigation.srv import ComputePath


def test_compute_path_results():
    """Verify explicit failures, corrected endpoints, short paths and old-path clearing."""
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('planning_service_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = node.create_client(ComputePath, '/navigation/compute_path')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = node.create_publisher(OccupancyGrid, '/costmap', qos)
    paths = []
    node.create_subscription(Path, '/planned_path', paths.append, 10)
    process = subprocess.Popen([os.environ['PLANNER_EXECUTABLE']],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def pose(x, y):
        """Create a normalized synthetic map pose."""
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.orientation.w = 1.0
        return message

    def call(start=None, goal=None):
        """Invoke the planning API and require a bounded response."""
        request = ComputePath.Request()
        request.start = start or pose(0.5, 2.5)
        request.goal = goal or pose(4.5, 2.5)
        request.generation = 42
        future = client.call_async(request)
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert future.done(), 'planning response timed out'
        result = future.result()
        assert result.generation == 42
        assert bool(result.path.poses) == result.success
        assert result.reason
        return result

    def map_result(grid, expected, start=None, goal=None):
        """Publish a replacement map and wait until its expected result is observed."""
        end = time.monotonic() + 5
        while time.monotonic() < end:
            maps.publish(grid)
            executor.spin_once(timeout_sec=0.03)
            result = call(start, goal)
            if result.error_code == expected:
                return result
        raise AssertionError(f'expected planning result {expected}')

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 1.0
    grid.info.width = 5
    grid.info.height = 5
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 25
    try:
        assert client.wait_for_service(timeout_sec=5)
        assert call().error_code == ComputePath.Response.NO_MAP
        result = map_result(grid, ComputePath.Response.SUCCESS)
        assert result.path.poses[0].pose.position.x == 0.5
        assert result.path.poses[-1].pose.position.x == 4.5
        assert result.path.poses[-1].pose.position.y == 2.5
        # A same-cell short path must contain actual requested endpoints.
        short = call(pose(1.2, 1.3), pose(1.8, 1.7))
        assert short.success
        assert short.path.poses[0].pose.position.x == 1.2
        assert short.path.poses[-1].pose.position.x == 1.8
        assert short.path.poses[-1].pose.position.y == 1.7
        invalid = pose(math.nan, 2.5)
        assert call(invalid).error_code == ComputePath.Response.INVALID_START
        invalid = pose(1, 1)
        invalid.header.frame_id = 'odom'
        assert call(goal=invalid).error_code == ComputePath.Response.INVALID_GOAL
        invalid = pose(1, 1)
        invalid.pose.orientation.w = 0.0
        assert call(goal=invalid).error_code == ComputePath.Response.INVALID_GOAL
        assert call(goal=pose(-0.1, 1)).error_code == ComputePath.Response.GOAL_BLOCKED
        grid.data[10] = -2  # Custom unsigned cost 254.
        assert map_result(grid, ComputePath.Response.START_BLOCKED).success is False
        grid.data = [0] * 25
        grid.data[14] = -2
        assert map_result(grid, ComputePath.Response.GOAL_BLOCKED).success is False
        grid.data = [0] * 25
        for row in range(5):
            grid.data[row * 5 + 2] = -2
        assert map_result(grid, ComputePath.Response.NO_PATH).success is False
        end = time.monotonic() + 2
        while time.monotonic() < end and (not paths or paths[-1].poses):
            executor.spin_once(timeout_sec=0.01)
        assert paths and not paths[-1].poses
        # An invalid latest map must invalidate a previously usable map.
        grid.data = [0] * 24
        assert map_result(grid, ComputePath.Response.NO_MAP).success is False
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
