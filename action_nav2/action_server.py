#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from tf_transformations import quaternion_from_euler
from action_msgs.msg import GoalStatus
import time
from math import sqrt, pow
from actions_quiz_msg.action import Distance
from nav_msgs.msg import Odometry

class QuizActionServer(Node):
    def __init__(self):
        super().__init__('distance_action_server')

        # Action server to accept goals
        self.action_server = ActionServer(
            self, Distance, 'distance_as', self.execute_callback)

        # Publisher to set initial pose
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Wait for localization to be active
        self.wait_for_localization()

        # Set the initial pose
        self.set_initial_pose(0.0, 0.0, 0.0)

        # Variables for tracking
        self.current_position = None
        self.goal_position = None
        self.total_distance_traveled = 0.0
        self.previous_position = None

        # Subscription to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.get_logger().info("Action Server is ready.")

        # Initialize goal_handle
        self.goal_handle = None

    def wait_for_localization(self):
        self.get_logger().info("Waiting for localization to be active...")
        while not self.count_subscribers('/initialpose') > 0:
            self.get_logger().info("Waiting for subscribers to /initialpose...")
            time.sleep(1.0)
        self.get_logger().info("Localization is active.")

    def set_initial_pose(self, x, y, yaw):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, yaw)
        initial_pose.pose.pose.orientation.x = q[0]
        initial_pose.pose.pose.orientation.y = q[1]
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]

        # Publish initial pose multiple times
        for _ in range(10):
            self.initial_pose_publisher.publish(initial_pose)
            time.sleep(0.1)

        self.get_logger().info(f"Initial pose set to x: {x}, y: {y}, yaw: {yaw}")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = position

        if self.previous_position is not None:
            distance = sqrt(
                pow(position.x - self.previous_position.x, 2) +
                pow(position.y - self.previous_position.y, 2)
            )
            self.total_distance_traveled += distance

        self.previous_position = position

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')

        # Store the goal_handle for use in publish_feedback
        self.goal_handle = goal_handle

        x = goal_handle.request.x
        y = goal_handle.request.y
        yaw = goal_handle.request.yaw

        self.goal_position = (x, y)
        self.total_distance_traveled = 0.0
        self.previous_position = None

        self.get_logger().info(f"Navigating to: x={x}, y={y}, yaw={yaw}")

        # Send navigation goal
        navigation_result = await self.send_navigation_goal(x, y, yaw)

        if navigation_result:
            self.get_logger().info('Goal reached successfully!')
            goal_handle.succeed()
            result = Distance.Result()
            result.success = True
            result.distance_traveled = self.total_distance_traveled
            return result
        else:
            self.get_logger().info('Failed to reach goal.')
            goal_handle.abort()
            result = Distance.Result()
            result.success = False
            result.distance_traveled = self.total_distance_traveled
            return result

    async def send_navigation_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()

        # Set goal position and orientation
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Wait for action server
        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.nav_to_pose_client.wait_for_server()

        # Send goal
        self.get_logger().info(f"Sending navigation goal to: x={x}, y={y}, yaw={yaw}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        nav_goal_handle = await send_goal_future

        if not nav_goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return False

        self.get_logger().info('Navigation goal accepted')

        # Start feedback timer
        self.feedback_timer = self.create_timer(1.0, self.publish_feedback)

        # Wait for result
        get_result_future = nav_goal_handle.get_result_async()
        nav_result = await get_result_future

        # Stop feedback timer
        self.feedback_timer.cancel()

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            return True
        else:
            self.get_logger().info(f'Navigation failed with status: {nav_result.status}')
            return False

    def publish_feedback(self):
        if self.current_position is None or self.goal_position is None:
            return

        # Calculate distance left
        dx = self.goal_position[0] - self.current_position.x
        dy = self.goal_position[1] - self.current_position.y
        distance_left = sqrt(dx * dx + dy * dy)

        # Publish feedback to action client
        feedback_msg = Distance.Feedback()
        feedback_msg.distance_left = distance_left

        if self.goal_handle is not None:
            self.goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: Distance left = {distance_left:.2f} meters")
        else:
            self.get_logger().warn("Goal handle is None, cannot publish feedback.")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = QuizActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()