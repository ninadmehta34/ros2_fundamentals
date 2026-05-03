#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

from actions_quiz_msg.action import Distance

class QuizActionClient(Node):

    def __init__(self):
        super().__init__('distance_action_client')
        self._action_client = ActionClient(self, Distance, 'distance_as')
        self._done_event = threading.Event()

    def send_goal(self, x, y, yaw):
        goal_msg = Distance.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.yaw = yaw

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')

        # Wait until the action server is available
        self._action_client.wait_for_server()

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # Attach callback for when the goal response is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle the goal response
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the action server.')
            self._done_event.set()
            return

        self.get_logger().info('Goal accepted by the action server.')

        # Attach callback for when the result is received
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Handle the result
        result = future.result().result
        self.get_logger().info(f'Action completed with success: {result.success}')
        self.get_logger().info(f'Total distance traveled: {result.distance_traveled:.2f} meters')
        # Signal that we're done
        self._done_event.set()

    def feedback_callback(self, feedback_msg):
        # Handle feedback
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: Distance left = {feedback.distance_left:.2f} meters")

def main(args=None):
    rclpy.init(args=args)

    action_client = QuizActionClient()

    # Define your goal coordinates here
    x = 8.3
    y = -2.2
    yaw = -0.2  # Include yaw input

    action_client.send_goal(x, y, yaw)

    # Spin until the action is complete
    while rclpy.ok():
        rclpy.spin_once(action_client)
        if action_client._done_event.is_set():
            break

    # Clean up and shutdown
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()