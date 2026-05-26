#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from turtlesim_patrol.action import ExecuteCircle
import sys

class CirclePatrolClient(Node):

    def __init__(self):
        super().__init__('circle_patrol_client')
        self._action_client = ActionClient(self, ExecuteCircle, 'execute_circle')

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        print(f'\r  [FEEDBACK] {fb.distance_traveled:.2f} m | {fb.current_status}',
              end='', flush=True)

    def send_goal_and_wait(self, radius: float):
        self._action_client.wait_for_server()

        goal_msg = ExecuteCircle.Goal()
        goal_msg.radius = radius

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  
        )

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by server.')
            return

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        print()  
        result = get_result_future.result().result
        if result.success:
            self.get_logger().info(f'SUCCESS: {result.final_report}')
        else:
            self.get_logger().error(f'ABORTED/FAILED: {result.final_report}')


def main(args=None):
    rclpy.init(args=args)

    radius = 3.0
    if len(sys.argv) > 1:
        radius = float(sys.argv[1])  

    client = CirclePatrolClient()
    client.send_goal_and_wait(radius)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()