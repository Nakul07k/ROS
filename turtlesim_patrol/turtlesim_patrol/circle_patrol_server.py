#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from turtlesim_patrol.action import ExecuteCircle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math
import time



WALL_MIN = 0.5
WALL_MAX = 10.5
LINEAR_VELOCITY = 1.5 

class CirclePatrolServer(Node):

    def __init__(self):
        super().__init__('circle_patrol_server')

        self.callback_group = ReentrantCallbackGroup()

        
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        
        self._action_server = ActionServer(
            self,
            ExecuteCircle,
            'execute_circle',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        
        self.current_pose = None

        self.get_logger().info('Circle Patrol Action Server is READY.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, goal_request):
        radius = goal_request.radius
        if radius <= 0.0:
            self.get_logger().warn(f'Rejecting goal: radius {radius} must be positive.')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting goal: radius = {radius:.2f} m')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    def stop_turtle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def is_near_wall(self, x, y):
        return x < WALL_MIN or x > WALL_MAX or y < WALL_MIN or y > WALL_MAX

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing circular patrol...')

        radius = goal_handle.request.radius
        angular_velocity = LINEAR_VELOCITY / radius  # w = v / r

        # Wait until we have a valid pose
        timeout = 5.0
        start_wait = time.time()
        while self.current_pose is None:
            if time.time() - start_wait > timeout:
                self.get_logger().error('Timed out waiting for turtle pose!')
                goal_handle.abort()
                result = ExecuteCircle.Result()
                result.success = False
                result.final_report = 'Mission Aborted: Could not get turtle pose.'
                return result
            time.sleep(0.05)

        # Record starting position
        x_start = self.current_pose.x
        y_start = self.current_pose.y
        self.get_logger().info(f'Start position: ({x_start:.2f}, {y_start:.2f})')

        # Compute expected circumference
        circumference = 2.0 * math.pi * radius

        feedback_msg = ExecuteCircle.Feedback()
        result = ExecuteCircle.Result()

        # Movement loop
        distance_traveled = 0.0
        loop_rate_hz = 20
        dt = 1.0 / loop_rate_hz
        completed_fraction = 0.0
        TOLERANCE = 0.2           
        MIN_TRAVEL = 0.5         

        twist = Twist()
        twist.linear.x = LINEAR_VELOCITY
        twist.angular.z = angular_velocity

        rate = self.create_rate(loop_rate_hz)

        while rclpy.ok():
            # Check for cancellation 
            if goal_handle.is_cancel_requested:
                self.stop_turtle()
                goal_handle.canceled()
                result.success = False
                result.final_report = 'Mission Cancelled by client.'
                self.get_logger().info('Goal cancelled.')
                return result

            # Wall collision check 
            if self.current_pose is not None:
                x = self.current_pose.x
                y = self.current_pose.y

                if self.is_near_wall(x, y):
                    self.stop_turtle()
                    goal_handle.abort()
                    result.success = False
                    result.final_report = 'Mission Aborted: Boundary Collision Imminent!'
                    self.get_logger().error(result.final_report)
                    return result

                # Arc distance traveled (arc = v * t = v / loop_rate per tick)
                distance_traveled += LINEAR_VELOCITY * dt
                completed_fraction = distance_traveled / circumference

                #  Check if turtle returned to start (after completing most loop) 
                if distance_traveled > MIN_TRAVEL:
                    dx = x - x_start
                    dy = y - y_start
                    dist_to_start = math.sqrt(dx * dx + dy * dy)

                    if completed_fraction >= 0.85 and dist_to_start < TOLERANCE:
                        # Full loop complete!
                        self.stop_turtle()
                        goal_handle.succeed()
                        result.success = True
                        result.final_report = (
                            f'Mission Complete! Full circle executed. '
                            f'Radius: {radius:.2f}m | '
                            f'Distance traveled: {distance_traveled:.2f}m'
                        )
                        self.get_logger().info(result.final_report)
                        return result

            #  Publish movement
            self.cmd_vel_publisher.publish(twist)

            #  Send feedback 
            feedback_msg.distance_traveled = float(distance_traveled)
            feedback_msg.current_status = (
                f'Moving... {completed_fraction * 100:.1f}% of circle complete'
            )
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()

        # Fallback (should not reach here)
        self.stop_turtle()
        goal_handle.abort()
        result.success = False
        result.final_report = 'Mission Aborted: ROS shutdown.'
        return result


def main(args=None):
    rclpy.init(args=args)
    server = CirclePatrolServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
