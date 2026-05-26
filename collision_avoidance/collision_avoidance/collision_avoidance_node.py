import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    qos_profile_sensor_data,
)
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

WALL_MIN = 0.0
WALL_MAX = 11.0


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node')
        self.declare_parameter('safety_threshold', 1.5)

        self.current_x     = 5.5
        self.current_y     = 5.5
        self.current_theta = 0.0
        self.pose_received = False

        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    
        self.vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            cmd_vel_qos,
        )

        
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            qos_profile_sensor_data,
        )

        
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f'CollisionAvoidanceNode ready at 50 Hz. '
            f'safety_threshold = {self.get_safety_threshold():.2f} m'
        )

  
    def get_safety_threshold(self) -> float:
        return self.get_parameter('safety_threshold') \
                   .get_parameter_value().double_value

    def is_near_wall(self, x: float, y: float, threshold: float) -> bool:
        return (
            x < WALL_MIN + threshold or
            x > WALL_MAX - threshold or
            y < WALL_MIN + threshold or
            y > WALL_MAX - threshold
        )

    def spin_toward_center(self, x: float, y: float, theta: float) -> float:
        """Returns angular.z to spin turtle toward center (5.5, 5.5)."""
        angle_to_center = math.atan2(5.5 - y, 5.5 - x)
        angle_diff = angle_to_center - theta
        
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        return 2.5 if angle_diff >= 0 else -2.5

    
    def pose_callback(self, msg: Pose) -> None:
        self.current_x     = msg.x
        self.current_y     = msg.y
        self.current_theta = msg.theta
        self.pose_received = True

    
    def control_loop(self) -> None:
        if not self.pose_received:
            return  

        threshold = self.get_safety_threshold()

        if self.is_near_wall(self.current_x, self.current_y, threshold):
            
            twist = Twist()
            twist.linear.x  = 0.0
            twist.angular.z = self.spin_toward_center(
                self.current_x, self.current_y, self.current_theta
            )
            self.vel_pub.publish(twist)

            self.get_logger().warn(
                f'WALL OVERRIDE — x={self.current_x:.2f} '
                f'y={self.current_y:.2f} '
                f'threshold={threshold:.2f}',
                throttle_duration_sec=1.0,
            )
        # Safe zone: publish nothing — teleop drives freely


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()