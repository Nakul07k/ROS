import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class pub(Node):
    def __init__(self):
        super().__init__("pub")
        self.node=self.create_publisher(String,"topic",1)
        self.timer=self.create_timer(1.0,self.timerc)
        self.i=0

    def timerc(self):
        msg=String()
        msg.data="hello %d"%self.i
        self.node.publish(msg)
        self.get_logger().info(msg.data)
        self.i+=1

def main():
    rclpy.init()
    node=pub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
