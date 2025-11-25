import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MySimulationNode(Node):
    
    def __init__(self):
        super().__init__('simulation_node') ## ne pas supprimer, permet au truc davoir acces aux fonctionnalites ros2
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        # Example: read front, left, and right distances
        front = msg.ranges[0]
        left = msg.ranges[90]
        right = msg.ranges[270]

        self.get_logger().info("right : " + str(right))

        if left < 0.5 and right < 0.5 :
            self.move_robot(-0.5, 0.0) ## move backwards, good for debug
            self.get_logger().info("bloqueeeee")
        elif left < 1 and right > 0.8 :
            self.move_robot(0.7, 1.8) ## turn left
            self.get_logger().info("gauche*************************")
            self.get_logger().info("left : " + str(left))
            self.get_logger().info("right : " + str(right))
        elif right < 1 and left > 0.8 :
            self.move_robot(0.7, -1.8) ## turn right
            self.get_logger().info("droiteeeeeeeeeeeeeeeeeeeeeeeeeee")
            self.get_logger().info("left : " + str(left))
            self.get_logger().info("right : " + str(right))
        else :
            self.move_robot(0.9, 0.0) ## forward
            self.get_logger().info("avance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.get_logger().info("left : " + str(left))
            self.get_logger().info("right : " + str(right))
            self.get_logger().info("front : " + str(front))

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args) ## laisser ca tjrs en premier
    node = MySimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()






