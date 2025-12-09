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
        self.Kp = 1.2
        self.max_speed = 0.9
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        # Example: read front, left, and right distances
        n = len(ranges)
        front = msg.ranges[n//2]
        left = msg.ranges[int(n*3/4)]
        right = msg.ranges[int(n*1/4)]

        self.get_logger().info("left : " + str(left))
        self.get_logger().info("right : " + str(right))
        self.get_logger().info("front : " + str(front))

        error = left - right ## we want left = right for a centered robot
        angular = self.Kp*error
        
        angular = min(2.0,angular)

        if front < 1.0:
            ## if we are in front of the wall, turn to the larger free space
            angular = 1.2 if right > left else -1.2
            linear = 0.0
            self.move_robot(linear,angular)
            self.get_logger().info("touuuurne")
            self.get_logger().info("angular : "+str(angular))
            return
        
        linear = self.max_speed - abs(angular)*0.3
        linear = max(0.15,linear)

        self.move_robot(linear,angular)

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






