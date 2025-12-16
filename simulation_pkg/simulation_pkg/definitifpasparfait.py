
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MySimulationNode(Node):
    
    def __init__(self):
        super().__init__('simulation_node') # important, allows node to access ROS 2 functions
        self.subscriber_ = self.create_subscription( # receives scanner data
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # declaring necessary constants and initial state of the robot
        self.Kp = 0.40
        self.max_speed = 0.8
        self.max_turn_speed = 0.6
        self.state = "FORWARD"
        self.turn_direction = None
        self.turn_trigger = 0.9
        self.front_open_threshold = 2.5
        self.linear = 0.6
    
    def scan_callback(self, msg):
        ranges = msg.ranges

        n = len(ranges) # guarantees that we are using the correct values for our application
        # regardless of the amount of values the sensor captures
        front = msg.ranges[n//2]
        left = msg.ranges[int(n*3/4)]
        right = msg.ranges[int(n*1/4)]

        # prints the current sensor values, useful for troubleshooting and tuning
        self.get_logger().info("left : " + str(left))
        self.get_logger().info("right : " + str(right))
        self.get_logger().info("front : " + str(front))

        # FSM : goes into the behaviour decided in the previous loop / initialisation
        if self.state == "FORWARD":
            self.forward_behavior(front, left, right)
        elif self.state == "TURN":
            self.turn_behavior(front,left,right)
        elif self.state == "REALIGN":
            self.realign_behavior(front, left, right)

    # -------------------------
    # FORWARD: normal driving
    # -------------------------
    def forward_behavior(self, front, left, right):

        # TURN condition
        if front < self.turn_trigger:
            self.state = "TURN"
            # turn toward the more open side
            self.turn_direction = "LEFT" if right > left else "RIGHT"
            return

        # centering controller
        error = left - right
        angular = self.Kp * error

        # limit angular velocity
        angular = max(min(angular, self.max_turn_speed), -self.max_turn_speed)

        # adjust speed
        self.linear = self.max_speed - abs(angular) * 0.2
        self.linear = max(0.2, self.linear) # error we didn't catch before, should be 0.6 or max_speed

        self.move_robot(self.linear, angular)
        self.get_logger().info("devant") # troubleshooting, announces state we are in

    # -------------------------
    # TURN: fixed rotation
    # -------------------------
    def turn_behavior(self, front,left,right):

        # apply fixed angular speed
        angular = 1.1 if self.turn_direction == "LEFT" else -1.1

        # no linear motion during turn
        self.move_robot(0.0, angular)
        self.get_logger().info("tourne")

        # detect when new corridor is visible
        if front > self.front_open_threshold:
            self.state = "REALIGN"

    # -------------------------
    # REALIGN: small correction
    # -------------------------
    def realign_behavior(self, front, left, right):
        # small proportional correction
        error = left - right
        angular = 0.15 * error

        # small forward motion while stabilizing
        self.move_robot(0.25, angular)
        self.get_logger().info("realigne")

        # when stable → forward
        if abs(error) < 0.15:
            self.state = "FORWARD"

        self.move_robot(self.linear,angular)

        # if the next turn is before the end of this state, allows robot to switch state
        if front < self.turn_trigger:
            self.state = "TURN"
            # turn toward the more open side
            self.turn_direction = "LEFT" if right > left else "RIGHT"
            return

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args) ## always first
    node = MySimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
