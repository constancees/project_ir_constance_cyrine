import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MySimulationNode(Node):

    def __init__(self):
        super().__init__("simulation_node")

        self.subscriber_ = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # States: "FORWARD", "TURN", "REALIGN"
        self.state = "FORWARD"
        self.turn_direction = None   # "LEFT" or "RIGHT"

        # Gains
        self.Kp = 0.55
        self.max_speed = 0.8
        self.max_turn_speed = 0.6

        # thresholds
        self.turn_trigger = 1.0       # when to start turning
        self.front_open_threshold = 1.8   # when new corridor is visible

    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        front = ranges[n//2]
        left  = ranges[int(n*3/4)]
        right = ranges[int(n*1/4)]

        if self.state == "FORWARD":
            self.forward_behavior(front, left, right)
        elif self.state == "TURN":
            self.turn_behavior(front)
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

        # Centering controller
        error = left - right
        angular = self.Kp * error

        # limit angular velocity
        angular = max(min(angular, self.max_turn_speed), -self.max_turn_speed)

        # adjust speed
        linear = self.max_speed - abs(angular) * 0.2
        linear = max(0.2, linear)

        self.move_robot(linear, angular)

    # -------------------------
    # TURN: fixed rotation
    # -------------------------
    def turn_behavior(self, front):

        # Apply fixed angular speed
        angular = 0.6 if self.turn_direction == "LEFT" else -0.6

        # No linear motion during turn
        self.move_robot(0.0, angular)

        # Detect when new corridor is visible
        if front > self.front_open_threshold:
            self.state = "REALIGN"

    # -------------------------
    # REALIGN: small correction
    # -------------------------
    def realign_behavior(self, front, left, right):
        # small proportional correction
        error = left - right
        angular = 0.3 * error

        # small forward motion while stabilizing
        self.move_robot(0.4, angular)

        # when stable → forward
        if abs(error) < 0.1:
            self.state = "FORWARD"

    # -------------------------
    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MySimulationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
