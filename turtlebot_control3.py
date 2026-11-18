#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos, pi



class TurtleBot3Controller(Node):

    MAX_LINEAR = 0.2
    MAX_ANGULAR = 1.5
    SIDE_LENGTH = 1.0

    def __init__(self):
        super().__init__('turtlebot3_controller')


        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_theta = None
        self.turned = 0.0
        self.goal_reached = False

        self.state = 'forward'
        self.sides_done = 0
        self.start_x = None
        self.start_y = None
        self.turn_start = 0.0

    def circle(self):
        #robot does a circle
        angle = self.turned
        vel_msg = Twist()

        #if robot hasnt rotated full circle, move robot in circle
        if angle <= 2*pi:
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.5
            self.velocity_publisher.publish(vel_msg)
        #robot has done full circle -> stop    
        else:
            self.velocity_publisher.publish(Twist())
            self.goal_reached = True

    def square(self):
        #robot does a square
        vel_msg = Twist()

        #if all sides are done -> stop
        if self.sides_done >= 4:
            self.velocity_publisher.publish(Twist())
            self.goal_reached = True
            return


        if self.state == 'forward':
            if self.start_x is None:
                self.start_x = self.x
                self.start_y = self.y

            dist = sqrt((self.x - self.start_x)**2 + (self.y - self.start_y)**2)

            if dist < self.SIDE_LENGTH:
                vel_msg.linear.x = 0.2
            else:
                self.state = 'turn'
                self.turn_start = self.turned

        elif self.state == 'turn':
            angle = self.turned - self.turn_start

            if angle < pi/2:
                vel_msg.angular.z = 0.5
            else:
                self.sides_done += 1
                self.state = 'forward'
                self.start_x = None

        self.velocity_publisher.publish(vel_msg)

    def update_pose(self, msg):
        #updates robot pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.last_theta is not None:
            d = self.theta - self.last_theta
            self.turned += atan2(sin(d), cos(d))
        self.last_theta = self.theta
        return self.turned


def main(args=None):
    #asks for user input and ŕuns control loop
    rclpy.init(args=args)
    node = TurtleBot3Controller()
    shape = input("Do you want circle(c) or square(s): ")
    if shape in ("c", "s"):
        try:
            while rclpy.ok() and not node.goal_reached:
                if shape == "c":
                    node.circle()
                else:
                    node.square()
                rclpy.spin_once(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        print("incorrect input")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()