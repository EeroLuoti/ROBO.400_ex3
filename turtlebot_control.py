#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos

'''
asks goal position as input from
user, then moves turtlebot to that pose
'''

class TurtleBot3Controller(Node):

    MAX_LINEAR = 0.2
    MAX_ANGULAR = 1.5

    def __init__(self):
        super().__init__('turtlebot3_controller')


        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        self.pose_subscriber = self.create_subscription(
            Odometry, '/odom', self.update_pose, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_received = False

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.distance_tolerance = 0.05
        self.goal_reached = False
        self.timer = None

    def update_pose(self, msg):
        #update robot's x, y, and theta from /odom 
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = round(p.x, 4)
        self.y = round(p.y, 4)

        self.theta = atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose_received = True

    def euclidean_distance(self):
        #distance between the robot and the goal
        return sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)

    def angle_error(self):
        #difference between robot and goal angle
        steering_angle = atan2(self.goal_y - self.y, self.goal_x - self.x)
        error = steering_angle - self.theta
        return atan2(sin(error), cos(error))

    def control_loop(self):
    	#runs the control loop 
        distance = self.euclidean_distance()

        if distance >= self.distance_tolerance:
            error = self.angle_error()
            vel_msg = Twist()

            # If the robot is pointing far away from the goal, turn in place
            if abs(error) > 0.5:
                vel_msg.linear.x = 0.0
            else:
                vel_msg.linear.x = min(self.MAX_LINEAR, 1.5 * distance)

            vel_msg.angular.z = min(self.MAX_ANGULAR, 2.0 * error)

            self.velocity_publisher.publish(vel_msg)
        else:
        #stop robot when goal reached
            self.stop()
            self.timer.cancel()
            self.goal_reached = True
            self.get_logger().info('Goal reached!')

    def stop(self):
        #send zero velocity command to robot
        self.velocity_publisher.publish(Twist())

    def move2goal(self):
        #ask goal position from user and start control loop
        self.get_logger().info('Waiting for /odom ...')
        while rclpy.ok() and not self.pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            return

        self.get_logger().info(
            f'Current position: x={self.x:.2f}, y={self.y:.2f} '
            f'(odom frame, origin = robot start point)')

        self.goal_x = float(input("Set your x goal: "))
        self.goal_y = float(input("Set your y goal: "))
        self.distance_tolerance = float(
            input("Set your tolerance (e.g. 0.05): "))

        self.timer = self.create_timer(0.1, self.control_loop)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Controller()

    try:
        node.move2goal()
        while rclpy.ok() and not node.goal_reached:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
