#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions
from .tf.transformations import euler_from_quaternion
from mpmath import *

class Position: 
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

class Charge:
    def __init__(self, x, y, intensity) -> None:
        self.position = Position(x, y)
        self.intensity = intensity

    def get_force_at_pos(self, x, y):

        vec = (self.position.x - x, self.position.y - x)
        vec_intensity = sqrt((vec[0] ** 2) + (vec[1] ** 2))
        vec_angle = atan(vec[1] / vec[0])

        r_vec = self.intensity / (vec_intensity ** 2)

        return r_vec * cos(vec_angle), r_vec * sin(vec_angle)

    def get_axis_force(self, charge_pos, pos):
        r = charge_pos - pos
        return self.intensity / (abs(r) * r)


class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        self.dist_to_finish = 0.2
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        

        self.positions = [((-0.15, 0.0, 1.57079632679), (0.15, 0.0, 1.57079632679), (0.0, 1.8, 0.0))]
        self.current_position = 0

        self.robot_pos = {
            'serp1': self.positions[self.current_position][0],
            'serp2': self.positions[self.current_position][1]
        }

        intensity = -1.0
        self.charges = [
            Charge(-0.42, -0.22, intensity), #edge
            Charge(0.0, -0.22, intensity),
            Charge(0.42, -0.22, intensity), #edge
            Charge(0.42, 0.3, intensity),
            Charge(0.42, 0.7, intensity),
            Charge(0.42, 1.1, intensity),
            Charge(0.42, 1.5, intensity),
            Charge(0.42, 2.01, intensity), #edge
            Charge(0.0, 2.01, intensity),
            Charge(-0.42, 2.01, intensity), #edge
            Charge(-0.42, 1.5, intensity),
            Charge(-0.42, 1.1, intensity),
            Charge(-0.42, 0.7, intensity),
            Charge(-0.42, 0.3, intensity)
        ]

        # **** Create publishers ****
        self.pub1:Publisher = self.create_publisher(Twist, "/cmd_vel1", 1)

        self.pub2:Publisher = self.create_publisher(Twist, "/cmd_vel2", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/end_beacon_laser", self.processEndLiDAR, 1)

        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)

        self.create_subscription(Odometry, "/odom1", lambda msg: self.updatePosition(msg, "serp1"), 1)

        self.create_subscription(Odometry, "/odom2", lambda msg: self.updatePosition(msg, "serp2"), 1)
        # ******************************

    # Change the speed of the robot
    def change_speed(self, publisher, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        publisher.publish(twist_msg)

    # Send a request to move a model
    def move_model(self, model_name, x, y, theta):
        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = model_name
        request.pose = Pose2D()
        request.pose.x = x
        request.pose.y = y
        request.pose.theta = theta
        client.call_async(request)
    
    # Handle LiDAR data
    def processEndLiDAR(self, data):
        clean_data = [x for x in data.ranges if str(x) != 'nan']
        if not clean_data: return
        if min(clean_data) <= self.dist_to_finish:
            self.reset()

        self.calculate_trajectory()
    
    # Process collisions
    def processCollisions(self, data):
        if len(data.collisions) > 0:
            self.reset()
    
    def updatePosition(self, data, model):
        rotation = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
        self.robot_pos[model] = (data.pose.pose.position.x, data.pose.pose.position.y, rotation)

    def reset(self):
        self.change_speed(self.pub1, 0.0, 0.0)
        self.change_speed(self.pub2, 0.0, 0.0)

        self.current_position = (self.current_position + 1) % len(self.positions)

        position = self.positions[self.current_position]
        self.move_model('serp1', position[0][0], position[0][1], position[0][2])
        self.move_model('serp2', position[1][0], position[1][1], position[1][2])
        self.move_model('end_beacon', position[2][0], position[2][1], position[2][2])

    def calculate_trajectory(self):
        intensity, angle = self.calculate_force()
        serp_rot = self.robot_pos['serp1'][2]

        linear_speed = 0.0
        angular_speed = 0.0
        
        ang_diff = angle - serp_rot
        if abs(ang_diff) < 0.02:
            linear_speed = min(intensity, self.max_linear_speed)
        else:
            angular_speed = min([ang_diff * 10, (self.max_angular_speed * sign(ang_diff))], key=abs)        

        self.change_speed(self.pub1, float(linear_speed), float(angular_speed))


    def calculate_force(self):
        x_force = 0
        y_force = 0
        self.get_logger().info('***********************************************')
        for charge in self.charges:
            x, y = charge.get_force_at_pos(self.robot_pos['serp1'][0], self.robot_pos['serp1'][1])
            x_force += x
            y_force += y
            self.get_logger().info(str(x_force) + '   ' + str(y_force))
        self.get_logger().info('++++++++++++++++++++++++++++++++++++++++++++++++')
        #end_position = self.positions[self.current_position][2]
        #end_charge = Charge(end_position[0], end_position[1], 5.0)
        #x, y = end_charge.get_force_at_pos(self.robot_pos['serp1'][0], self.robot_pos['serp1'][1])
        #x_force += x
        #y_force += y
        
        intensity = sqrt((x_force ** 2) + (y_force ** 2))
        angle = atan(y_force / x_force)

        return intensity, angle

def main(args = None):
    rclpy.init()
    
    serp = SerpController()

    rclpy.spin(serp)

if __name__ == "__main__":
    main()
