#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions
from tf_transformations import euler_from_quaternion 
from .charges.charges import *
from mpmath import *

class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        # Max distance from left robot to end beacon to finish
        self.dist_to_finish = 0.2

        # Speed limits
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0

        # Ideal distance between the two robots
        self.robot_distance = 0.3

        # All possible starting positions for all models (serp1, serp2, end_beacon)
        self.positions = [
            (Position(Point(-0.15, 0.0), 1.57079632679), Position(Point(0.15, 0.0), 1.57079632679), Position(Point(1.6, 1.6), 0.0)),
            (Position(Point(1.6, 1.45), 3.14159265359), Position(Point(1.6, 1.75), 3.14159265359), Position(Point(0.0, 0.0), 0.0))
        ]
        self.current_position = -1

        # To store the robots positions at all times
        self.robot_pos = {
            'serp1': self.positions[0][0],
            'serp2': self.positions[0][1]
        }

        # Charge layout of the map for Potential Fields algorithm
        intensity = -1.0
        self.charges = [
            LineSegmentCharge(Point(-0.42, -0.22), Point(0.42, -0.22), intensity),
            CompositCharge(
                [
                    LineSegmentCharge(Point(0.42, -0.22), Point(0.42, 1.22), intensity),
                    LineSegmentCharge(Point(0.42, 1.22), Point(1.82, 1.22), intensity)
                ],
                PointCharge(Point(0.42, 1.22), intensity),
                intensity
            ),
            LineSegmentCharge(Point(1.82, 1.22), Point(1.82, 2.01), intensity),
            LineSegmentCharge(Point(1.82, 2.01), Point(-0.42, 2.01), intensity),
            LineSegmentCharge(Point(-0.42, 2.01), Point(-0.42, -0.22), intensity)
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

        self.position_readings = 100
        self.reset()

    # Change the speed of the robot
    def change_robot_speeds(self, publisher, linear, angular):
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
        if clean_data:
            robot_dist = (self.robot_pos['serp1'].point - self.robot_pos['serp2'].point).magnitude()
            # If the left robot is close enough to the end and the right robot close enough to the left, finish successfuly 
            if min(clean_data) <= self.dist_to_finish and robot_dist < 0.5:
                self.reset()
    
    # Process collisions
    def processCollisions(self, data):
        # If a collision happens, finish unsuccessfuly 
        if len(data.collisions) > 0:
            self.reset()
    
    # Receive most recent position for robot of name <model> and store it
    def updatePosition(self, data, model):
        rotation = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
        self.robot_pos[model] = Position(Point(data.pose.pose.position.x, data.pose.pose.position.y), (rotation + (2*pi)) % (2*pi))

        # If the most recent position readings for both robots have arrived (even number of readigs), update their trajectories based on Potential Fields
        self.position_readings += 1
        if self.position_readings % 2 == 0:
            self.update_robot_speeds()

    # Reset the map by putting all the models in the new initial position
    def reset(self):
        if self.position_readings < 20: return

        self.change_robot_speeds(self.pub1, 0.0, 0.0)
        self.change_robot_speeds(self.pub2, 0.0, 0.0)

        self.position_readings = 0

        self.current_position = (self.current_position + 1) % len(self.positions)

        position = self.positions[self.current_position]
        self.move_model('serp1', position[0].point.x, position[0].point.y, position[0].rot)
        self.move_model('serp2', position[1].point.x, position[1].point.y, position[1].rot)
        self.move_model('end_beacon', position[2].point.x, position[2].point.y, position[2].rot)

    # Update the robots trajectories using the Potential Fields algorithm
    def update_robot_speeds(self):
        
        # Charges to avoid the robots colliding with each other ****************************************************************
        # Left robot charge that affects the right robot
        left_robot_charge = PointCharge(self.robot_pos['serp1'].point, -1.0)

        # Right robot charge that affects the left robot
        right_robot_charge = PointCharge(self.robot_pos['serp2'].point, -1.0)
        # **********************************************************************************************************************


        # Find the new target middle point for the robots **********************************************************************
        # Find the middle point between the two robots
        middle_point = self.robot_pos['serp1'].point.add_vector((self.robot_pos['serp2'].point - self.robot_pos['serp1'].point) / 2)

        # Add a positive charge in the end beacon position to attract the fobots to the end
        target_charge = PointCharge(self.positions[self.current_position][2].point, 15.0)
        
        # Get the force vector with the map and target charges. Intensity capped by max_linear_speed
        intensity, angle = self.calculate_force(middle_point, [target_charge])
        shift = min(intensity, self.max_linear_speed)

        # Add the force vector to the middle point to find the target
        target_middle_point = middle_point.add_vector(Vector(shift * cos(angle), shift * sin(angle)))
        # **********************************************************************************************************************


        # Ideal distance from the middle point to each robot
        dist_to_middle = self.robot_distance / 2


        # Find new speeds for the left robot ***********************************************************************************
        # Fint the target point for the left robot and create a tension point
        left_target_angle = angle + (pi/2)
        left_target_point = target_middle_point.add_vector(Vector(dist_to_middle * cos(left_target_angle), dist_to_middle * sin(left_target_angle)))
        left_target_tension = PointTension(left_target_point, 100.0)

        # Use the map and right robot charges and the target tension point to determine the force and with the force determine speeds
        left_intensity, left_angle = self.calculate_force(self.robot_pos['serp1'].point, [left_target_tension, right_robot_charge])
        left_linear_speed, left_angular_speed = self.calculate_speeds(left_angle, self.robot_pos['serp1'].rot)
        # **********************************************************************************************************************


        # Find new speeds for the right robot ***********************************************************************************
        # Fint the target point for the right robot and create a tension point
        right_target_angle = angle - (pi/2)
        right_target_point = target_middle_point.add_vector(Vector(dist_to_middle * cos(right_target_angle), dist_to_middle * sin(right_target_angle)))
        right_target_tension = PointTension(right_target_point, 100.0)

        # Use the map and left robot charges and the target tension point to determine the force and with the force determine speeds
        right_intensity, right_angle = self.calculate_force(self.robot_pos['serp2'].point, [right_target_tension, left_robot_charge])
        right_linear_speed, right_angular_speed = self.calculate_speeds(right_angle, self.robot_pos['serp2'].rot)
        # **********************************************************************************************************************

        
        # Robots only move forward when both are facing the the right way
        if left_linear_speed == 0 or right_linear_speed == 0: left_linear_speed = right_linear_speed = 0.0
        # To make sure both robots stay side by side, the robot with more force intensity moves at the maximum linear speed
        # The others linear speed is scaled down based on both intensities
        else:
            if left_intensity > right_intensity: right_linear_speed *= right_intensity / left_intensity
            else: left_linear_speed *= left_intensity / right_intensity

        # Apply new speeds
        self.change_robot_speeds(self.pub1, float(left_linear_speed), float(left_angular_speed))
        self.change_robot_speeds(self.pub2, float(right_linear_speed), float(right_angular_speed))

    # Determine the movement for the robot, if facing the right way: move forward, if not: rotate
    def calculate_speeds(self, angle, robot_rot):
        # Diference of the angle force to the direction the robot is facing. Minimum of positive and negative rotation
        ang_diff = angle - robot_rot
        if abs(ang_diff) > pi: ang_diff = ang_diff - ((2*pi) * sign(ang_diff))

        linear_speed = 0.0
        angular_speed = 0.0

        if abs(ang_diff) < 0.02:
            linear_speed = self.max_linear_speed
        else:
            # Rotate at maximum angular speed unless the diference is to small
            angular_speed = min([ang_diff * 5, (self.max_angular_speed * sign(ang_diff))], key=abs)       

        return linear_speed, angular_speed

    # Calculate the force on a point in space from map charges + list of extra chages
    def calculate_force(self, point, extra_charges):

        # Determine x and y components of sum of all forces from charges *****
        x_force = 0
        y_force = 0
        for charge in self.charges:
            force = charge.get_force_at_point(point)
            if force != None:
                x_force += force.x
                y_force += force.y

        for charge in extra_charges:
            force = charge.get_force_at_point(point)
            if force != None:
                x_force += force.x
                y_force += force.y
        # *********************************************************************
        
        # Determine intensity and angle of the force
        intensity = sqrt((x_force ** 2) + (y_force ** 2))
        angle = (atan2(y_force, x_force) + (2*pi)) % (2*pi)

        return intensity, angle
    
def main(args = None):
    rclpy.init()
    
    serp = SerpController()

    rclpy.spin(serp)

if __name__ == "__main__":
    main()
