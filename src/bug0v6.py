#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
import math
import csv
import time

class Bug0Robot:
    class State:
        FORWARD = "FORWARD"
        TURN_LEFT = "TURN_LEFT"
        TURN_RIGHT = "TURN_RIGHT"
        ALIGN_GOAL = "ALIGN_GOAL"

    def __init__(self):
        rospy.init_node('bug0_robot')
        self.state = self.State.FORWARD
        self.goal_reached = False
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.get_cmd_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.get_cmd = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cmd_clbk)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10Hz
        self.goal_received = False
        self.yaw = 0
        self.position_ = {'x':0,'y':0}
        self.goal_points = self.load_goal_points('/home/swagatika/catkin_ws/src/new_description/src/waypoints.csv')
        self.selected_goal_index = None
        self.prompt_user_for_goal()

        self.regions_ = {
            'right':  float('inf'),
            'fright': float('inf'),
            'front':  float('inf'),
            'fleft':  float('inf'),
            'left':   float('inf'),
        }
        self.current_action = None

    def scan_callback(self, msg):
        self.regions_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }

    def cmd_clbk(self, msg):
        self.goal_position_ = msg.pose.position
        self.goal_received = True
        rospy.loginfo("Received goal at x: {}, y: {}".format(self.goal_position_.x, self.goal_position_.y))

    def odom_callback(self, msg):
        self.position_ = msg.pose.pose.position
        self.yaw = self.calculate_yaw(msg.pose.pose.orientation)
        
    def prompt_user_for_goal(self):
        print("Please enter the number of the goal you wish to navigate to:")
        for index, goal in enumerate(self.goal_points, start=1):
            print(f"{index}. ({goal['x']}, {goal['y']})")
        user_choice = input("Enter choice (1 or 2): ")
        self.selected_goal_index = int(user_choice) - 1  # Adjust index based on zero-based list
        self.publish_selected_goal()
    
    @staticmethod
    def calculate_yaw(orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = transformations.euler_from_quaternion(q)
        return yaw

    def move_forward(self, speed=0.5):
        rospy.loginfo("Moving Forward")
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0
        self.cmd_vel_pub.publish(twist_msg)

    def turn_left(self, speed=0.5):
        rospy.loginfo("Turning Left")
        twist = Twist()
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)

    def turn_right(self, speed=0.5):
        rospy.loginfo("Turning Right")
        twist_msg = Twist()
        twist_msg.angular.z = -speed
        self.cmd_vel_pub.publish(twist_msg)

    def bug0_logic(self):
        if self.state == self.State.FORWARD:
            self.forward_state_logic()
        elif self.state == self.State.TURN_LEFT:
            self.turn_left_state_logic()
        elif self.state == self.State.TURN_RIGHT:
            self.turn_right_state_logic()
        elif self.state == self.State.ALIGN_GOAL:
            self.align_goal_state_logic()

    def forward_state_logic(self):
        if self.regions_['front'] > 1.0:
            self.move_forward()
        else:
            self.change_state(self.State.TURN_LEFT)

    def turn_left_state_logic(self):
        self.turn_left()
        self.change_state(self.State.FORWARD)

    def turn_right_state_logic(self):
        self.turn_right()
        self.change_state(self.State.FORWARD)

    def align_goal_state_logic(self):
        # Calculate the direction vector from the robot to the goal
        dx = self.goal_position_.x - self.position_.x
        dy = self.goal_position_.y - self.position_.y
        
        # Normalize the direction vector
        direction_vector = math.sqrt(dx**2 + dy**2)
        
        # Calculate the angle between the robot's current direction and the direction to the goal
        theta = math.atan2(dy, dx) - self.yaw
        
        # Ensure the angle is within [-pi, pi] range
        if abs(theta) > math.pi:
            theta -= math.copysign(math.pi, theta)
        
        # Calculate the rotation speed needed to align with the goal
        rotation_speed = max(-0.5, min(0.5, theta))
        
        # Rotate the robot towards the goal
        self.turn_right(speed=rotation_speed)



    def change_state(self, new_state):
        self.state = new_state
        rospy.loginfo(f"Changing state to {self.state}")

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x- point2.x)**2 + (point1.y - point2.y)**2)

    def load_goal_points(self,csv_file_path):
        goal_points = []
        with open(csv_file_path, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for row in reader:
                goal_point = {'x': float(row[1]), 'y': float(row[2])}
                goal_points.append(goal_point)
        return goal_points

    def publish_selected_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "/map"  # Adjust frame_id as per your setup
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = self.goal_points[self.selected_goal_index]['x']
        goal_msg.pose.position.y = self.goal_points[self.selected_goal_index]['y']
        goal_msg.pose.orientation.w = 1.0  # Simple straight-line motion
        self.get_cmd_pub.publish(goal_msg)
        rospy.loginfo(f"Published goal at x: {goal_msg.pose.position.x}, y: {goal_msg.pose.position.y}")


    def run(self):
        self.start_time = time.time()
        while not rospy.is_shutdown() and not self.goal_received:
            elapsed_time = time.time() - self.start_time
            if elapsed_time > 60:  # Timeout after 60 seconds
                rospy.logwarn("Timeout waiting for goal. Exiting.")
                break
            self.rate.sleep()

        while not rospy.is_shutdown():
            self.bug0_logic()
            
            # Check if the robot has reached the goal
            if self.goal_received and self.calculate_distance(self.position_, self.goal_position_) < 1:
                rospy.loginfo("Goal reached!")
                self.goal_reached = True
                self.cmd_vel_pub.publish(Twist())
                
                break
            
            self.rate.sleep()


if __name__ == "__main__":
    bug_zero = Bug0Robot()
    try:
        bug_zero.run()
    except rospy.ROSInterruptException:
        pass
