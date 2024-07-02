#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
import math
import time

class BugZero:
    def __init__(self):
        rospy.init_node('bug0')
        self.regions_ = None
        self.state_ = 0  # 0 - Normal operation, 1 - Wall following
        self.obstacle_detected_ = False
        self.rate = rospy.Rate(20)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        # self.get_cmd = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.cmd_clbk)
        self.position_ = {'x': 0, 'y': 0}
        self.goal_position_ = {'x': 16, 'y': 0}
        self.yaw = None
        
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)
        

        while not rospy.is_shutdown() and self.regions_ is None:
            self.rate.sleep()
        
        while not rospy.is_shutdown():
            if self.regions_ is None:
                continue
            
            if self.state_ == 0:
                self.check_for_obstacles()
            else:
                self.wall_following_logic()
                
            self.rate.sleep()
            
    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
            return angle
        else: return 1

    def clbk_laser(self, msg):
        self.regions_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }
        # rospy.loginfo(self.regions_['front'])
        
    def clbk_odom(self, msg):
        global position_, yaw_
        position_ = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        yaw_ = euler[2]
        self.yaw = yaw_


    def check_for_obstacles(self):
        if self.regions_['front'] <= 1:  # Adjust threshold as needed
            rospy.loginfo("Obstacle detected in front.")
            self.obstacle_detected_ = True
            self.switch_to_wall_following()
        else:
            # rospy.loginfo("Moving fwd")
            # Calculate desired yaw
            desired_yaw = math.atan2(self.goal_position_['y'] - self.position_['y'], self.goal_position_['x'] - self.position_['x'])
            # rospy.loginfo('des\w  ',desired_yaw)
            # # Normalize angle to [-pi, pi] range
            # rospy.loginfo('desired yaw   ',desired_yaw)
            desired_yaw = self.normalize_angle(desired_yaw)
            
            # Get current yaw
            current_yaw = self.yaw
            
            # Calculate error in yaw
            # rospy.loginfo(desired_yaw)
            # rospy.loginfo(current_yaw  )
            err_yaw = self.normalize_angle(desired_yaw - current_yaw)
            
            if abs(err_yaw) < 0.1:  # Threshold for being aligned with the goal
                rospy.loginfo("Heading towards the goal. Moving forward.")
                self.move_forward(0.5)  # Adjust speed as needed
            else:
                rospy.loginfo("Not heading towards the goal. Adjusting direction.")
                self.wall_following_logic()


    def is_aligned_with_goal(self):
        # Calculate the angle between the robot's current orientation and the goal direction
        goal_direction = math.atan2(self.goal_position_['y'] - self.position_['y'], self.goal_position_['x'] - self.position_['x'])
        current_direction = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # Assuming yaw is in radians
        angle_difference = abs(goal_direction - current_direction)

        # Normalize the angle difference to [-pi, pi] range
        normalized_angle_difference = self.normalize_angle(angle_difference)

        # Consider the robot aligned with the goal if the absolute difference is less than a small threshold
        return abs(normalized_angle_difference) < 0.1


    def wall_following_logic(self):
        rospy.loginfo("Wall following mode activated.")
        target_distance = 1.5  # Target distance from the wall
        turn_speed = 1  # Speed for turning, in m/s
        move_speed = 1  # Speed for moving forward, in m/s

        current_distance = self.regions_['front']

        # Check if the robot is aligned with the goal before deciding to turn
        if self.is_aligned_with_goal():
            rospy.loginfo("Aligned with the goal. Proceeding forward.")
            self.move_forward(move_speed)
        elif current_distance < target_distance:
            # Turn away from the wall
            self.turn_left(turn_speed)
        elif current_distance > target_distance:
            # Turn towards the wall
            self.turn_right(turn_speed)
        else:
            # Move forward if exactly at the target distance
            self.move_forward(move_speed)

    def turn_left(self, speed):
        rospy.loginfo('Turning left')
        twist = Twist()
        twist.angular.z = speed
        rate = rospy.Rate(10)
        start = time.time()
        current = time.time()
        

        while not rospy.is_shutdown() and (current - start) < 10.000000000000000:
            current = time.time()
            if self.is_aligned_with_goal():  # Check if aligned before continuing to turn
                break
            self.check_for_obstacles()
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def turn_right(self, speed):
        rospy.loginfo('turning right')
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = speed
        rate=rospy.Rate(10)
        start = time.time()
        current= time.time()
        # print(start)
        # print(current)
            
        while not rospy.is_shutdown() and (current - start)< 10.000000000000000:
            current = (time.time())
            # print((current - start))
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

    def move_forward(self, speed):
        rospy.loginfo('moving fwd')
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0
        rate=rospy.Rate(10)
        start = time.time()
        current= time.time()
        # print(start)
        # print(current)
            
        while not rospy.is_shutdown() and (current - start)< 10.000000000000000:
            current = (time.time())
            # print((current - start))
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

    def stop(self):
        rospy.loginfo('stopping ')
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        rate=rospy.Rate(10)
        start = time.time()
        current= time.time()
        # print(start)
        # print(current)
            
        while not rospy.is_shutdown() and (current - start)< 10.000000000000000:
            current = (time.time())
            print((current - start))
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()    

    def switch_to_normal_operation(self):
        rospy.loginfo("Switching back to normal operation.")
        self.state_ = 0
        self.obstacle_detected_ = False

    def switch_to_wall_following(self):
        rospy.loginfo("Switching to wall following mode.")
        self.state_ = 1

if __name__ == "__main__":
    BugZero()
