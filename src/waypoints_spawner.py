#!/usr/bin/env python3

import rospy
import csv
import os
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

def read_waypoints_from_csv(csv_file_path):
    waypoints = []
    with open(csv_file_path, mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            waypoint = {
                'name': row['name'],
                'position_x': float(row['position_x']),
                'position_y': float(row['position_y']),
                'position_z': float(row['position_z']),
                'orientation_x': float(row['orientation_x']),
                'orientation_y': float(row['orientation_y']),
                'orientation_z': float(row['orientation_z']),
                'orientation_w': float(row['orientation_w']),
            }
            waypoints.append(waypoint)
    return waypoints

def waypoint_spawner(goal):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    movebase_goal = MoveBaseGoal()
    # Header
    movebase_goal.target_pose.header.stamp = rospy.Time.now()
    movebase_goal.target_pose.header.frame_id = "map"
    # Position
    movebase_goal.target_pose.pose.position.x = goal['position_x']
    movebase_goal.target_pose.pose.position.y = goal['position_y']
    movebase_goal.target_pose.pose.position.z = goal['position_z']
    # Orientation
    movebase_goal.target_pose.pose.orientation.x = goal['orientation_x']
    movebase_goal.target_pose.pose.orientation.y = goal['orientation_y']
    movebase_goal.target_pose.pose.orientation.z = goal['orientation_z']
    movebase_goal.target_pose.pose.orientation.w = goal['orientation_w']

    client.send_goal(movebase_goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_spawner')

        csv_file_path = os.path.join(os.getenv('HOME'), 'catkin_ws/src/new_description/src/waypoints.csv')
        goal_list = read_waypoints_from_csv(csv_file_path)

        rospy.loginfo(f"Waypoint list: {goal_list}")

        while True:
            print("\nGOAL LIST:")
            for idx, goal in enumerate(goal_list):
                print(f"{idx}) {goal['name']}")

            value = input("\nInsert goal index: ")

            if value.isdigit():
                value_parsed = int(value)

            while not(value.isdigit() and value_parsed in range(len(goal_list))):
                print(f"Error: the input must be an integer between 0 and {len(goal_list) - 1}")
                value = input("Insert goal index: ")

                if value.isdigit():
                    value_parsed = int(value)

            goal = goal_list[value_parsed]
            result = waypoint_spawner(goal)

            if result:
                rospy.loginfo(f"\n({goal['name']} x: {goal['position_x']}, y: {goal['position_y']}) Goal execution done!\n")

    except rospy.ROSInterruptException:
        pass
