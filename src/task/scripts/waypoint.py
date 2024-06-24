#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def read_waypoints(filename):
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            x, y, yaw = map(float, line.split())
            waypoints.append((x, y, yaw))
    return waypoints

def send_goal(client, x, y, yaw):
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()

    # Set the goal
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, yaw, 1.0))

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()

def main():
    rospy.init_node('send_goal_from_file')

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    waypoints = read_waypoints('/home/kompor/movel_task_ws/src/task/scripts/waypoint.txt')

    for x, y, yaw in waypoints:
        result = send_goal(client, x, y, yaw)
        if result:
            rospy.loginfo(f"Goal at ({x}, {y}, {yaw}) execution done!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")