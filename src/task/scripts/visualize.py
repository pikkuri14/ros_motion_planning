#!/usr/bin/env python

import rospy
import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from shapely.geometry import LineString

def read_path_from_rosbag(bag_file):
    poses = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, _ in bag.read_messages():
            if topic == '/vslam2d_pose':  # adjust topic based on your rosbag data
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = msg.header.stamp
                pose_stamped.header.frame_id = msg.header.frame_id
                pose_stamped.pose = msg.pose
                poses.append(pose_stamped)
    return poses

def simplify_path_to_n_poses(poses, n):
    if len(poses) <= n:
        return poses
    
    # Extract (x, y) coordinates from PoseStamped messages
    xy_points = [(pose.pose.position.x, pose.pose.position.y) for pose in poses]
    
    # Compute the interval length for downsampling
    interval = len(xy_points) / n
    
    # Simplify path by downsampling
    simplified_poses = []
    for i in range(n):
        index = int(i * interval)
        simplified_poses.append(poses[index])
    
    return simplified_poses

def main():
    rospy.init_node('path_simplification_node')
    rate = rospy.Rate(10)  # 10 Hz
    
    bag_file = './src/task/scripts/path_test.bag'
    raw_path_pub = rospy.Publisher('/raw_path', Path, queue_size=10)
    simplified_path_pub = rospy.Publisher('/simplified_path', Path, queue_size=10)
    
    # Example: Simplify path to exactly 100 poses
    N = 100
    
    while not rospy.is_shutdown():
        raw_path_msg = Path()
        simplified_path_msg = Path()
        
        # Read path from rosbag
        raw_poses = read_path_from_rosbag(bag_file)
        raw_path_msg.header.stamp = rospy.Time.now()
        raw_path_msg.header.frame_id = "map"  # adjust frame_id as needed
        raw_path_msg.poses = raw_poses
        
        # Simplify path to N poses
        simplified_poses = simplify_path_to_n_poses(raw_poses, N)
        simplified_path_msg.header.stamp = rospy.Time.now()
        simplified_path_msg.header.frame_id = "map"  # adjust frame_id as needed
        simplified_path_msg.poses = simplified_poses
        
        # Publish messages
        raw_path_pub.publish(raw_path_msg)
        simplified_path_pub.publish(simplified_path_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
