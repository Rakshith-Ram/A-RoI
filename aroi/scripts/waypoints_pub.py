#################################  PUBLISH WAYPOINTS FOR FRONT, BACK RoIs  #################################


import rospy
import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool, Float32
from nav_msgs.msg import Path
from scipy.spatial import KDTree


# Global variables
current_x, current_y, current_z = 0,0,0
waypoints_front = []
waypoints_back = []

# Waypoints
file_path = "/home/rakshithram/roi_1703/scripts/new_waypoints.txt"

try:
    with open(file_path, 'r') as file:
        file_contents = file.read()
except FileNotFoundError:
    rospy.logerr("File not found.")
    exit()
except Exception as e:
    rospy.logerr(f"An error occurred: {e}")
    exit()

waypoints_data = file_contents.strip().split('\n')

waypoints = []
for line in waypoints_data:
    x, y = map(float, line.strip("[],").split(','))
    waypoints.append([x, y])

# Build KDTree for fast nearest neighbor search
waypoint_tree = KDTree(waypoints)


def callback_ndt_pose(data):

    global current_x
    global current_y
    global current_z

    current_x = data.pose.position.x
    current_y = data.pose.position.y
    current_z = data.pose.position.z


def find_nearest_waypoint_index():

    if waypoint_tree is None:
        rospy.logwarn("Waypoint KDTree is not initialized.")
        return -1

    distance_threshold = 2.0

    distance, index = waypoint_tree.query([current_x, current_y]) 

    if distance > distance_threshold:
        rospy.logwarn("No valid waypoint found within threshold.")
        return -1

    return index



# Parameters for the sliding window
front_window_size = 30
back_window_size = 30

front_skip = 2
back_skip = 4


# Initialize ROS node
rospy.init_node('waypoint_publisher')
rospy.loginfo("Waypoint publisher node is running successfully!")

# Publishers
f_wp_publisher = rospy.Publisher('/waypoints_front', Float32MultiArray, queue_size=10)
b_wp_publisher = rospy.Publisher('/waypoints_back', Float32MultiArray, queue_size=10)
wp_full_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10)


# Subscriber
rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)


rate = rospy.Rate(10)  # 10Hz

while not rospy.is_shutdown():

    waypoint_idx = find_nearest_waypoint_index()

    if waypoint_idx == -1:
        rospy.logwarn("No valid waypoint found.")
        rate.sleep()
        continue
    
    # Adjusting front and back windows based on current waypoint index
    
    front_start_idx = max(0, waypoint_idx + front_skip)
    front_end_idx = min(len(waypoints), front_start_idx + front_window_size)
    front_window = waypoints[front_start_idx:front_end_idx]
    
    back_start_idx = max(0, waypoint_idx - back_skip)
    back_end_idx = max(0, back_start_idx - back_window_size)
    back_window = waypoints[back_end_idx:back_start_idx] 

    waypoints_front = []
    waypoints_back = []
    
    # Publish front waypoints
    for wp in front_window:
        waypoints_front.append(wp[0])
        waypoints_front.append(wp[1])

    f_wp_msg = Float32MultiArray(data=waypoints_front)
    f_wp_publisher.publish(f_wp_msg)

    # Publish back waypoints
    for wp in back_window:
        waypoints_back.append(wp[0])
        waypoints_back.append(wp[1])
    
    b_wp_msg = Float32MultiArray(data=waypoints_back)
    b_wp_publisher.publish(b_wp_msg)
    

    marker_array = MarkerArray()

    for i in range(0, len(waypoints)-1):
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = i # Unique ID for each marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = waypoints[i][0]
        marker.pose.position.y = waypoints[i][1]
        marker.pose.position.z = current_z - 2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Small sphere radius
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.8
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.7
        marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(marker)
    
    wp_full_pub.publish(marker_array)

    rate.sleep()


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

'''
Rakshith Ram [17-3-25]
'''