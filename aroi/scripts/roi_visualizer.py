##################################  VISUALIZE ALL THE RoIs (F,B,R,L,ES)  ###################################


import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point, PoseStamped

current_z = 0

rospy.init_node("roi_visualizer", anonymous=True)

roi_main_marker_pub = rospy.Publisher("roi_main_markers", Marker, queue_size=100)
roi_right_marker_pub = rospy.Publisher("roi_right_markers", Marker, queue_size=100)
roi_left_marker_pub = rospy.Publisher("roi_left_markers", Marker, queue_size=100)
roi_back_marker_pub = rospy.Publisher("roi_back_markers", Marker, queue_size=100)
roi_es_marker_pub = rospy.Publisher("roi_es_markers", Marker, queue_size=100)


def callback_ndt_pose(data):
    
    global current_z
    current_z = data.pose.position.z


def callback_roi_main(data):

    marker = Marker()
    marker.header.frame_id = "map"  
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_main_triangles"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.a = 0.5  # Transparency
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration(1.0)  # Marker lifetime

    points = data.data  # List of points from the Float32MultiArray
    if len(points) % 8 != 0:
        rospy.logwarn("Received malformed rectangle data. Ignoring.")
        return

    num_rectangles = len(points) // 8

    for i in range(num_rectangles):
        p1 = Point(x=points[i * 8 + 0], y=points[i * 8 + 1], z=current_z-2)
        p2 = Point(x=points[i * 8 + 2], y=points[i * 8 + 3], z=current_z-2)
        p3 = Point(x=points[i * 8 + 4], y=points[i * 8 + 5], z=current_z-2)
        p4 = Point(x=points[i * 8 + 6], y=points[i * 8 + 7], z=current_z-2)

        marker.points.extend([p1, p2, p3, p1, p3, p4])
        #marker.points.extend([p1, p3, p4])

    roi_main_marker_pub.publish(marker)


def callback_roi_right(data):

    marker = Marker()
    marker.header.frame_id = "map"  
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_right_lines"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1  # Line width
    marker.color.a = 0.8  # Transparency
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration(1.0)  # Marker lifetime

    points = data.data  
    if len(points) % 8 != 0:
        rospy.logwarn("Received malformed rectangle data. Ignoring.")
        return

    num_rectangles = len(points) // 8

    for i in range(num_rectangles):
        # Extract the four corners of the rectangle
        p1 = Point(x=points[i * 8 + 0], y=points[i * 8 + 1], z=current_z-2)
        p2 = Point(x=points[i * 8 + 2], y=points[i * 8 + 3], z=current_z-2)
        p3 = Point(x=points[i * 8 + 4], y=points[i * 8 + 5], z=current_z-2)
        p4 = Point(x=points[i * 8 + 6], y=points[i * 8 + 7], z=current_z-2)

        marker.points.extend([p1, p2])  # Line between p1 and p2

    roi_right_marker_pub.publish(marker)


def callback_roi_left(data):
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_left_lines"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1  # Line width
    marker.color.a = 0.8  # Full opacity
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration(1.0)  # Marker lifetime

    points = data.data
    if len(points) % 8 != 0:
        rospy.logwarn("Received malformed rectangle data. Ignoring.")
        return

    num_rectangles = len(points) // 8

    for i in range(num_rectangles):
        # Extract the four corners of the rectangle
        p1 = Point(x=points[i * 8 + 0], y=points[i * 8 + 1], z=current_z-2)
        p2 = Point(x=points[i * 8 + 2], y=points[i * 8 + 3], z=current_z-2)
        p3 = Point(x=points[i * 8 + 4], y=points[i * 8 + 5], z=current_z-2)
        p4 = Point(x=points[i * 8 + 6], y=points[i * 8 + 7], z=current_z-2)

        marker.points.extend([p3, p4])  # Line between p3 and p4

    roi_left_marker_pub.publish(marker)


def callback_roi_back(data):
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_back_triangles"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.a = 0.3  # Transparency
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.3
    marker.lifetime = rospy.Duration(1.0)  # Marker lifetime

    points = data.data
    if len(points) % 8 != 0:
        rospy.logwarn("Received malformed rectangle data. Ignoring.")
        return

    num_rectangles = len(points) // 8

    for i in range(num_rectangles):
        p1 = Point(x=points[i * 8 + 0], y=points[i * 8 + 1], z=current_z-2)
        p2 = Point(x=points[i * 8 + 2], y=points[i * 8 + 3], z=current_z-2)
        p3 = Point(x=points[i * 8 + 4], y=points[i * 8 + 5], z=current_z-2)
        p4 = Point(x=points[i * 8 + 6], y=points[i * 8 + 7], z=current_z-2)

        marker.points.extend([p1, p2, p3])
        marker.points.extend([p1, p3, p4])

    roi_back_marker_pub.publish(marker)


def callback_roi_es(data):

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_es_lines"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2  # Line width
    marker.color.a = 0.8  # Transparency
    marker.color.r = 1.0
    marker.color.g = 0.3
    marker.color.b = 0.2
    marker.lifetime = rospy.Duration(1.0)  # Marker lifetime

    points = data.data
    if len(points) != 8:
        rospy.logwarn("Received malformed rectangle data. Ignoring.")
        return

    # Define the four corners of the rectangle
    p1 = Point(x=points[0], y=points[1], z=current_z-1.9)
    p2 = Point(x=points[2], y=points[3], z=current_z-1.9)
    p3 = Point(x=points[4], y=points[5], z=current_z-1.9)
    p4 = Point(x=points[6], y=points[7], z=current_z-1.9)

    marker.points.extend([p1, p4])
    marker.points.extend([p4, p3])
    marker.points.extend([p3, p2])
    marker.points.extend([p2, p1])

    roi_es_marker_pub.publish(marker)


rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
rospy.Subscriber("/roi_main_array", Float32MultiArray, callback_roi_main)
rospy.Subscriber("/roi_right_array", Float32MultiArray, callback_roi_right)
rospy.Subscriber("/roi_left_array", Float32MultiArray, callback_roi_left)
rospy.Subscriber("/roi_back_array", Float32MultiArray, callback_roi_back)
rospy.Subscriber("/roi_es_coords", Float32MultiArray, callback_roi_es)

rospy.loginfo("ROI_visualizer node is running successfully!")

rospy.spin()


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

'''
Rakshith Ram [30-1-25]
'''