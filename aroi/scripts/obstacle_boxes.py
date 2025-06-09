###########################  Subscribe to JSK boxes and publishes 2D PCA corners  ############################


import rospy
import tf.transformations
import numpy as np
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped

# Publishers/Global variables
pub_bbox_2d = None
pub_markers = None
last_marker_count = 0
current_z = 0 


def callback_ndt_pose(data):

    global current_z
    current_z = data.pose.position.z


def compute_2d_corners(position, orientation, dimensions):

    """Compute the 4 corner points of the bounding box in 2D (x, y)"""

    # Neglecting dimesion in Z
    dx, dy = dimensions.x / 2, dimensions.y / 2
    corners_local = np.array([[-dx, -dy], [dx, -dy], [dx, dy], [-dx, dy]])
    
    yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
    rot_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    
    corners_world = np.dot(rot_matrix, corners_local.T).T + np.array([position.x, position.y])

    # returning two formats 1d for roi calculations, 2d for marker visualization
    return corners_world.flatten(), corners_world


def visualize_bboxes(corners_list):

    """Create MarkerArray for visualization"""

    global last_marker_count
    global current_z
    marker_array = MarkerArray()
    
    for i, corners in enumerate(corners_list):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bbox_2d"
        marker.id = i
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        for corner in corners:
            p = Point(x=corner[0], y=corner[1], z=current_z-2)
            marker.points.append(p)
        marker.points.append(marker.points[0])
        
        marker_array.markers.append(marker)
    
    # Deleting the previouly published markers on new data arrival
    for i in range(len(corners_list), last_marker_count):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bbox_2d"
        marker.id = i
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    
    last_marker_count = len(corners_list)
    pub_markers.publish(marker_array)


def callback(msg):

    rospy.loginfo(f"Received {len(msg.boxes)} objects")
    
    # 
    bbox_array, corners_list = [], []
    for box in msg.boxes:
        corners_1d, corners_2d = compute_2d_corners(box.pose.position, box.pose.orientation, box.dimensions)
        bbox_array.extend(corners_1d)
        corners_list.append(corners_2d)
    
    pub_bbox_2d.publish(Float32MultiArray(data=bbox_array))
    visualize_bboxes(corners_list)


def listener():

    global pub_bbox_2d, pub_markers

    rospy.init_node('bbox_2d_publisher', anonymous=True)

    pub_bbox_2d = rospy.Publisher("/pca_2d_corners", Float32MultiArray, queue_size=10)
    pub_markers = rospy.Publisher("/bbox_2d_markers", MarkerArray, queue_size=10)

    rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
    rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, callback)
    

    rospy.spin()


if __name__ == '__main__':
    listener()


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

'''
Rakshith Ram [18-2-25]
'''