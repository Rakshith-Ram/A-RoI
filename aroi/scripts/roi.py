########################  GENERATES THE CO-ORDINATES FOR ALL THE RoIs (F,B,R,L,ES)  ########################


import rospy
import os
import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool, Float32
from shapely.geometry import Polygon

current_x, current_y, yaw = 0, 0, 0
obs_coords = []
waypoints_front = []
waypoints_back = []

# Initialize ROS node
rospy.init_node('roi_generation', anonymous=True)

roi_publisher = rospy.Publisher('/roi_main_array', Float32MultiArray, queue_size=10)
back_roi_publisher = rospy.Publisher('/roi_back_array', Float32MultiArray, queue_size=10)
roi_es_publisher = rospy.Publisher('/roi_es_coords', Float32MultiArray, queue_size=10)
right_roi_publisher = rospy.Publisher('/roi_right_array', Float32MultiArray, queue_size=10)
left_roi_publisher = rospy.Publisher('/roi_left_array', Float32MultiArray, queue_size=10)
roi_data_pub = rospy.Publisher('/roi_data', Float32MultiArray, queue_size=10)


# Calculate points for additional lines and planes
offset_r = 1.0  # meters
offset_l = 1.0

offset_rr = 3.0
offset_ll = 1.5

b_offset_l = 1.5
b_offset_r = 3.0


def callback_ndt_pose(data):
    global current_x
    global current_y
    current_x = data.pose.position.x
    current_y = data.pose.position.y

def callback_euler_angle(data):
    global yaw
    yaw = data.data[2]


def do_shapes_intersect(roi, obs_rectangle):
    
    # Create polygons
    roi = Polygon(roi)
    obs = Polygon(obs_rectangle)
    
    # Check for intersection
    return roi.intersects(obs)


def callback_pca_corners_global(data):
    global obs_coords
    obs_coords = data.data


def callback_waypoints_front(data):
    global waypoints_front
    waypoints_front = data.data
    #print(waypoints_front)


def callback_waypoints_back(data):
    global waypoints_back
    waypoints_back = data.data


rospy.Subscriber("/pca_2d_corners", Float32MultiArray, callback_pca_corners_global)
rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
rospy.Subscriber("/eular_angle", Float32MultiArray, callback_euler_angle)
rospy.Subscriber("/waypoints_front", Float32MultiArray, callback_waypoints_front)
rospy.Subscriber("/waypoints_back", Float32MultiArray, callback_waypoints_back)

try:
    rospy.wait_for_message("/ndt_pose", PoseStamped)
    
except rospy.ROSInterruptException:
    rospy.loginfo("Node is shutting down.")


rate = rospy.Rate(10)  # 10Hz

while not rospy.is_shutdown():

    offset_points = []
    back_offset_points = []

    for i in range(0, len(waypoints_front) - 3, 2):

        p = [waypoints_front[i], waypoints_front[i + 1]]
        pn = [waypoints_front[i + 2], waypoints_front[i + 3]]

        # Calculate direction vector from p1 to p2
        dx = pn[0] - p[0]
        dy = pn[1] - p[1]

        # Calculate perpendicular unit vector
        length = (dx**2 + dy**2)**0.5
        ux = dy / length
        uy = -dx / length

        # Calculate offset points for the plane
        p_r = Point(x=p[0] + offset_r * ux, y=p[1] + offset_r * uy)
        p_l = Point(x=p[0] - offset_l * ux, y=p[1] - offset_l * uy)
        p_rr = Point(x=p[0] + offset_rr * ux, y=p[1] + offset_rr * uy)
        p_ll = Point(x=p[0] - offset_ll * ux, y=p[1] - offset_ll * uy)

        offset_points.append([p_r, p_l, p_rr, p_ll])


    for i in range(0, len(waypoints_back) - 3, 2):

        b = [waypoints_back[i], waypoints_back[i + 1]]
        bn = [waypoints_back[i + 2], waypoints_back[i + 3]]

        # Calculate direction vector from p1 to p2
        dx = bn[0] - b[0]
        dy = bn[1] - b[1]

        # Calculate perpendicular unit vector
        length = (dx**2 + dy**2)**0.5
        ux = dy / length
        uy = -dx / length

        # Calculate offset points for the plane
        b_r = Point(x=b[0] + b_offset_r * ux, y=b[1] + b_offset_r * uy)
        b_l = Point(x=b[0] - b_offset_l * ux, y=b[1] - b_offset_l * uy)

        back_offset_points.append([b_r, b_l])

    #---------------------------------------------------------------------------------------------#

    roi = []
    flag_roi = False
    obs_dist = float("inf")

    for i in range(len(offset_points) - 1):
        
        # coords of roi
        roi_rect_coords = [(offset_points[i][0].x, offset_points[i][0].y), 
                           (offset_points[i + 1][0].x, offset_points[i + 1][0].y),
                           (offset_points[i + 1][1].x, offset_points[i + 1][1].y),
                           (offset_points[i][1].x, offset_points[i][1].y)]

        obs_dist = float("inf")
        
        for j in range(0, len(obs_coords), 8):
            
            if len(obs_coords) < j + 7:
                continue

            # Extract rectangle points
            x1, y1 = obs_coords[j], obs_coords[j + 1]
            x2, y2 = obs_coords[j + 2], obs_coords[j + 3]
            x3, y3 = obs_coords[j + 4], obs_coords[j + 5]
            x4, y4 = obs_coords[j + 6], obs_coords[j + 7]
            
            obs_rect_coords = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

            if do_shapes_intersect(roi_rect_coords, obs_rect_coords):
                flag_roi = True
                obs_dist = min(
                    math.sqrt((x1 - current_x) ** 2 + (y1 - current_y) ** 2),
                    math.sqrt((x2 - current_x) ** 2 + (y2 - current_y) ** 2),
                    math.sqrt((x3 - current_x) ** 2 + (y3 - current_y) ** 2),
                    math.sqrt((x4 - current_x) ** 2 + (y4 - current_y) ** 2))
                break
        
        if flag_roi == True:
            break
        
        roi.append(offset_points[i][0].x)
        roi.append(offset_points[i][0].y)
        roi.append(offset_points[i + 1][0].x)
        roi.append(offset_points[i + 1][0].y)
        roi.append(offset_points[i + 1][1].x)
        roi.append(offset_points[i + 1][1].y)
        roi.append(offset_points[i][1].x)
        roi.append(offset_points[i][1].y)
    
    print("|  main ROI   | ", flag_roi,"  | ", round(obs_dist, 2),"m  |")
    roi_msg = Float32MultiArray(data=roi)
    roi_publisher.publish(roi_msg)

    #---------------------------------------------------------------------------------------------#

    right_roi = []
    flag_right_roi = False
    obs_dist_right = float("inf")

    for i in range(len(offset_points) - 1):
        
        # coords of roi
        right_roi_rect_coords = [(offset_points[i][2].x, offset_points[i][2].y), 
                           (offset_points[i + 1][2].x, offset_points[i + 1][2].y),
                           (offset_points[i + 1][0].x, offset_points[i + 1][0].y),
                           (offset_points[i][0].x, offset_points[i][0].y)]
        
        obs_dist_right = float("inf")

        for j in range(0, len(obs_coords), 8):
            
            if len(obs_coords) < j + 7:
                continue

            # Extract rectangle points
            x1, y1 = obs_coords[j], obs_coords[j + 1]
            x2, y2 = obs_coords[j + 2], obs_coords[j + 3]
            x3, y3 = obs_coords[j + 4], obs_coords[j + 5]
            x4, y4 = obs_coords[j + 6], obs_coords[j + 7]
            
            obs_rect_coords = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

            if do_shapes_intersect(right_roi_rect_coords, obs_rect_coords):
                flag_right_roi = True
                obs_dist_right = min(
                    math.sqrt((x1 - current_x) ** 2 + (y1 - current_y) ** 2),
                    math.sqrt((x2 - current_x) ** 2 + (y2 - current_y) ** 2),
                    math.sqrt((x3 - current_x) ** 2 + (y3 - current_y) ** 2),
                    math.sqrt((x4 - current_x) ** 2 + (y4 - current_y) ** 2))
                break
        
        if flag_right_roi == True:# or flag_roi == True:
            break
        
        right_roi.append(offset_points[i][2].x)
        right_roi.append(offset_points[i][2].y)
        right_roi.append(offset_points[i + 1][2].x)
        right_roi.append(offset_points[i + 1][2].y)
        right_roi.append(offset_points[i + 1][0].x)
        right_roi.append(offset_points[i + 1][0].y)
        right_roi.append(offset_points[i][0].x)
        right_roi.append(offset_points[i][0].y)
    
    print("|  right ROI  | ", flag_right_roi,"  | ", round(obs_dist_right, 2),"m  |")
    right_roi_msg = Float32MultiArray(data=right_roi)
    right_roi_publisher.publish(right_roi_msg)

    #---------------------------------------------------------------------------------------------#

    left_roi = []
    flag_left_roi = False
    obs_dist_left = float("inf")

    for i in range(len(offset_points) - 1):
        
        # coords of roi
        left_roi_rect_coords = [(offset_points[i][1].x, offset_points[i][1].y), 
                           (offset_points[i + 1][1].x, offset_points[i + 1][1].y),
                           (offset_points[i + 1][3].x, offset_points[i + 1][3].y),
                           (offset_points[i][3].x, offset_points[i][3].y)]
        
        obs_dist_left = float("inf")

        for j in range(0, len(obs_coords), 8):
            
            if len(obs_coords) < j + 7:
                continue

            # Extract rectangle points
            x1, y1 = obs_coords[j], obs_coords[j + 1]
            x2, y2 = obs_coords[j + 2], obs_coords[j + 3]
            x3, y3 = obs_coords[j + 4], obs_coords[j + 5]
            x4, y4 = obs_coords[j + 6], obs_coords[j + 7]
            
            obs_rect_coords = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

            if do_shapes_intersect(left_roi_rect_coords, obs_rect_coords):
                flag_left_roi = True
                obs_dist_left = min(
                    math.sqrt((x1 - current_x) ** 2 + (y1 - current_y) ** 2),
                    math.sqrt((x2 - current_x) ** 2 + (y2 - current_y) ** 2),
                    math.sqrt((x3 - current_x) ** 2 + (y3 - current_y) ** 2),
                    math.sqrt((x4 - current_x) ** 2 + (y4 - current_y) ** 2))
                break
        
        if flag_left_roi == True: # or flag_roi == True:
            break
        
        left_roi.append(offset_points[i][1].x)
        left_roi.append(offset_points[i][1].y)
        left_roi.append(offset_points[i + 1][1].x)
        left_roi.append(offset_points[i + 1][1].y)
        left_roi.append(offset_points[i + 1][3].x)
        left_roi.append(offset_points[i + 1][3].y)
        left_roi.append(offset_points[i][3].x)
        left_roi.append(offset_points[i][3].y)
    
    print("|  left ROI   | ", flag_left_roi,"  | ", round(obs_dist_left, 2),"m  |")
    left_roi_msg = Float32MultiArray(data=left_roi)
    left_roi_publisher.publish(left_roi_msg)

    #---------------------------------------------------------------------------------------------#

    back_roi = []
    flag_back_roi = False
    obs_dist_back = float("inf")

    for i in range(len(back_offset_points) - 1):
        
        # coords of roi
        roi_rect_coords = [(back_offset_points[i][0].x, back_offset_points[i][0].y), 
                           (back_offset_points[i + 1][0].x, back_offset_points[i + 1][0].y),
                           (back_offset_points[i + 1][1].x, back_offset_points[i + 1][1].y),
                           (back_offset_points[i][1].x, back_offset_points[i][1].y)]

        back_roi.append(back_offset_points[i][0].x)
        back_roi.append(back_offset_points[i][0].y)
        back_roi.append(back_offset_points[i + 1][0].x)
        back_roi.append(back_offset_points[i + 1][0].y)
        back_roi.append(back_offset_points[i + 1][1].x)
        back_roi.append(back_offset_points[i + 1][1].y)
        back_roi.append(back_offset_points[i][1].x)
        back_roi.append(back_offset_points[i][1].y)

    for i in range(len(back_offset_points) - 1):

        # coords of roi
        roi_rect_coords = [(back_offset_points[i][0].x, back_offset_points[i][0].y), 
                           (back_offset_points[i + 1][0].x, back_offset_points[i + 1][0].y),
                           (back_offset_points[i + 1][1].x, back_offset_points[i + 1][1].y),
                           (back_offset_points[i][1].x, back_offset_points[i][1].y)]

        obs_dist_back = float("inf")
        
        for j in range(0, len(obs_coords), 8):
            
            if len(obs_coords) < j + 7:
                continue

            # Extract rectangle points
            x1, y1 = obs_coords[j], obs_coords[j + 1]
            x2, y2 = obs_coords[j + 2], obs_coords[j + 3]
            x3, y3 = obs_coords[j + 4], obs_coords[j + 5]
            x4, y4 = obs_coords[j + 6], obs_coords[j + 7]
            
            obs_rect_coords = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

            if do_shapes_intersect(roi_rect_coords, obs_rect_coords):
                flag_back_roi = True
                obs_dist_back= min(
                    math.sqrt((x1 - current_x) ** 2 + (y1 - current_y) ** 2),
                    math.sqrt((x2 - current_x) ** 2 + (y2 - current_y) ** 2),
                    math.sqrt((x3 - current_x) ** 2 + (y3 - current_y) ** 2),
                    math.sqrt((x4 - current_x) ** 2 + (y4 - current_y) ** 2))
                break
        
        if flag_back_roi == True:
            break
    
    print("|  back ROI   | ", flag_back_roi,"  | ", round(obs_dist_back, 2), "m  |")
    back_roi_msg = Float32MultiArray(data=back_roi)
    back_roi_publisher.publish(back_roi_msg)

    #---------------------------------------------------------------------------------------------#

    roi_es = []
    flag_es = False
    
    original_coords = [(2.5,-1.25), (-3.5,-1.25), (-3.5,1.25), (2.5, 1.25)]

    # Create the rotation matrix
    rotation_matrix = np.array([
        [np.cos(yaw+0.05), -np.sin(yaw+0.05)],
        [np.sin(yaw+0.05),  np.cos(yaw+0.05)]])

    roi_es_coords = []
    for x, y in original_coords:
        # Rotate and translate
        rotated = np.dot(rotation_matrix, np.array([x, y]))
        new_x, new_y = rotated[0] + current_x, rotated[1] + current_y
        roi_es_coords.append((new_x, new_y))

        roi_es.append(new_x)
        roi_es.append(new_y)


    for j in range(0, len(obs_coords), 8):
            
        if len(obs_coords) < j + 7:
            continue

        # Extract rectangle points
        x1, y1 = obs_coords[j], obs_coords[j + 1]
        x2, y2 = obs_coords[j + 2], obs_coords[j + 3]
        x3, y3 = obs_coords[j + 4], obs_coords[j + 5]
        x4, y4 = obs_coords[j + 6], obs_coords[j + 7]
        
        obs_rect_coords = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

        if do_shapes_intersect(roi_es_coords, obs_rect_coords):
            flag_es = True
            break

    print("|  es ROI     | ", flag_es, "  |")
    print()
    roi_es_msg = Float32MultiArray(data=roi_es)
    roi_es_publisher.publish(roi_es_msg)

    #---------------------------------------------------------------------------------------------#

    roi_data = [float(flag_roi), obs_dist, float(flag_right_roi), obs_dist_right,
            float(flag_left_roi), obs_dist_left, float(flag_back_roi), obs_dist_back, float(flag_es)]

    msg = Float32MultiArray(data=roi_data)
    roi_data_pub.publish(msg)

    rate.sleep()  # 10Hz


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

'''
Rakshith Ram [30-1-25]
'''