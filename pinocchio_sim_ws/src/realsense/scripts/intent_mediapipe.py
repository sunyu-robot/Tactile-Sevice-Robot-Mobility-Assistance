# -*- coding: utf-8 -*-
"""
Created on 2023.04.10

@brief: get the user's intention and locate the position of user's hip by mediapipe
"""
import traceback
import sys
import os
import rospy
import cv2
import mediapipe as mp
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import numpy as np
import math
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/realsense/scripts")
from realsense import realsense
# from pyquaternion import Quaternion
# from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2.aruco as aruco
from aruco import Aruco_detect

depth_temp = 0
width = 0
height = 0
extrinsic_matrix_R = np.array([[0,0,1],[0,-1,0],[1,0,0]])
extrinsic_matrix_t = np.array([0.5459,0,1.1241])
''' 
Obtaining the three-dimensional coordinates of the point
'''
def get_3d_camera_coordinate(pointx,pointy,depth_frame,camera_intrin):
    global depth_temp
    pose_x = int(pointx * width)
    pose_y = int(pointy * height)
    depth_point = depth_frame.get_distance(pose_x,pose_y)
    # print(depth_point)
    # extrinsic_matrix_R = np.eye((3),dtype=np.float64)
    if (depth_point!=0):
        depth_temp = depth_point
    point_3d_list = rs.rs2_deproject_pixel_to_point(camera_intrin, [pose_x, pose_y], depth_point)
    point_3d = np.array(point_3d_list).reshape(3)
    pointex = np.dot(extrinsic_matrix_R,point_3d) + extrinsic_matrix_t

    return pointex

''' 
Calculating the angle between three points
'''
def get_angle(point_1,point_2,point_3):
    angle_12 = math.atan2(point_2.y - point_1.y, point_1.x - point_2.x) * 180 / math.pi   
    angle_23 = math.atan2(point_2.y - point_3.y,point_3.x - point_2.x) * 180 / math.pi
    angle = angle_12 - angle_23
    return angle

''' 
Draw point for a better look
'''
def draw_point(img,px,py,radius = 5):
    x_xs = int(px * width)
    y_xs = int(py * height)
    cv2.circle(img, (x_xs, y_xs), radius, (0, 0, 255), -1)

def project_vector_onto_plane(u, n):
    dot_product = np.dot(u, n)
    projection = u - dot_product * n
    return projection

def angle_between_vectors(u, v):
    dot_product = np.dot(u, v)
    magnitudes = np.linalg.norm(u) * np.linalg.norm(v)
    cos_theta = dot_product / magnitudes
    theta = math.acos(cos_theta)*np.sign(v[1])
    return theta

if __name__ == "__main__":
    # init ros node and realsense camera
    rospy.init_node('intent', anonymous=True) 
    realsense = realsense()
    pose_pub = rospy.Publisher("/realsense/pose", Pose, queue_size=10)
    result_pub = rospy.Publisher("/realsense/result", Bool, queue_size=10)
    angle_pub = rospy.Publisher("/realsense/angle", Float64, queue_size=10)
    depth_img_pub = rospy.Publisher("depth_image", Image, queue_size=10)
    camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=10)

    # Setting up the Pose function.
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.7, model_complexity=1)

    # get the center of aligned stream
    color_frame,depth_frame = realsense.get_frame()
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    center = (width / 2, height / 2)

    # camera_matrix is the intrinsic matrix, dist is the distortion vector, more imformation can be found in /doc/calibration.md
    camera_matrix = depth_frame.profile.as_video_stream_profile().intrinsics
    camera_matrix.fx = 644.591547364499
    camera_matrix.fy = 644.140670873576
    camera_matrix.ppx = 633.421613500672
    camera_matrix.ppy = 372.042174330502
    camera_matrix_np = np.array([[camera_matrix.fx, 0, camera_matrix.ppx],[0, camera_matrix.fy, camera_matrix.ppy],[0, 0, 1]])
    dist=np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(5, 1)
    aruco_detect = Aruco_detect(camera_matrix_np, dist)

    # set the rotation matrix
    rot_mat =  cv2.getRotationMatrix2D(center, 90, 1)
    rot_mat[0,2] += (height - width) / 2
    rot_mat[1,2] += (width - height) / 2

    # set up parameters
    angel_threshold = 35
    result_msg = Bool()
    pose_msg = Pose()
    angle_msg = Float64()
    result_msg.data = False

    bridge = CvBridge()
    # Create CameraInfo message
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = 'camera_depth_optical_frame'
    camera_info_msg.height = 480
    camera_info_msg.width = 840
    camera_info_msg.distortion_model = 'plumb_bob'
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.K = [423.7496337890625, 0.0, 426.1506042480469, 0.0, 423.7496337890625, 241.25096130371094, 0.0, 0.0, 1.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [423.7496337890625, 0.0, 426.1506042480469, 0.0, 0.0, 423.7496337890625, 241.25096130371094, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info_msg.binning_x = 0
    camera_info_msg.binning_y = 0
    camera_info_msg.roi.do_rectify = False
    
    while not rospy.is_shutdown(): 
        person_detected = False        
        start_time = cv2.getTickCount()  # get start time to calculate FPS
    
        # get color frame and depth frame
        color_frame,depth_frame = realsense.get_frame()
        if not color_frame or not depth_frame:
            continue
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        imageRGB = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

        # Perform the Pose Detection.
        results = pose.process(imageRGB)
        imageRGB.flags.writeable = True
        imageRGB = cv2.cvtColor(imageRGB, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            imageRGB,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

        # try to get the landmarks
        try:
            landmarks = results.pose_landmarks.landmark
            # Get coordinates
            left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value]
            right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]
            left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
            left_elbow = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW]
            left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER] 
            right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            # calculate the angle of user's left elbow and compare numerical size with threshold
            angle_left_elbow = get_angle(left_shoulder,left_elbow,left_wrist)
            # calculate the depth of hip
            waist_x = (left_hip.x+right_hip.x+left_shoulder.x+right_shoulder.x)/4
            waist_y = (left_hip.y+right_hip.y+left_shoulder.y+right_shoulder.y)/4
            waist_camera = get_3d_camera_coordinate(waist_x,waist_y,depth_frame,camera_matrix)
            draw_point(imageRGB,waist_x,waist_y)

            pose_msg.position.x = waist_camera[0]
            pose_msg.position.y = waist_camera[1]
            pose_msg.position.z = waist_camera[2]
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0

            pose_pub.publish(pose_msg)
            if angle_left_elbow <= angel_threshold:
                result_msg.data = True
            else:
                result_msg.data = False
            result_pub.publish(result_msg)
                       
        except Exception as e:
            result_msg.data = False
            result_pub.publish(result_msg)
            traceback.print_exc()
            rospy.logerr("An error occurred: " + str(e))
            pass

        # rotate the image and calculate FPS to have a better view
        img_rotated = cv2.warpAffine(imageRGB, rot_mat, (height, width))
        end_time = cv2.getTickCount()
        time_interval = (end_time - start_time) / cv2.getTickFrequency()
        fps = 1 / time_interval
        cv2.putText(img_rotated, "FPS: {:.2f}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('MediaPipe Pose',img_rotated)
        if cv2.waitKey(5) & 0xFF == 2:
            break
        # publish pose and result
        
        # 发布深度图和相机参数
        # camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_pub.publish(camera_info_msg)
        angle_pub.publish(angle_msg)
        depth_msg = bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        depth_msg.header.stamp = rospy.Time.now()
        depth_img_pub.publish(depth_msg)