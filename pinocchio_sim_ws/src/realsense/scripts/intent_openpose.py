# -*- coding: utf-8 -*-
"""
Created on 2023.04.18

@brief: get the user's intention and locate the position of user's hip by openpose
"""

import sys
# import os
# path = os.path.abspath(".")
sys.path.insert(0,"/home/sunyu/tencent_zju_joint_project/src/pinocchio_sim_ws/src/realsense")
import rospy
import cv2
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import torch
import numpy as np
import math
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from realsense import realsense
from models.with_mobilenet import PoseEstimationWithMobileNet
from modules.keypoints import extract_keypoints, group_keypoints
from modules.load_state import load_state
from modules.pose import Poses, track_poses

"""
Image normalization
"""
def normalize(img, img_mean, img_scale):
    img = np.array(img, dtype=np.float32)
    img = (img - img_mean) * img_scale
    return img

"""
Image Padding
"""
def pad_width(img, stride, pad_value, min_dims):
    h, w, _ = img.shape
    h = min(min_dims[0], h)
    min_dims[0] = math.ceil(min_dims[0] / float(stride)) * stride
    min_dims[1] = max(min_dims[1], w)
    min_dims[1] = math.ceil(min_dims[1] / float(stride)) * stride
    pad = []
    pad.append(int(math.floor((min_dims[0] - h) / 2.0)))
    pad.append(int(math.floor((min_dims[1] - w) / 2.0)))
    pad.append(int(min_dims[0] - h - pad[0]))
    pad.append(int(min_dims[1] - w - pad[1]))
    padded_img = cv2.copyMakeBorder(img, pad[0], pad[2], pad[1], pad[3],
                                    cv2.BORDER_CONSTANT, value=pad_value)
    return padded_img, pad

"""
Scale the input image scaling to the specified size and fill the image, then get the PAF & heatmap
"""
def infer_fast(net, img, net_input_height_size, stride, upsample_ratio,
               pad_value=(0, 0, 0), img_mean=np.array([128, 128, 128], np.float32), img_scale=np.float32(1/256)):
    # get the scale of the imput img
    height, width, _ = img.shape
    scale = net_input_height_size / height

    # Preprocessing input images
    scaled_img = cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    scaled_img = normalize(scaled_img, img_mean, img_scale)
    min_dims = [net_input_height_size, max(scaled_img.shape[1], net_input_height_size)]
    padded_img, pad = pad_width(scaled_img, stride, pad_value, min_dims)

    # Convert the filled image into tensor data and feed it into a neural network for forward calculation
    tensor_img = torch.from_numpy(padded_img).permute(2, 0, 1).unsqueeze(0).float()
    tensor_img = tensor_img.cuda()
    stages_output = net(tensor_img)

    # get heatmaps
    stage2_heatmaps = stages_output[-2]
    heatmaps = np.transpose(stage2_heatmaps.squeeze().cpu().data.numpy(), (1, 2, 0))
    heatmaps = cv2.resize(heatmaps, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)
    
    # get PAFS
    stage2_pafs = stages_output[-1]
    pafs = np.transpose(stage2_pafs.squeeze().cpu().data.numpy(), (1, 2, 0))
    pafs = cv2.resize(pafs, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    return heatmaps, pafs, scale, pad

"""
Estimating human posture from input images
"""
def run_demo(net, img):
    start_time = cv2.getTickCount()
    # set parameters
    height_size = 256
    stride = 8
    upsample_ratio = 4
    num_keypoints = Poses.num_kpts
    previous_poses = []

    orig_img = img.copy()
    heatmaps, pafs, scale, pad = infer_fast(net, img, height_size, stride, upsample_ratio)

    total_keypoints_num = 0
    all_keypoints_by_type = []
    for kpt_idx in range(num_keypoints):  # 19th for bg
        total_keypoints_num += extract_keypoints(heatmaps[:, :, kpt_idx], all_keypoints_by_type, total_keypoints_num)

    pose_entries, all_keypoints = group_keypoints(all_keypoints_by_type, pafs)
    for kpt_id in range(all_keypoints.shape[0]):
        all_keypoints[kpt_id, 0] = (all_keypoints[kpt_id, 0] * stride / upsample_ratio - pad[1]) / scale
        all_keypoints[kpt_id, 1] = (all_keypoints[kpt_id, 1] * stride / upsample_ratio - pad[0]) / scale
    current_poses = []
    for n in range(len(pose_entries)):
        if len(pose_entries[n]) == 0:
            continue
        pose_keypoints = np.ones((num_keypoints, 2), dtype=np.int32) * - 1
        for kpt_id in range(num_keypoints):
            if pose_entries[n][kpt_id] != -1.0:  # keypoint was found
                pose_keypoints[kpt_id, 0] = int(all_keypoints[int(pose_entries[n][kpt_id]), 0])
                pose_keypoints[kpt_id, 1] = int(all_keypoints[int(pose_entries[n][kpt_id]), 1])
        pose = Poses(pose_keypoints, pose_entries[n][18])
        current_poses.append(pose)
    end_time = cv2.getTickCount()
    time_interval = (end_time - start_time) / cv2.getTickFrequency()
    fps = 1 / time_interval

    for pose in current_poses:
        pose.draw(img)
    img = cv2.addWeighted(orig_img, 0.6, img, 0.4, 0)
    cv2.putText(img, "FPS: {:.2f}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow('Lightweight Human Pose Estimation Python Demo', img)
    cv2.waitKey(1)

    return current_poses


if __name__ == "__main__":

    # init ros node and realsense camera
    rospy.init_node('intent', anonymous=True) 
    realsense = realsense()
    pose_pub = rospy.Publisher("pose", Point, queue_size=10)
    result_pub = rospy.Publisher("result", Bool, queue_size=10)

    # load a pre-trained model checkpoint file and move the model to the GPU 
    checkpoint_path = "/home/sunyu/tencent_zju_joint_project/src/pinocchio_sim_ws/src/realsense/models/checkpoint_iter_370000.pth"
    net = PoseEstimationWithMobileNet()
    checkpoint = torch.load(checkpoint_path, map_location='cpu')
    load_state(net, checkpoint)
    net = net.eval()
    net = net.cuda()

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
    # Extrinsic matrix
    extrinsic_matrix = np.eye((4),dtype=np.float64)

    # set the rotation matrix
    rot_mat =  cv2.getRotationMatrix2D(center, -90, 1)
    rot_mat[0,2] += (height - width) / 2
    rot_mat[1,2] += (width - height) / 2

    # set up parameters
    angel_threshold = 35
    result_msg = Bool()
    pose_msg = Point()
    result_msg.data = False

    while not rospy.is_shutdown(): 
        person_detected = False
        start_time = cv2.getTickCount()  # get start time to calculate FPS
        color, depth = realsense.get_frame()
        if not color or not depth:
            continue
        color_image = np.asanyarray(color.get_data())
        # detect person
        current_poses = run_demo(net, color_image)
        person_num = len(current_poses)