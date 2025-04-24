#! /usr/bin/env  python

import copy
import rospy

from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import numpy as np
from scipy.spatial.transform import Rotation as R 

class SingleMarkerBroadcaster:
    def __init__(self):
        self.server = InteractiveMarkerServer("dummy_human")
        self.pose = PoseStamped()

        self.pose_pub = rospy.Publisher('/dummy_human_pose', PoseStamped, queue_size=1)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
        # self.marker_d_pub = rospy.Publisher('/desired_pose', Marker, queue_size=10)

        self.global_frame = 'world'

    def makeBox(self, msg):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker
    
    def quaternion2rot(self, quaternion):
        r = R.from_quat(quaternion) # 顺序为 (x, y, z, w)
        rot = r.as_matrix()
        return rot

    def create_marker(self):
        self.int_marker = InteractiveMarker()
        int_marker = self.int_marker
        int_marker.header.frame_id = self.global_frame
        int_marker.header.stamp = rospy.Time.now()
        self.pose.header = int_marker.header

        int_marker.pose.position.x = 1.15
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.9
        self.pose.pose.position = int_marker.pose.position

        int_marker.pose.orientation.x = 0
        int_marker.pose.orientation.y = 0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 1.0
        self.pose.pose.orientation = int_marker.pose.orientation

        int_marker.scale = 0.45

        int_marker.name = "DummyHuman"
        int_marker.description = "Pose for the dummy human"

        control = InteractiveMarkerControl()

        # Custom move on plane
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        
        
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.name = "click"
        control.markers.append(self.makeBox(int_marker))
        int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(int_marker, self.update_pose_callback)

    def apply_changes(self):
        self.server.applyChanges()

    def update_pose_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose.header.frame_id = self.global_frame
            self.pose.header.stamp = rospy.Time.now()
            self.pose.pose.position = feedback.pose.position
            self.pose.pose.orientation = feedback.pose.orientation
            self.marker_cube()  # 更新标记
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo("Marker clicked, publish new goal!")
            self.pose_pub.publish(self.pose)

    def marker_cube(self):
        marker = Marker()
        marker.header.frame_id = "world"  
        marker.header.stamp = rospy.Time.now()
        marker.ns = "dummy_human_marker"
        marker.id = 0  
        marker.type = Marker.CUBE  
        marker.action = Marker.ADD  
        marker.pose.position.x = 1.15
        marker.pose.position.y = 0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.4
        marker.scale.z = 1.8

        L = np.array([0.0, 0.0, -0.9])
        quaternion = [
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w
        ]
        R = self.quaternion2rot(quaternion)
        L_rotated = np.dot(R, L)
        marker.pose.position.x -= L_rotated[0]
        marker.pose.position.y -= L_rotated[1]
        marker.pose.position.z -= L_rotated[2]
        self.pose.pose.position = marker.pose.position

        marker.color.r = 1.0  # 红色
        marker.color.g = 0.0  # 绿色
        marker.color.b = 0.0  # 蓝色
        marker.color.a = 0.7  # 透明度
        marker.pose.orientation = self.pose.pose.orientation
    
        
        self.marker_pub.publish(marker)
        self.pose_pub.publish(self.pose)

if __name__ == '__main__':
    rospy.init_node('dummy_human_publisher', anonymous=True)

    try:
        interactiveTargetPose = SingleMarkerBroadcaster()
        interactiveTargetPose.create_marker()
        interactiveTargetPose.apply_changes()
        # interactiveTargetPose.marker_cube()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
