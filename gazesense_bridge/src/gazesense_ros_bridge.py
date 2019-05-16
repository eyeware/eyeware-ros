#!/usr/bin/env python
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np
import copy
import math
import time
import cv2 as cv
import rospy

from GazeSense import Camera, multi_rot, get_rotation_matrix_in_z, GazeSenseClient, get_rotation_matrix_in_x, CylinderGazeTarget, PlanarGazeTarget, PointGazeTarget
import GazeSense as GS

from tf.transformations import quaternion_from_matrix

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

from gazesense_bridge.msg import Person
from gazesense_bridge.msg import PersonArray
from gazesense_bridge.msg import ScreenTargets

import tf.transformations as tft
import math as m
from gazesense_bridge.srv import ChangeUser, ChangeUserResponse

class GazesenseRosBridge(object):

    current_target = ""
    current_confidence = 0
    target_data = ""

    def __init__(self):
        """
        GazesenseRosBridge constructor

        """
        # DO NOT EDIT HERE BUT IN THE LAUNCH FILE
        rospy.init_node("gazesense_bridge")
        rospy.sleep(0.5)

        self.frame_id = rospy.get_param("~gaze_sense_frame_id", "camera_link")
        host_ip = rospy.get_param("~gaze_sense_host_ip", '192.168.4.191')
        pub_rate = rospy.get_param("~pub_rate", 10)
        rospy.loginfo("Connecting to: %s" % str(host_ip))

        # self.gaze_pub = rospy.Publisher("gaze_objects", GazeObjects, queue_size=2)
        self.person_pub = rospy.Publisher("gs_persons", PersonArray, queue_size=2)
        # self.robot = Cobot("", init_robot_commander=True)
        self.pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=1)

        rospy.Service("/change_user", ChangeUser, self.change_user_cb)

        self.gc = GazeSenseClient(server_host_ip=host_ip,
                                  server_port=12000,
                                  callback=self.gazesense_client_callback,
                                  verbose=True)

        # Set up static world
        self.static_gaze_sense_targets = []
        self.dynamic_gaze_sense_targets = []
        self.setup_static_world()

        self.last_frame = None
        self.previous_frame_time = 0
        self.video_request_period_s = 0.05
        self.gc.start()

        self._setting_up_new_username = None

        rospy.loginfo("Gazesense connection started")

        rate = rospy.Rate(pub_rate)
        while not rospy.is_shutdown():
            self.update_dynamic_world()
            self.publish_markers()
            rate.sleep()

        self.gc.stop()

    def change_user_cb(self, msg):
        self._setting_up_new_username = msg
        #self.gc.send_current_user(msg.user_name)

        return ChangeUserResponse(True)

    def gazesense_client_callback(self, data):
        """
        Callback function of the zeromq communication with gazesense api
        :param data:
        :return:
        """
        self.current_target = data['GazeCoding']

        if data['ConnectionOK'] and data['InTracking']:

            if data['GazeCoding'] in data['Attention Scores']:
                self.current_confidence = float(data['Attention Scores'][data['GazeCoding']])

            msg = PersonArray()

            # FOR LOOP ON PERSON SHOULD START HERE
            # for loop the persons
            p = Person()

            # Nose tip
            p.nose_tip.x = data['nose_tip'][0]
            p.nose_tip.y = data['nose_tip'][1]
            p.nose_tip.z = data['nose_tip'][2]

            p.point_of_regard.x = data['point_of_regard_3D'][0]
            p.point_of_regard.y = data['point_of_regard_3D'][1]
            p.point_of_regard.z = data['point_of_regard_3D'][2]

            # Head Pose
            p.head_pose.position.x = data['head_pose'][1][0]
            p.head_pose.position.y = data['head_pose'][1][1]
            p.head_pose.position.z = data['head_pose'][1][2]

            # TODO gaze_origin_left
            # TODO gaze_origin_right
            # TODO gaze_vector_left
            # TODO gaze_vector_right

            matrix = [c + [0] for c in data['head_pose'][0]] + [[0, 0, 0, 1]]
            matrix = np.array(matrix)
            q = quaternion_from_matrix(matrix)
            p.head_pose.orientation.x = q[0]
            p.head_pose.orientation.y = q[1]
            p.head_pose.orientation.z = q[2]
            p.head_pose.orientation.w = q[3]


            # Attention
            p.attention.current_target = data['GazeCoding']
            p.attention.tracking = data['InTracking']

            for label in data['Attention Scores']:
                p.attention.object_names_gaze.append(label)
                p.attention.object_scores_gaze.append(round(float(data['Attention Scores'][label]), 2))

            for label in data['Head Attention Scores']:
                p.attention.object_names_head.append(label)
                p.attention.object_scores_head.append(round(float(data['Attention Scores'][label]), 2))

            msg.persons.append(p)

            # FOR LOOP ON PERSON SHOULD END HERE

            self.person_pub.publish(msg)

    def update_dynamic_world(self):

        if self._setting_up_new_username is not None:
            self.gc.send_current_user(self._setting_up_new_username.user_name)
            self._setting_up_new_username = None

        # clear dynamic sense targets
        self.dynamic_gaze_sense_targets[:] = []

        # Send update to gazesense software
        self.dynamic_gaze_sense_targets.extend(self.static_gaze_sense_targets)
        self.gc.send_gaze_targets_list(self.dynamic_gaze_sense_targets)

        # Camera Position
        cam = Camera(rotation=multi_rot(get_rotation_matrix_in_z(0),
                                        get_rotation_matrix_in_x(-math.pi / 2.5 )),
                     translation=[-1.20, -0.20, 0.20])
        self.gc.send_camera_pose(cam)

        # Marker position
        if self.last_frame is not None:

            # self.update_marker_position()
            self.last_frame = None

        if (rospy.get_time() - self.previous_frame_time) > self.video_request_period_s > 0.0:
            self.gc.request_next_video_frame()
            self.previous_frame_time = rospy.get_time()

    def setup_static_world(self):
        """
        Sets up the static world for gaze sense.
        todo: Should be made configurable trough json / yaml
        :return:
        """
        world_objects = []

        num_point_targets = 3
        for i in range(num_point_targets):
            t = [0.0, 0.0, 0.0]
            t[0] = math.cos(-6.28 * time.time() / 10.0 + 6.28 * i / num_point_targets)
            t[1] = math.sin(-6.28 * time.time() / 10.0 + 6.28 * i / num_point_targets)
            gt = PointGazeTarget('target_{}'.format(i), translation=t)
            world_objects.append(gt)

        self.static_gaze_sense_targets = world_objects

    
    def publish_markers(self):
        """
        For the currently defined set of gaze targets we publish markers for rviz visualization
        """
        marker_array = MarkerArray()

        idx = 0
        for target in self.dynamic_gaze_sense_targets:
            marker = self.gaze_target_to_marker(target)

            marker.lifetime = rospy.Duration.from_sec(0.1)
            marker.action = marker.ADD
            marker.ns = "gazesense_markers"
            marker.id = idx

            if target.label_ in self.current_target:
                marker.color.a = 1.0
                marker.color.r = self.current_confidence
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)

            idx += 1

        self.pub.publish(marker_array)

    def gaze_target_to_marker(self, obj):
        """
        Converts a GazeSenseTarget to a rviz marker
        :param obj: A GazeSenseTarget object
        :return: A marker object
        """

        # For every object this is the same
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = self.frame_id

        if type(obj) is CylinderGazeTarget:
            marker.type = marker.CYLINDER
            # size
            marker.scale.x = obj.radius_ * 2.0
            marker.scale.y = obj.radius_ * 2.0
            marker.scale.z = obj.height_

            # Position

            if obj.rotation_:
                res = np.dot(obj.rotation_, [0, 0, obj.height_ / 2.0])
                marker.pose.position.x = res[0]
                marker.pose.position.y = res[1]
                marker.pose.position.z = res[2]

                mat = tft.identity_matrix()
                mat[0:3, 0:3] = np.array(obj.rotation_, dtype=np.float64)
                quad = tft.quaternion_from_matrix(mat)
                marker.pose.orientation.x = quad[0]
                marker.pose.orientation.y = quad[1]
                marker.pose.orientation.z = quad[2]
                marker.pose.orientation.w = quad[3]
            else:
                # Gazesense defines zero of cylinder at bottom
                marker.pose.position.z = obj.height_ / 2.0

            if obj.translation_:
                marker.pose.position.x += obj.translation_[0]
                marker.pose.position.y += obj.translation_[1]
                marker.pose.position.z += obj.translation_[2]

        if type(obj) is PointGazeTarget:
            marker.type = marker.SPHERE

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            if obj.translation_:
                marker.pose.position.x += obj.translation_[0]
                marker.pose.position.y += obj.translation_[1]
                marker.pose.position.z += obj.translation_[2]

        if type(obj) is PlanarGazeTarget:
            # d    c
            # _____     y
            # |\  |     |
            # | \ |     |
            # |  \|     |
            # -----     ---> x
            # a   b
            points = []
            # Point a
            a = Point()
            # Point b
            b = copy.deepcopy(a)
            b.x += obj.size_x_
            # Point c
            c = copy.deepcopy(b)
            c.y += obj.size_y_
            # Point d
            d = copy.deepcopy(a)
            d.y += obj.size_y_

            points.append(copy.deepcopy(a))
            points.append(copy.deepcopy(b))
            points.append(copy.deepcopy(d))
            points.append(copy.deepcopy(d))
            points.append(copy.deepcopy(b))
            points.append(copy.deepcopy(c))

            if obj.rotation_:
                self.apply_rotation(points, obj.rotation_)

            if obj.translation_:
                self.apply_translation(points, obj.translation_)

            marker.type = marker.TRIANGLE_LIST
            marker.points = points
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1

        return marker

    @staticmethod
    def apply_rotation(points, rotation):
        """
        Applies a rotaion specified by a 3x3 rotation matrix
        to a set op points defied in points operates in place
        on the data in the point array as arrays are passed by reference
        :param points:
        :param rotation:
        :return: nothing
        """
        for p in points:
            res = np.dot(rotation, [p.x, p.y, p.z])
            p.x = res[0]
            p.y = res[1]
            p.z = res[2]

    @staticmethod
    def apply_translation(points, translation):
        """
        Apply a translation specified in a 1x3 vector to the set of points
        specified in points
        :param points:
        :param translation:
        :return: nothing
        """
        for p in points:
            p.x += translation[0]
            p.y += translation[1]
            p.z += translation[2]
