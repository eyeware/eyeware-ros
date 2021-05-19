#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) 2021 Eyeware Tech SA http://www.eyeware.tech
"""

# Run this to put the GazeSense world frame in an existing tf
#
# rosrun tf static_transform_publisher 0 0 0 0.707 0 0 0.707 map World 10
# rosrun tf static_transform_publisher 0 0 0 -1.57079 0 -1.57079 camera_link Camera 100

import os
import sys
import json
import pprint
import math
import rospy
import numpy as np
import tf2_ros
import tf
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

import gazesense_msgs.msg
import utils

# Default color of a screen
COLOR = ColorRGBA(0.996, 0.788, 0.243, 1.0)

# Screen color when looked at by the user
SELECTED_COLOR = ColorRGBA(0.996, 0.288, 0.243, 1.0)

# z component, to obtain a thin screen
SCREEN_THICKNESS = 0.005

# Color of eye rays
RAY_COLOR = ColorRGBA(0.961, 0.471, 0, 1.0)

# Eye ray scale, to visualize lines and not arrows
RAY_SCALE = Vector3(0.005, 0.005, 0.0)

# Multiplying factor of direction to adapt its length
RAY_LENGTH = 0.3  # || ray.direction || = 1 meter

# Cube color of gaze intersection
FOCUS_COLOR = ColorRGBA(0.1, 0.1, 0.1, 1.0)

# Size of cube to be like in GazeSense viewer
FOCUS_SCALE = Vector3(0.05, 0.05, 0.05)

# Color of the camera
# Note: the triangle is not the same shade of purple as in GazeSense viewer
CAMERA_COLOR = ColorRGBA(0.686, 0.686, 0.913, 1.0)

# Marker lifetime in RViz
DURATION = rospy.Duration(0.1)


def screen_coord_to_marker_coord(u, v, screen):
    x = float(u) / screen["Resolution"][0] * screen["Size"][0]
    y = float(v) / screen["Resolution"][1] * screen["Size"][1]
    return x, y


def new_marker(scale=1.0):
    m = Marker()
    m.header.frame_id = utils.GS_WORLD
    m.header.stamp = rospy.Time.now()
    m.type = m.SPHERE
    m.scale = Vector3(scale, scale, scale)
    m.color = ColorRGBA(1, 0, 0, 1)
    m.lifetime = DURATION
    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    return m


def new_eye_marker(ray, frame_id, sign):
    m = new_marker()
    m.header.frame_id = utils.GS_WORLD
    m.type = m.ARROW
    m.color = RAY_COLOR
    m.scale = RAY_SCALE
    m.points.append(Point(ray.origin.x, ray.origin.y, ray.origin.z))
    m.points.append(
        Point(
            ray.origin.x + RAY_LENGTH * ray.direction.x,
            ray.origin.y + RAY_LENGTH * ray.direction.y,
            ray.origin.z + RAY_LENGTH * ray.direction.z,
        )
    )
    return m


def new_focus_marker(color=FOCUS_COLOR, scale=FOCUS_SCALE):
    m = new_marker()
    m.type = m.CUBE
    m.scale = scale
    m.color = color
    return m


def object_to_camera_marker(obj, scale=0.05):
    m = Marker()
    m.header.frame_id = obj["Name"]
    m.header.stamp = rospy.Time.now()
    m.type = m.MESH_RESOURCE
    m.mesh_resource = "package://gazesense_bridge/objects/camera.stl"
    m.scale = Vector3(scale, scale, scale)
    m.color = CAMERA_COLOR
    m.lifetime = DURATION
    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    return m


def object_to_screen_marker(obj, scale=0.5, is_selected=False):
    width, height = obj["Size"][:2]
    m = Marker()
    m.header.frame_id = obj["Name"]
    m.header.stamp = rospy.Time.now()
    m.type = m.CUBE
    m.scale = Vector3(width, height, SCREEN_THICKNESS)
    m.color = SELECTED_COLOR if is_selected else COLOR
    m.lifetime = DURATION
    m.pose.position.x = width / 2.0
    m.pose.position.y = height / 2.0
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    return m


class Visualizer:
    def __init__(self, setup, root="Camera"):
        rospy.loginfo("GazeSense visualizer with {}".format(setup))

        self.objects = utils.gs_load_objects(setup)

        self.screens = []
        self.cameras = []

        for name, obj in self.objects.items():
            if name.startswith(utils.GS_CAMERA):
                self.cameras.append(obj)
            else:
                self.screens.append(obj)

        if len(self.cameras) > 1:
            rospy.logwarn("Untested ROS code with multiple cameras")

        queue_size = 1

        self.subscriber = rospy.Subscriber("persons", gazesense_msgs.msg.TrackedUserArray, self.person_callback)

        self.camera_publisher = rospy.Publisher("camera", MarkerArray, queue_size=queue_size)

        self.screen_publisher = rospy.Publisher("screens", MarkerArray, queue_size=queue_size)

        self.gaze_publisher = rospy.Publisher("gaze", MarkerArray, queue_size=queue_size)

        self.gaze_intersection_publisher = rospy.Publisher("gaze_intersection", MarkerArray, queue_size=queue_size)

        self.person_broadcaster = tf2_ros.TransformBroadcaster()

    def add_object_markers(self, marker_array):
        """Fill marker_array.markers with screen and camera markers

        Args:
            marker_array: MarkerArray

        """
        for name, obj in self.objects.items():

            if name == "Camera":
                m = object_to_camera_marker(name, obj)
            else:
                m = object_to_screen_marker(name, obj)

            marker_array.markers.append(m)

    def publish_user_frame(self, user):
        """
        """
        position = user.tracking_info.head_pose.transform.translation
        rotation = user.tracking_info.head_pose.transform.rotation

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = utils.GS_WORLD
        t.child_frame_id = person_name
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        t.transform.rotation.x = rotation.x
        t.transform.rotation.y = rotation.y
        t.transform.rotation.z = rotation.z
        t.transform.rotation.w = rotation.w
        self.person_broadcaster.sendTransform(t)

    def person_callback(self, imsg):
        """
        """

        # Note: only tracking one person is supported for now
        if len(imsg.people) > 0:
            pid = 0
            user = imsg.people[pid]
            user_name = "person{}".format(imsg.people[pid].id)
        else:
            return

        screen_id = -1
        x = -1
        y = -1
        confidence = -1
        position = None
        rotation = None

        gaze_markers = MarkerArray()

        # User TF frame
        if not user.tracking_info.head_pose.is_lost:
            position = user.tracking_info.head_pose.transform.translation
            rotation = user.tracking_info.head_pose.transform.rotation
            confidence = user.tracking_info.screen_gaze.confidence.value

            t = TransformStamped()
            t.header.stamp = imsg.header.stamp
            t.header.frame_id = utils.GS_WORLD
            t.child_frame_id = user_name
            t.transform.translation.x = position.x
            t.transform.translation.y = position.y
            t.transform.translation.z = position.z

            t.transform.rotation.x = rotation.x
            t.transform.rotation.y = rotation.y
            t.transform.rotation.z = rotation.z
            t.transform.rotation.w = rotation.w
            self.person_broadcaster.sendTransform(t)

            # Eye rays
            if not user.tracking_info.gaze.is_lost:
                left_eye_ray = user.tracking_info.gaze.left_eye_ray
                m = new_eye_marker(left_eye_ray, user_name, -1)
                gaze_markers.markers.append(m)
                right_eye_ray = user.tracking_info.gaze.right_eye_ray
                m = new_eye_marker(right_eye_ray, user_name, 1)
                gaze_markers.markers.append(m)

        if not user.tracking_info.screen_gaze.is_lost:
            screen_id = user.tracking_info.screen_gaze.screen_id
            x = user.tracking_info.screen_gaze.x
            y = user.tracking_info.screen_gaze.y

        # If gaze is unreliable, we do as if no screen is looked at
        if confidence == gazesense_msgs.msg.TrackingConfidence.UNRELIABLE:
            screen_id = -1

        camera_markers = self.get_camera_markers()

        screen_markers = self.get_screen_markers(screen_id)

        gaze_intersection_markers = self.get_gaze_intersection_markers(screen_id, x, y)

        # Set proper timestamp and IDs of markers
        for markers in [
            camera_markers,
            screen_markers,
            gaze_intersection_markers,
            gaze_markers,
        ]:
            for i, m in enumerate(markers.markers):
                m.id = i
                m.header.stamp = imsg.header.stamp

        self.camera_publisher.publish(camera_markers)
        self.screen_publisher.publish(screen_markers)
        self.gaze_publisher.publish(gaze_markers)
        self.gaze_intersection_publisher.publish(gaze_intersection_markers)

    def get_camera_markers(self):
        """
        """
        msg = MarkerArray()
        for i, camera in enumerate(self.cameras):
            m = object_to_camera_marker(camera)
            msg.markers.append(m)
        return msg

    def get_screen_markers(self, screen_id):
        """
        """
        msg = MarkerArray()
        for i, screen in enumerate(self.screens):
            m = object_to_screen_marker(screen, is_selected=(i == screen_id))
            msg.markers.append(m)
        return msg

    def get_gaze_intersection_markers(self, screen_id, x, y):
        """
        """
        msg = MarkerArray()
        if screen_id >= 0:
            m = new_focus_marker()
            screen = self.screens[screen_id]
            m.header.frame_id = screen["Name"]
            x, y = screen_coord_to_marker_coord(x, y, screen)
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0
            msg.markers.append(m)
        return msg


if __name__ == "__main__":
    """
    """
    rospy.init_node("gazesense_viewer")
    try:
        setup = rospy.get_param("~setup", "automotive.json")
        visualizer = Visualizer(setup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
