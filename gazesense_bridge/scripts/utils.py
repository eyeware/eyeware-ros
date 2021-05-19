#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) 2021 Eyeware Tech SA http://www.eyeware.tech
"""

import sys
import math
import json
import collections

import numpy as np

import rospy

from geometry_msgs.msg import TransformStamped
import gazesense_msgs.msg


try:
    import tf
except:
    pass

try:
    import gazesense_client as gs
except:
    pass


import transformations

GS_WORLD = "World"
GS_CAMERA = "Camera"


def gs_load_objects(filename):
    objects = {}
    with open(filename) as f:
        objects = json.load(f, object_pairs_hook=collections.OrderedDict)
    return objects


def gs_object_to_frame(name, obj, is_root=False):
    """Create a TF frame for the input object
    """
    # Camera has a different frame in GazeSense
    droll = 180
    dpitch = 0
    dyaw = 0
    if name.startswith(GS_CAMERA):
        droll = 0
        dpitch = 0
        dyaw = 180
        origin = np.array([0.0, 0.0, 0.0])
    else:
        # We need to know the origin (top left corner of screen) of
        # the frame
        w, h = obj["Size"][:2]
        origin = np.array([-w / 2, h / 2, 0, 1]).reshape(4, 1)

        rotation = tf.transformations.euler_matrix(
            math.radians(obj["Rotation"][0]),
            math.radians(obj["Rotation"][1]),
            math.radians(obj["Rotation"][2]),
            axes="sxyz",
        )
        origin = np.matmul(rotation, origin)

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()

    t.transform.translation.x = origin[0] + obj["Center"][0]
    t.transform.translation.y = origin[1] + obj["Center"][1]
    t.transform.translation.z = origin[2] + obj["Center"][2]

    euler = [math.radians(a) for a in obj["Rotation"]]
    quat = tf.transformations.quaternion_from_euler(*euler)

    rot = tf.transformations.quaternion_from_euler(math.radians(droll), math.radians(dpitch), math.radians(dyaw),)
    quat = tf.transformations.quaternion_multiply(quat, rot)

    if is_root:
        # We know the transformation world -> object, but we need
        # object -> world, because we want the object at the root of
        # the TF tree
        t.header.frame_id = name
        t.child_frame_id = GS_WORLD
        hom = tf.transformations.quaternion_matrix(quat)
        quat = tf.transformations.quaternion_from_matrix(hom.transpose())
        T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,]).reshape(-1, 1)
        d = -np.matmul(hom[0:3, 0:3], T).squeeze()
        t.transform.translation.x = d[0]
        t.transform.translation.y = d[1]
        t.transform.translation.z = d[2]
    else:
        t.header.frame_id = GS_WORLD
        t.child_frame_id = name

    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


def gs_tracking_confidence_to_msg(confidence):
    msg = gazesense_msgs.msg.TrackingConfidence()
    if confidence == gs.TrackingConfidence.UNRELIABLE:
        msg.value = msg.UNRELIABLE
    elif confidence == gs.TrackingConfidence.LOW:
        msg.value = msg.LOW
    elif confidence == gs.TrackingConfidence.MEDIUM:
        msg.value = msg.MEDIUM
    elif confidence == gs.TrackingConfidence.HIGH:
        msg.value = msg.HIGH
    else:
        rospy.logwarn("Unknown confidence {}, setting it to UNRELIABLE".format(confidence))
        msg.value = msg.UNRELIABLE
    return msg


def gs_head_pose_info_to_msg(head_pose_info):
    """
    """
    msg = gazesense_msgs.msg.HeadPoseInfo()
    msg.is_lost = head_pose_info.is_lost
    if not head_pose_info.is_lost:
        msg.track_session_uid = head_pose_info.track_session_uid
        msg.transform.translation.x = head_pose_info.transform.translation[0]
        msg.transform.translation.y = head_pose_info.transform.translation[1]
        msg.transform.translation.z = head_pose_info.transform.translation[2]

        # Python 2/3 problem for tf.transformations
        R = np.zeros((4, 4), dtype=np.float)
        R[3, 3] = 1
        R[:3, :3] = head_pose_info.transform.rotation

        euler = list(transformations.euler_from_matrix(R))
        euler[1] *= -1
        q = transformations.quaternion_from_euler(*euler)

        ##############################################################
        # UNCOMMENT WHEN tf.transformations CAN BE CALLED
        # q = tf.transformations.quaternion_from_matrix(R)
        # q = transformations.quaternion_from_matrix(R)
        ##############################################################

        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

    return msg


def gs_ray_to_msg(ray):
    msg = gazesense_msgs.msg.Ray3D()
    msg.origin.x = ray.origin.x
    msg.origin.y = ray.origin.y
    msg.origin.z = ray.origin.z
    msg.direction.x = ray.direction.x
    msg.direction.y = ray.direction.y
    msg.direction.z = ray.direction.z
    return msg


def gs_gaze_info_to_msg(gaze_info):
    msg = gazesense_msgs.msg.GazeInfo()
    msg.is_lost = gaze_info.is_lost
    msg.left_eye_ray = gs_ray_to_msg(gaze_info.left_eye_ray)
    msg.right_eye_ray = gs_ray_to_msg(gaze_info.right_eye_ray)
    return msg


def gs_screen_gaze_info_to_msg(screen_gaze_info):
    msg = gazesense_msgs.msg.ScreenGazeInfo()
    msg.is_lost = screen_gaze_info.is_lost
    if not screen_gaze_info.is_lost:
        msg.screen_id = screen_gaze_info.screen_id
        msg.x = screen_gaze_info.x
        msg.y = screen_gaze_info.y
        msg.confidence = gs_tracking_confidence_to_msg(screen_gaze_info.confidence)
    return msg


def gs_blink_info_to_msg(blink_info):
    msg = gazesense_msgs.msg.BlinkInfo()
    msg.is_lost = blink_info.is_lost
    if not blink_info.is_lost:
        msg.left_eye_closed = blink_info.left_eye_closed
        msg.right_eye_closed = blink_info.right_eye_closed
        msg.confidence_left = gs_tracking_confidence_to_msg(blink_info.confidence_left)
        msg.confidence_right = gs_tracking_confidence_to_msg(blink_info.confidence_right)
    return msg


def gs_user_to_msg(p):
    msg = gazesense_msgs.msg.TrackedUser()
    msg.id = p.id
    msg.tracking_info.head_pose = gs_head_pose_info_to_msg(p.tracking_info.head_pose)
    if not msg.tracking_info.head_pose.is_lost:
        msg.tracking_info.gaze = gs_gaze_info_to_msg(p.tracking_info.gaze)
    msg.tracking_info.screen_gaze = gs_screen_gaze_info_to_msg(p.tracking_info.screen_gaze)
    msg.tracking_info.blink = gs_blink_info_to_msg(p.tracking_info.blink)
    return msg
