#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) 2021 Eyeware Tech SA http://www.eyeware.tech
"""


import rospy
import tf2_ros
import utils


def print_help():
    rospy.loginfo("You can link GazeSense TF tree with, for instance:")
    rospy.loginfo(("     rosrun tf static_transform_publisher 0 0 0 " "yaw pitch roll YOUR_ROOT GS_ROOT 100"))
    rospy.loginfo(("where GS_ROOT is the root of the GazeSense tree, " "for instance 'Camera' or 'World'"))
    rospy.loginfo(("and YOUR_ROOT is the root of your TF tree"))


def publish_tf(setup, root=utils.GS_WORLD):
    """
    """
    rospy.loginfo("Publish TF tree from {}".format(setup))
    rospy.loginfo("TF tree root is '{}'".format(root))
    print_help()

    objects = utils.gs_load_objects(setup)

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transforms = []
    for name, obj in objects.items():
        t = utils.gs_object_to_frame(name, obj, (name == root))
        if t is not None:
            transforms.append(t)
        else:
            rospy.logwarn("Could not create frame for {}".format(name))

    broadcaster.sendTransform(transforms)


if __name__ == "__main__":
    rospy.init_node("gazesense_tf_publisher")
    try:
        setup = rospy.get_param("~setup", "automotive.json")
        root = rospy.get_param("~root", utils.GS_WORLD)
        publish_tf(setup, root)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
