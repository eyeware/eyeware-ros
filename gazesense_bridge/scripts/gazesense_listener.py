#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) 2021 Eyeware Tech SA http://www.eyeware.tech
"""


import rospy
import gazesense_msgs.msg


def callback(msg):

    rospy.loginfo("GazeSense tracking {} user(s)".format(len(msg.people)))

    for p in msg.people:
        head_pose = p.tracking_info.head_pose
        s = "- Person {} at {:.2f} {:.2f} {:.2f}".format(
            p.id,
            head_pose.transform.translation.x,
            head_pose.transform.translation.y,
            head_pose.transform.translation.z,
        )
        screen_gaze = p.tracking_info.screen_gaze
        if not screen_gaze.is_lost:
            s = "{} is looking at screen {} at ({:d},{:d})".format(
                s, screen_gaze.screen_id, screen_gaze.x, screen_gaze.y
            )
        rospy.loginfo(s)


def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/gazesense/persons", gazesense_msgs.msg.TrackedUserArray, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
