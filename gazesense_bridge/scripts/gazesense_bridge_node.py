#!/usr/bin/env python3
# coding: utf8

"""
Copyright (c) 2021 Eyeware Tech SA http://www.eyeware.tech
"""

import sys
import rospy

import gazesense_msgs.msg
import gazesense_client as gs
import utils


class GazesenseBridge(gs.TrackerListener):
    def __init__(self):
        super().__init__()

        self.publisher = rospy.Publisher("persons", gazesense_msgs.msg.TrackedUserArray, queue_size=5)

    def on_track_ready(self, event):

        if len(event.people) < 1:
            return

        msg = gazesense_msgs.msg.TrackedUserArray()

        for i, p in enumerate(event.people):
            msg.people.append(utils.gs_user_to_msg(p))

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = utils.GS_WORLD

        self.publisher.publish(msg)


if __name__ == "__main__":
    """
    """
    rospy.init_node("gazesense_bridge")
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 12000)

    try:
        client = gs.Client(host, port)
        bridge = GazesenseBridge()
        client.register_tracker_listener(bridge)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
