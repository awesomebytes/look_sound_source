#!/usr/bin/env python

import rospy
from hark_msgs.msg import HarkSource
from control_msgs.msg import PointHeadActionGoal

"""
Make the robot head look in the direction the sound came from
expecting the microphone to be over the head.

Authors: Sammy Pfeiffer
"""


class LookAudio(object):
    def __init__(self, head_link="xtion_link", min_power=30):
        rospy.loginfo("Initalizing LookAudio")
        self.head_link = head_link
        self.min_power = min_power
        self.last_audio = None
        self.head_pub = rospy.Publisher('/head_controller/point_head_action/goal',
                                        PointHeadActionGoal,
                                        queue_size=1)
        self.audio_source_sub = rospy.Subscriber('/hark_source',
                                                 HarkSource,
                                                 self.sound_cb,
                                                 queue_size=1)

        rospy.loginfo("Initialized.")

    def sound_cb(self, data):
        self.last_audio = data

    def look_audio(self, x, y, z, frame='base_link'):
        # send goals to head from last pose to look at it
        phag = PointHeadActionGoal()
        phag.goal.pointing_axis.x = 1.0
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.15)
        phag.goal.pointing_frame = self.head_link
        phag.goal.target.header.frame_id = frame
        phag.goal.target.point.x = x
        phag.goal.target.point.y = y
        phag.goal.target.point.z = z
        self.head_pub.publish(phag)
        rospy.loginfo("Looking at: " + str(phag.goal.target))

    def do_work(self):
        if self.last_audio is None:
            return

        if len(self.last_audio.src) > 0:
            # React to loud stuff only
            if self.last_audio.src[0].power > self.min_power:
                y = self.last_audio.src[0].y
                az = self.last_audio.src[0].azimuth
                # Look only to things kinda in front
                if abs(az) > 60:
                    return
                # Scale a bit the movement of the head to not
                # move too crazy
                scale = 0.5
                y = (az / 100.0) * scale
                # If the movement is too tiny don't do it
                if abs(y) > 0.03:
                    self.look_audio(0.95, y, 0.0,
                                    frame=self.head_link)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.last_audio is not None:
                self.do_work()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('lookaudio')
    ll = LookAudio()
    ll.run()
