#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
"""Save images of multiple topics with timestamp synchronization.

Usage: rosrun image_view extract_images_sync _inputs:='[<topic_0>, <topic_1>]'
"""

import sys
import os

import cv2

import cv_bridge
import message_filters
import rospy
from sensor_msgs.msg import Image


class ExtractImagesSync(object):

    def __init__(self, param_outdir):
        self.outdir = param_outdir
        self.seq = 0
        self.timestamps =[]

        self.f = open(self.outdir+'/stamps.txt','w')
        # self.fname_fmt = rospy.get_param(
        #     '~filename_format', 'frame%04i_%i.jpg')
        self.fname_fmt = rospy.get_param(
            '~filename_format', '%s%.2i%s')
        self.do_dynamic_scaling = rospy.get_param(
            '~do_dynamic_scaling', False)
        img_topics = rospy.get_param('~inputs', None)
        if img_topics is None:
            rospy.logwarn("""\
extract_images_sync: rosparam '~inputs' has not been specified! \
Typical command-line usage:
\t$ rosrun image_view extract_images_sync _inputs:=<image_topic>
\t$ rosrun image_view extract_images_sync \
_inputs:='[<image_topic>, <image_topic>]'""")
            sys.exit(1)
        if not isinstance(img_topics, list):
            img_topics = [img_topics]
        subs = []
        for t in img_topics:
            subs.append(message_filters.Subscriber(t, Image))
        if rospy.get_param('~approximate_sync', False):
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, queue_size=10, slop=.1)
        else:
            sync = message_filters.TimeSynchronizer(
                subs, queue_size=10)
        # import IPython
        # IPython.embed()
        sync.registerCallback(self.save)

    def save(self, *imgmsgs):
        seq = self.seq
        bridge = cv_bridge.CvBridge()
        for i, imgmsg in enumerate(imgmsgs):
            img = bridge.imgmsg_to_cv2(imgmsg)
            channels = img.shape[2] if img.ndim == 3 else 1
            # encoding_in = bridge.dtype_with_channels_to_cvtype2(
            #     img.dtype, channels)
            encoding_in = imgmsg.encoding
            # import IPython
            # IPython.embed()
            img = cv_bridge.cvtColorForDisplay(
                img, encoding_in=encoding_in, encoding_out='',
                do_dynamic_scaling=self.do_dynamic_scaling)

            #assume image is color (i = 0), set fname as rgb#.jpg
            fname = self.fname_fmt % ('rgb', seq, '.jpg')
            if(i== 1):
                fname = self.fname_fmt % ('depth', seq, '.png')
            print('Save image as {0}'.format(fname))
            cv2.imwrite(self.outdir + "/" + fname, img)
        timestamp = imgmsgs[0].header.stamp.to_sec()
        self.timestamps.append(timestamp)
        self.f.write(str(timestamp)+'\n')
        self.seq = seq + 1

    # def save_timestamps():

    #     print "saving timestamps to file!"
    #     with open('timestamps.txt','w') as f:
    #         for t in self.timestamps:
    #             print >> f, item


if __name__ == '__main__':
    if not os.path.exists(sys.argv[1]):
        os.makedirs(sys.argv[1])

    rospy.init_node('extract_images_sync')
    extractor = ExtractImagesSync(sys.argv[1])

    # rospy.on_shutdown(extractor.save_timestamps)
    rospy.spin()
