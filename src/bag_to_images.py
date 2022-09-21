#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

from glob import glob
import os
import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("-i","--input_files", help="Input ROS bag.", nargs='+')
    parser.add_argument("-o","--output", help="Output directory.")
    parser.add_argument("-t","--topic", default="/camera/image_raw/compressed", help="topic")

    args = parser.parse_args()

    print ("Extract images from %s on topic %s into %s" % (args.input_files,
                                                          args.topic, args.output))
    if not os.path.exists(args.output):
        os.makedirs(args.output)

    file_list_sorted = sorted(args.input_files)
    count = 0
    for fn in file_list_sorted:
        bag = rosbag.Bag(fn, "r")
        bridge = CvBridge()

        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            cv_img = bridge.compressed_imgmsg_to_cv2(msg)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BAYER_GB2RGB)

            cv2.imwrite(os.path.join(args.output, "%f.jpg" % t.to_sec()), cv_img)
            print ("Wrote image %i" % count)

            count += 1

        bag.close()

    return

if __name__ == '__main__':
    main()