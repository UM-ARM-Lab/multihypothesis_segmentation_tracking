#! /usr/bin/env python

from __future__ import print_function
import time
import rospy
import actionlib

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
from matplotlib import cm

from mps_msgs.msg import MatchFrameSegmentsAction, MatchFrameSegmentsGoal, MatchFrameSegmentsResult, IndexMap

# http://scikit-image.org/docs/dev/auto_examples/segmentation/plot_segmentations.html
from skimage.segmentation import mark_boundaries
from scipy.stats import wasserstein_distance
from scipy.optimize import linear_sum_assignment

bgr = ('b', 'g', 'r')


def hists(img, segments, s):
    mask = np.zeros(img.shape[:2], np.uint8)
    mask[segments == s] = 1
    h = []
    for i, col in enumerate(bgr):
        hi = cv2.calcHist([img], [i], mask, [256], [0, 256])
        h.append(hi) # (hi / (1.0+np.sum(hi)))
    return h


def mask_hists(img, segments, unique_segments):
    histograms = dict()
    for s in unique_segments:
        histograms[s] = hists(img, segments, s)
    return histograms


def segment_distances(unique_segments1, histograms1, unique_segments2, histograms2):
    I = len(unique_segments1)
    J = len(unique_segments2)
    D = np.zeros([I, J])
    for i, n in enumerate(unique_segments1):
        for j, m in enumerate(unique_segments2):
            for k in range(3):
                D[i, j] += wasserstein_distance(histograms1[n][k].flatten(), histograms2[m][k].flatten())

    # plt.figure()
    # plt.imshow(D, cmap=cm.gist_heat, interpolation='nearest')
    # plt.show()
    return D


class HistogramMatcher(object):
    def __init__(self, name):
        self.bridge = CvBridge()
        self.action_name = name
        self.server = actionlib.SimpleActionServer(self.action_name, MatchFrameSegmentsAction,
                                                   execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        try:
            img1 = self.bridge.imgmsg_to_cv2(goal.frames[0].rgb, 'bgr8')
            img2 = self.bridge.imgmsg_to_cv2(goal.frames[-1].rgb, 'bgr8')
            seg1 = self.bridge.imgmsg_to_cv2(goal.frames[0].segments, 'mono16')
            seg2 = self.bridge.imgmsg_to_cv2(goal.frames[-1].segments, 'mono16')
            print(img1.shape, seg1.shape)
        except CvBridgeError as e:
            print(e)
            self.server.set_aborted(result=None, text=e.message)
            return
        # NB: contours encoding is '64FC1'
        # plt.figure()
        # plt.imshow(mark_boundaries(img1[:, :, ::-1], seg1))
        # plt.show()

        unique_segments1 = np.sort(np.unique(seg1))
        unique_segments2 = np.sort(np.unique(seg2))

        # NB: label '0' designates 'background'
        unique_segments1 = unique_segments1[unique_segments1 != 0]
        unique_segments2 = unique_segments2[unique_segments2 != 0]

        histograms1 = mask_hists(img1, seg1, unique_segments1)
        histograms2 = mask_hists(img2, seg2, unique_segments2)

        D = segment_distances(unique_segments1, histograms1, unique_segments2, histograms2)

        row_ind, col_ind = linear_sum_assignment(D)

        resp = MatchFrameSegmentsResult()
        resp.stamps.append(goal.frames[0].rgb.header.stamp)
        resp.stamps.append(goal.frames[-1].rgb.header.stamp)

        a = IndexMap()
        b = IndexMap()
        k = min(len(unique_segments1), len(unique_segments2))
        assert(len(row_ind) == k)
        for i in range(k):
            # a.keys.append(unique_segments1[row_ind[i]])
            # a.values.append(row_ind[i])
            # b.keys.append(unique_segments2[col_ind[i]])
            # b.values.append(col_ind[i])
            # ^ Why is this wrong?
            n = unique_segments1[i]
            # m = unique_segments2[i]
            a.keys.append(n)
            a.values.append(row_ind[i])
            j = col_ind[i]
            b.keys.append(unique_segments2[j])
            b.values.append(row_ind[i])  # < Why is this not col_ind?

        resp.segments_to_bundles.append(a)
        resp.segments_to_bundles.append(b)

        print(resp)

        self.server.set_succeeded(result=resp)

        # plt.figure()
        # plt.imshow(D, cmap=cm.gist_heat, interpolation='nearest')
        # plt.show()


if __name__ == '__main__':
    rospy.init_node('histogram_matcher')
    server = HistogramMatcher(rospy.get_name())
    rospy.spin()
