#!/usr/bin/env python

from scipy import sparse
from sklearn.cluster import SpectralClustering, DBSCAN
from mps_msgs.srv import *
import numpy as np
import rospy

import matplotlib.pyplot as plt


def handle_segment_graph(req):
    if 'spectral' == req.algorithm:
        adj = sparse.coo_matrix((req.adjacency.value, (req.adjacency.row_index, req.adjacency.col_index)))
        sc = SpectralClustering(n_clusters=req.num_labels, affinity='precomputed', assign_labels='discretize')
        labels = sc.fit_predict(adj)
    elif 'dbscan' == req.algorithm:
        distances = np.exp(-np.asarray(req.adjacency.value)**2)
        print(distances.min(), distances.max())
        logbins = np.geomspace(distances.min()+1e-20, distances.max(), 10)
        plt.hist(distances, bins=logbins)
        plt.xscale('log')
        plt.show()
        adj = sparse.coo_matrix((distances, (req.adjacency.row_index, req.adjacency.col_index)))
        sc = DBSCAN(metric='precomputed', n_jobs=-1, eps=1e-7)
        labels = sc.fit_predict(adj)
    return SegmentGraphResponse(len(np.unique(labels)), labels)


if __name__ == "__main__":
    rospy.init_node('graph_segmentation_server')
    s = rospy.Service('segment_graph', SegmentGraph, handle_segment_graph)
    rospy.spin()
