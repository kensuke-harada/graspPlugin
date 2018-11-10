#!/usr/bin/env python

import sys
import math
import os.path


class FeatureGenrator:
    def __init__(self):
        self.min_bb = []
        self.max_bb = []
        self.fmin_bb = []
        self.fmax_bb = []
        self.points = []

    def _read_data(self, sv_point_file):
        count = 0
        for line in open(sv_point_file):
            if count == 0:
                tmp = map(float, line.rstrip("\n").split(" "))
                self.min_bb = tmp[:3]
                self.max_bb = tmp[3:]
            elif count == 1:
                tmp = map(float, line.rstrip("\n").split(" "))
                self.fmin_bb = tmp[:3]
                self.fmax_bb = tmp[3:]
            else:
                self.points.append(map(float, line.rstrip("\n").split(" ")))
            count += 1

    def _calc_sum_height(self):
        return sum(map(lambda p: self.max_bb[1] - p[1], self.points))

    def _calc_sum_dist(self):
        return sum(map(lambda p: min(
            min(self.fmax_bb[2] - p[2], p[2] - self.fmin_bb[2]),
            min(self.fmax_bb[0] - p[0], p[0] - self.fmin_bb[0])),
                       self.points))

    def generate_2dim_feature(self, sv_point_file):
        self._read_data(sv_point_file)
        return "%f %f" % (self._calc_sum_height(), self._calc_sum_dist())

    def _calc_histogram(self, arr, dim):
        max_idx = dim - 1
        step = 0.01
        for p in self.points:
            height = self.fmax_bb[1] - p[1]
            dist = min(self.fmax_bb[2] - p[2], p[2] - self.fmin_bb[2])
            h_idx = int(min(math.floor(height / step), max_idx))
            d_idx = int(max(min(math.floor(dist / step), max_idx), 0))
            center_h = (h_idx + 0.5) * step
            center_d = (d_idx + 0.5) * step
            n_h_idx = (min(h_idx + 1, max_idx)
                       if (height - center_h) > 0 else max(h_idx - 1, 0))
            n_d_idx = (min(d_idx + 1, max_idx)
                       if (dist - center_d) > 0 else max(d_idx - 1, 0))
            rate_h = 1.0 - abs(height - center_h) / step
            rate_d = 1.0 - abs(dist - center_d) / step
            arr[h_idx][d_idx] += rate_h * rate_d
            arr[h_idx][n_d_idx] += rate_h * (1.0 - rate_d)
            arr[n_h_idx][d_idx] += (1.0 - rate_h) * rate_d
            arr[n_h_idx][n_d_idx] += (1.0 - rate_h) * (1.0 - rate_d)

    def generate_25dim_feature(self, sv_point_file):
        self._read_data(sv_point_file)
        dim = 5
        arr = [[0 for i in range(dim)] for j in range(dim)]
        self._calc_histogram(arr, dim)
        feature_str = ""
        for i in range(dim):
            for j in range(dim):
                feature_str += str(arr[i][j]) + " "
        return feature_str

labelfile = sys.argv[1] + "/label.txt"
if not os.path.exists(labelfile):
    sys.exit()

features = ""

for line in open(labelfile):
    tmp = map(int, line.rstrip("\n").split(" "))
    if tmp[1] == -1:
        continue
    sv_point_file = sys.argv[1] + ("/sv_points_%04d" % tmp[0])
    g = FeatureGenrator()
    if tmp[1] == 1:
        features += "1 "
    elif tmp[1] == 0:
        features += "-1 "
    features += g.generate_25dim_feature(sv_point_file)
    features += "\n"

print features
