#!/usr/bin/env python

import sys
from math import ceil
import re
from tempfile import mkstemp
from shutil import move
from os import remove, close
from subprocess import Popen, PIPE


def chunk(ite, n):
    return [ite[x:x + n] for x in range(0, len(ite), n)]


class GridSearch:
    def __init__(self, n_fold=3, num_trees=[50], num_features=[5]):
        self.num_sample = 0
        self.n_fold = n_fold
        self.num_trees = num_trees
        self.num_features = num_features

    def generate_input_file(self, datasets, target_n):
        train_dataset = ""
        test_dataset = ""
        for i in range(self.n_fold):
            if i == target_n:
                test_dataset = datasets[i]
            else:
                train_dataset += datasets[i]
        ftrain = open("_traindataset", 'w')
        ftrain.write(train_dataset)
        ftest = open("_testdataset", 'w')
        ftest.write(test_dataset)

    def replace_ini_file(self, n_tree, n_feature):
        n_feature_pattern = r"numFeature"
        n_tree_pattern = r"numTree"
        ini_file = "param.ini"

        fh, temp_path = mkstemp()
        with open(temp_path, 'w') as new_file:
            with open(ini_file) as old_file:
                for line in old_file:
                    if re.match(n_feature_pattern, line):
                        new_file.write("numFeature=" + str(n_feature) + "\n")
                    elif re.match(n_tree_pattern, line):
                        new_file.write("numTree=" + str(n_tree) + "\n")
                    else:
                        new_file.write(line)
        close(fh)
        remove(ini_file)
        move(temp_path, ini_file)

    def train(self):
        p = Popen(["./train", "_traindataset"], stdout=PIPE)
        p.wait()

    def makemodel(self, filepath):
        p = Popen(["./train", filepath], stdout=PIPE)
        p.wait()

    def predict(self):
        p = Popen(["./predict", "_testdataset"], stdout=PIPE)
        p.wait()
        for line in p.stdout.readlines():
            if re.match(r"Accuracy:", line):
                return float(line.rstrip("\n").split(" ")[1])
        return 0.0

    def grid_search(self, datafile):
        samples = []
        for line in open(datafile):
            samples.append(line)
        self.num_sample = int(ceil(len(samples) / float(self.n_fold)))
        chunked = chunk(samples, self.num_sample)
        datasets = [''.join(data) for data in chunked]

        best_num_tree = 50
        best_num_features = 5
        best_accuracy = 0
        for n_tree in self.num_trees:
            for n_feature in self.num_features:
                print "parameters: num_tree(" + str(n_tree) + ")",
                print "num_feature(" + str(n_feature) + ")"
                self.replace_ini_file(n_tree, n_feature)
                sum_acc = 0
                for i in range(self.n_fold):
                    self.generate_input_file(datasets, i)
                    self.train()
                    acc = self.predict()
                    print " test" + str(i) + " :" + str(acc)
                    sum_acc += acc
                ave_acc = sum_acc / self.n_fold
                print " average:" + str(ave_acc)
                if ave_acc > best_accuracy:
                    best_num_features = n_feature
                    best_num_tree = n_tree
                    best_accuracy = ave_acc

        print "best: num_tree(" + str(best_num_tree) + ")",
        print "num_feature(" + str(best_num_features) + ")",
        print "accuracy " + str(best_accuracy)
        self.replace_ini_file(best_num_tree, best_num_features)
        self.makemodel(datafile)

if __name__ == '__main__':
    search = GridSearch(n_fold=3,
                        num_trees=[50, 100, 150, 200],
                        num_features=[1, 3, 5, 7])
    datafile = sys.argv[1]

    search.grid_search(datafile)
