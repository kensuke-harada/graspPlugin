from cnoid.Util import *
from cnoid.Base import *
from cnoid.PickAndPlacePlannerPlugin import *
import glob
import os.path


class LearningDataManipulator:
    def __init__(self):
        self.g = GraspReconstractor.instance()
        self.datafiles = executableTopDirectory() + "/data/*_angle.txt"

    def reconstract(self, index):
        self.g.reconstract(index)
        self.g.showPointCloud()
        self.g.computeInsidePoints(index, False, True)

    def extract(self, index):
        if not (self.g.reconstract(index)):
            print "fail: " + str(index)
            return
        self.g.computeInsidePoints(index, True, False)

    def extract_all(self):
        files = glob.glob(self.datafiles)
        for file in files:
            print "processing:" + file
            self.extract(int(os.path.basename(file).split("_")[0]))
