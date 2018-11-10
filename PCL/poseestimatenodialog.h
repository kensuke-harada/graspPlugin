/**
  * Weiwei Wan (wan-weiwei@aist.go.jp)
  */

#ifndef POSEESTIMATENODIALOG_H
#define POSEESTIMATENODIALOG_H

#include "PoseEstimateDialog.h"
#include "ObjectPoseEstimator.h"
#include "Registrator.h"
#include "Segmenter.h"
#include "ObjectPoseEstimatorInterface.h"
#include "PointCloudDrawer.h"
using namespace std;
using namespace cnoid;
using namespace grasp;
namespace fs = boost::filesystem;

class PoseEstimateNoDialog
{
public:
    static PoseEstimateNoDialog* instance() {
        static PoseEstimateNoDialog* instance =
                new PoseEstimateNoDialog();
        instance->readParams();
        return instance;
    }

public:
    PoseEstimateNoDialog();
    ~PoseEstimateNoDialog();

public:
    void readParams();
    void estimate();

private:
    void resetObjWithPlaneConstraints();

public:
    PoseEstimateDialog::Params param;
};

#endif // POSEESTIMATENODIALOG_H
