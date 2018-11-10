/**
  * Weiwei Wan (wan-weiwei@aist.go.jp)
  */

#include "poseestimatenodialog.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"
//#include "../PlacementsHandler/placementshandler.h"

PoseEstimateNoDialog::PoseEstimateNoDialog()
{
    this->readParams();
}

PoseEstimateNoDialog::~PoseEstimateNoDialog()
{

}

/**
 * @brief PoseEstimateNoDialog::readParams
 * directly copied from PoseEstimateDialog class
 */
void PoseEstimateNoDialog::readParams() {
    QString default_path = QString::fromStdString(
                cnoid::executableTopDirectory() +
                "/extplugin/graspPlugin/PCL/");

    QSettings setting(default_path + "config.ini", QSettings::IniFormat);
    setting.beginGroup("ObjPoseEstimate");

    QString show = setting.value("show_scene", "RESULT").toString();

    param.is_load_file  = setting.value("pcd_loadfile", false).toBool();
    param.is_save_file = setting.value("pcd_savefile", false).toBool();
    param.is_load_obj  = setting.value("pcd_loadobj", false).toBool();
    param.is_color = setting.value("show_color", true).toBool();
    param.is_show_segment = (show.toStdString() == "SEGMENT");
    param.is_show_moved = (show.toStdString() == "RESULT");
    param.is_show_scene = (show.toStdString() == "SCENE");
    param.sampling_density =
            setting.value("sampling_density", 0.005).toDouble();
    QDir::setCurrent(default_path);
    param.load_file_path =
            QFileInfo(setting.value("pcd_loadfilepath",
                                    default_path + "PCL/test_pcd.pcd").
                      toString()).absoluteFilePath().toStdString();
    param.save_file_path =
            QFileInfo(setting.value("pcd_savefilepath",
                                    default_path + "PCL/test_pcd.pcd").
                      toString()).absoluteFilePath().toStdString();
    param.obj_file_path =
            QFileInfo(setting.value("pcd_objfilepath",
                                    default_path + "PCL/test.yaml").
                      toString()).absoluteFilePath().toStdString();
    param.normal_radius =
            setting.value("SACIA_radius_normal", 0.01).toDouble();
    param.feature_radius =
            setting.value("SACIA_radius_feature", 0.02).toDouble();
    param.normal_k = setting.value("SACIA_knearest_normal", 10).toInt();
    param.feature_k  = setting.value("SACIA_knearest_feature", 10).toInt();
    param.use_ksearch = setting.value("SACIA_use_knearest", false).toBool();
    param.iteration = setting.value("ICP_iteration", 100).toInt();
    param.plane_remove_dist =
            setting.value("segmentation_distth", 0.005).toDouble();
    param.num_plane_remove = setting.value("segmentation_numplane", 1).toInt();
    param.num_candidate =
            setting.value("segmentation_searchcluster", 1).toInt();
    param.is_do_narrow = setting.value("region_enable", true).toBool();
    param.xmax = setting.value("region_xmax", 1.0).toDouble();
    param.xmin = setting.value("region_xmin", -1.0).toDouble();
    param.ymax = setting.value("region_ymax", 1.0).toDouble();
    param.ymin = setting.value("region_ymin", -1.0).toDouble();
    param.zmax = setting.value("region_zmax", 1.0).toDouble();
    param.zmin = setting.value("region_zmin", -1.0).toDouble();
    param.use_bbsimilarity =
            setting.value("use_boundingbox_similarity", false).toBool();
    param.is_handcamera = setting.value("hand_camera", false).toBool();
    param.do_recog_cvfh = setting.value("method_useCVFH", false).toBool();
    QString viewpoint_str =
            setting.value("viewpoint", "0.0 0.0 0.0").toString();
    QStringList viewpoint_strlist = viewpoint_str.split(" ");
    for (int i = 0; i < 3; i++) {
        param.view_point[i] = viewpoint_strlist.at(i).toDouble();
    }
    param.num_neighbor = setting.value("CVFH_NN", 42).toInt();
    QDir::setCurrent(QString::fromStdString(cnoid::executableTopDirectory()));
    setting.endGroup();
#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    param.do_recog_cvfh = false;
#endif
}

void PoseEstimateNoDialog::estimate()
{
    ObjectPoseEstimator* ope = new ObjectPoseEstimator();
    Segmenter* seg = new Segmenter();
    SAC_IAEstimator* init_est = new SAC_IAEstimator();
    ICPEstimator* est = new ICPEstimator();

    seg->setDistThresholdPlaneRemovement(param.plane_remove_dist);
    seg->setRemovePlaneNum(param.num_plane_remove);

    est->setIteration(param.iteration);
    try {
        //a new piece of pointcloud will be captured in this function
        //if is_load_file is false
        //grabbed_cloud and scene_cloud are set in this function
        ope->readScene(param.is_load_file,
                       param.is_save_file,
                       param.load_file_path,
                       param.save_file_path,
                       param.is_mergecloud,
                       param.is_handcamera);
        if (param.is_load_obj) {
            ope->readObjectVRML(param.obj_file_path);
        } else {
            //the object will be convreted to point cloud and
            //save to obj_cloud in this function
            ope->readObject();
        }
        //ope->makePrimitive();
        ope->setTargetRegion(param.xmax, param.xmin,
                             param.ymax, param.ymin,
                             param.zmax, param.zmin);
        ope->setCandidateNum(param.num_candidate);
        ope->setVoxelLeaf(param.sampling_density);
        ope->useNormalKSearch(param.use_ksearch);
        ope->useFeatureKSearch(param.use_ksearch);
        if(param.use_ksearch) {
            ope->setNormalK(param.normal_k);
            ope->setFeatureK(param.feature_k);
        } else {
            ope->setNormalRadius(param.normal_radius);
            ope->setFeatureRadius(param.feature_radius);
        }
        ope->setSegmenter(seg);
        ope->setInitialEstimator(init_est);
        ope->setEstimator(est);
        //sampling and cropping are performed in this function
        //the cropping is done in the camera frame
        ope->downsampling(param.is_do_narrow);
        if (!(ope->segment())) {
            throw("segmentation failed");
        }

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
        if (param.do_recog_cvfh) {
            cnoid::Vector3 view_point;
            ope->getViewPoint(param.is_handcamera, view_point);
            // core function for estimation
            bool b_selectbest = true;
            ope->cvfhEstimate(param.is_show_moved, view_point,
                              param.obj_file_path, param.num_neighbor,
                              param.use_bbsimilarity, b_selectbest);
        } else {
#endif
            if (!(ope->estimate(param.is_show_moved))) {
                throw("pose estimation failed");
            }
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
        }
#endif

//        this->resetObjWithPlaneConstraints();

        //set collision point cloud
        grasp::PlanBase* pb = grasp::PlanBase::instance();
        pb->RemoveAllPointCloudEnvironment();
        pb->pointCloudEnv.clear();
        vector<Vector3> cloud_vec;
        ObjectPoseEstimator::cloud2Vec(ope->env_cloud, cloud_vec);
        vector<Vector3> normal_vec;
        ObjectPoseEstimator::normal2Vec(ope->env_normal, normal_vec);
        pb->SetPointCloudEnvironment(cloud_vec, normal_vec);

				grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
        for (size_t i = 0; i < opes->size(); i++) {
					opes->at(i).env_point_cloud =
                    pb->pointCloudEnv.back();
        }

        PointCloudDraw* draw = new PointCloudDraw();
        draw->clear();
        if(param.is_show_segment) {
            draw->addPointCloud(seg->getClusteredCloud());
            draw->addPointCloud(seg->getSegmentedCloud(),
                                Vector3(1.0, 0, 0));
            draw->addPointCloud(ope->getSceneCloud());
        } else if(param.is_show_moved) {
            draw->addPointCloud(ope->env_cloud);
            //ope->saveEnv();
            grasp::PlanBase::instance()->flush();
        } else {
            if (param.is_color) {
                draw->addPointCloud(ope->getSceneColorCloud());
            } else {
                draw->addPointCloud(ope->getSceneCloud());
            }
        }
        draw->draw();
        delete draw;
    }
    catch(const char* str) {
        cout << str << endl;
    }

    delete ope;
    delete seg;
    delete est;
    delete init_est;
}

/**
 * @brief PoseEstimateNoDialog::resetObjWithPlaneConstraints
 * This function is NOT used since the mutual invoking between
 * two plugins are invalid
 */
/*
void PoseEstimateNoDialog::resetObjWithPlaneConstraints()
{
    // get the object_link from planbase
    grasp::PlanBase* pb = grasp::PlanBase::instance();
    Link* object_link = pb->targetObject->bodyItemObject->body()->link(0);
    cnoid::Matrix3 objR = object_link->R();
    cnoid::Vector3 objRPY = cnoid::rpyFromRot(objR);

    grasp::PlacementsHandler* ph = grasp::PlacementsHandler::instance();
    std::vector<cnoid::Matrix3> placementRs = ph->getPlacementRs();
    size_t prsSize = placementRs.size();
    double mindist = 2*constpi;
    for(size_t i = 0; i < prsSize; i++) {
        cnoid::Vector3 placementRPY = cnoid::rpyFromRot(placementRs[i]);
        double dist0 = fabs(placementRPY(0)-objRPY(0));
        if(dist0>constpi) {
            dist0=2*constpi-dist0;
        }
        double dist1 = fabs(placementRPY(1)-objRPY(1));
        if(dist0>constpi) {
            dist1=2*constpi-dist1;
        }
        double dist = dist0+dist1;
        if(dist<mindist) {
            cnoid::Vector3 newobjRPY(placementRPY(0), placementRPY(1), objRPY(2));
            cnoid::Matrix3 newobjR = cnoid::rotFromRpy(newobjRPY);
            object_link->R() =  newobjR;
            pb->pose_estimation_sols[0].R = object_link->R();
            mindist = dist;
        }
    }
}
*/
