
// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Kensuke Harada (AIST)
*/

#include <math.h>

#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/LazyCaller>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#ifdef BOOST_FILESYSTEM_VERSION
#undef BOOST_FILESYSTEM_VERSION
#define BOOST_FILESYSTEM_VERSION 3    // filesystem で日本語を使用する場合に定義
#endif
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // for benchmark

#include <pcl/pcl_config.h>

#include "../GeometryHandler/GeometryHandle.h"
#include "CylinderParameter.h"
#include "PCLBar.h"
#include "CalibrateDialog.h"
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#ifdef ENABLE_OSG
#define ENABLE_DRAW
#endif
#else
#define ENABLE_DRAW
#endif
#ifdef ENABLE_DRAW
#include "../Grasp/DrawUtility.h"
#endif
#include <QtGui>
#include <cnoid/SceneView>
#include "gettext.h"

#include "PoseEstimateDialog.h"
#include "poseestimatenodialog.h"
#include "GenerateDescriptorDialog.h"
#include "Calibration.h"
#include "CalibrationDialog.h"

#include "FittingDialog.h"

#include "../Grasp/Camera.h"

#include <cnoid/ExecutablePath>
#define PLUGIN_PATH cnoid::executableTopDirectory() + string("/")

void write_vrml_data(char *file,grasp::ObjectShape *wo);

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
//using namespace grasp::PickAndPlacePlanner;

//#define DISPLAY_APP_VEC

namespace fs = boost::filesystem;

PCLBar* PCLBar::instance()
{
    static PCLBar* instance = new PCLBar();
    return instance;
}

PCLBar::PCLBar()
    : ToolBar("PCLBar"),
      mes(*MessageView::instance()),
      os (MessageView::instance()->cout() )
{
#ifdef CNOID_GE_15
	setVisibleByDefault(true);
#endif
    addSeparator();
    addLabel(_("=PCL="));

    addButton(_("RGB"), _("Display Colored/Uncolored Point Clouds."))->
        sigClicked().connect(bind(&PCLBar::onRGB, this));

    addButton(_("Cap"), _("Capture Point Clouds."))->
        sigClicked().connect(bind(&PCLBar::onCapture, this));

    addButton(_("Seg"), _("Plane Segmentation"))->
        sigClicked().connect(bind(&PCLBar::onSegment, this));

    addButton(_("Tri"), _("Triangulation of Point Clouds."))->
        sigClicked().connect(bind(&PCLBar::onTriangle, this));

    addButton(_("Cyl"), _("Cylinder Segmentation"))->
        sigClicked().connect(bind(&PCLBar::onCylinder, this));

    addButton(_("Zmin"), _("Obtain the lowest point in z direction."))->
        sigClicked().connect(bind(&PCLBar::onCalibrate, this));

    addButton(_("Clear"), _("Clear Point Clouds"))->
        sigClicked().connect(bind(&PCLBar::onClear, this));

    addButton(_("Cube"), _("Capture Point Cloud by Cubes."))->
        sigClicked().connect(bind(&PCLBar::onCube, this));

    addButton(_("Est"), _("Estimate object pose"))->
        sigClicked().connect(bind(&PCLBar::onEst, this));

    addButton(_("EstND"), _("Estimate object pose without dialog"))->
        sigClicked().connect(bind(&PCLBar::onEstNoDiag, this));

    addButton(_("GrabberON/OFF"), _("Start/Stop grabber"))->
        sigClicked().connect(bind(&PCLBar::onGrabSwitch, this));

    addButton(_("Cap2"), _("Capture Point Clouds"))->
        sigClicked().connect(bind(&PCLBar::onCapture2, this));

    addButton(_("Tri2"), _("Triangulation of Point Clouds without smoothing."))->
        sigClicked().connect(bind(&PCLBar::onTriangle2, this));

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
    addButton(_("GenDes"), _("Generate CVFH Descriptors."))->
        sigClicked().connect(bind(&PCLBar::onGendes, this));
#endif
    // addButton(_("Env"), _("Read Env Point Clouds"))->
    // 	sigClicked().connect(bind(&PCLBar::onReadEnv, this));

    addButton(_("Fitting"), _("Fitting boxes/cylinders"))->
        sigClicked().connect(bind(&PCLBar::onFitting, this));

    addButton(_("ReloadCalibMat"), _("Reload the Calibration Matrix"))->
        sigClicked().connect(bind(&PCLBar::onReloadCM, this));

    addSeparator();

	addButton(_("CalibCap"), _("Capture Point Cloud for calibration"))->
        sigClicked().connect(bind(&PCLBar::onCalibCap, this));

	addButton(_("GenCalibMat"), _("Generate the calibration matrix"))->
        sigClicked().connect(bind(&PCLBar::onCalibration, this));

	addSeparator();

    rgbmode = true;

    PointCloudHandler::instance()->leafsize = 0.01;
    PointCloudHandler::instance()->setTransMatrix();
}

PCLBar::~PCLBar()
{
}


bool PCLBar::capture()
{
    PointCloudHandler * pc = PointCloudHandler::instance();
    pc->capture(rgbmode);

    return true;
}

bool PCLBar::segmentate(int mode)
{
    PointCloudHandler::instance()->segmentate(mode);

    return true;
}

bool PCLBar::displayCylinder()
{
#ifdef ENABLE_DRAW
    DrawUtility::instance()->cylinders.clear();

    CylinderFeatures * cf = CylinderFeatures::instance();
    int size = cf->cylinderParamSet.size();

    for(int i=0; i<size; i++){

            Cylinders cyl(cf->cylinderParamSet[i].pos, cf->cylinderParamSet[i].dir, cf->cylinderParamSet[i].radius, cf->cylinderParamSet[i].length);
            DrawUtility::instance()->cylinders.push_back(cyl);
    }

    DrawUtility::instance()->displayCylinders();
#endif
    return true;
}

bool PCLBar::displayPointCloud()
{
#ifdef ENABLE_DRAW
    DrawUtility::instance()->points.clear();
    DrawUtility::instance()->colors.clear();

    PointCloudHandler * pc = PointCloudHandler::instance();

    if(rgbmode){
        os << "Points: " << pc->cloud_xyzrgba->points.size() << endl;

        pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;
        for(it = pc->cloud_xyzrgba->begin();
            it != pc->cloud_xyzrgba->end(); it++) {

					if (!grasp::isnan(it->x) && !grasp::isnan(it->y) && !grasp::isnan(it->z)) {
                // Trans is done in the displaypointcloud function!
                DrawUtility::instance()->points.push_back(
                            pc->Trans(it->x, it->y, it->z));

                uint32_t rgb = *reinterpret_cast<int*>(&it->rgb);
                uint8_t r = (rgb >> 16) & 0x0000ff;
                uint8_t g = (rgb >> 8)  & 0x0000ff;
                uint8_t b = (rgb)       & 0x0000ff;
                DrawUtility::instance()->
                        colors.push_back(Vector3((double)r/256.0,
                                                 (double)g/256.0,
                                                 (double)b/256.0));
            }
        }
    }
    else{
        os << "Points: " << pc->cloud_xyz->points.size() << endl;
        pcl::PointCloud<pcl::PointXYZ>::iterator it;
        for(it = pc->cloud_xyz->begin(); it != pc->cloud_xyz->end(); it++) {
            if(!grasp::isnan(it->x) && !grasp::isnan(it->y) && !grasp::isnan(it->z)) {
                DrawUtility::instance()->points.push_back(
                            pc->Trans(it->x, it->y, it->z));
                DrawUtility::instance()->
                        colors.push_back(Vector3(0.5, 0.5, 0.5));
            }
        }
    }

    DrawUtility::instance()->displayPoints();
#endif
    return true;
}

bool PCLBar::displayApproachVec()
{
#ifdef ENABLE_DRAW
    CylinderFeatures * cy = CylinderFeatures::instance();

    for(size_t i=0; i<cy->cylinderParamSet.size(); i++) {
        for(size_t j=0; j<cy->cylinderParamSet[i].app.size(); j++) {
            DrawUtility::instance()->points.push_back(
                        cy->cylinderParamSet[i].app[j].pos);
            DrawUtility::instance()->colors.push_back(Vector3(1,0,0));

            Vector3 v2 = cy->cylinderParamSet[i].app[j].pos-
                    cy->cylinderParamSet[i].app[j].dir*0.1;
            DrawUtility::instance()->points.push_back(v2);
            DrawUtility::instance()->colors.push_back(Vector3(1,0,0));
        }
    }

    DrawUtility::instance()->displayLines();
#endif
    return true;
}



bool PCLBar::displayPolygon(){

    PointCloudHandler * pc = PointCloudHandler::instance();

    os << "cloud_xyz: " << pc->cloud_xyz->size() << endl;
    os << "cloud_xyz: " << pc->triangles.polygons.size() << endl;

    ColdetModelPtr coldetModel(new ColdetModel());

    coldetModel->setNumVertices(pc->cloud_xyz->size());
    coldetModel->setNumTriangles(pc->triangles.polygons.size());

    int vertexIndex = 0;
    int triangleIndex = 0;

    vector<double> vertex;
    vector<int> crd;


    for(size_t i=0; i<pc->cloud_xyz->size(); i++){

        pcl::PointXYZ at = pc->cloud_xyz->at(i);
#ifdef ENABLE_DRAW
        Vector3 v2 = pc->Trans(at.x, at.y,at.z);
#else
        Vector3 v2 = Vector3(at.x,at.y,at.z);
#endif
        coldetModel->setVertex(vertexIndex++, (float)v2[0], (float)v2[1],(float)v2[2]);
        // kinect と Choreonoid の座標系の単位を揃えるために、v*1000
        vertex.push_back(v2[0]*1000);
        vertex.push_back(v2[1]*1000);
        vertex.push_back(v2[2]*1000);
    }

    for (size_t i=0; i<pc->triangles.polygons.size(); ++i) {
        vector<int> idx;
        for (size_t j=0; j<pc->triangles.polygons[i].vertices.size(); ++j) {
            idx.push_back( pc->triangles.polygons[i].vertices[j] );
        }
        coldetModel->setTriangle(triangleIndex++, idx[0], idx[1], idx[2] );
        crd.push_back(idx[0]);
        crd.push_back(idx[1]);
        crd.push_back(idx[2]);
        crd.push_back(-1);
    }

    coldetModel->setName("Env");
    coldetModel->build();

    ObjectShape shape(vertex, crd);
    string wrlfile = PLUGIN_PATH + "extplugin/graspPlugin/PCL/models/coldetmodel.wrl";
    write_vrml_data((char*)wrlfile.c_str(), &shape);
    string wrlhrpfile = PLUGIN_PATH + "extplugin/graspPlugin/PCL/models/coldetmodelhrp.wrl";
    BodyItemPtr item(new BodyItem());
    if(item->load(wrlhrpfile, "OpenHRP-VRML-MODEL")) {
        RootItem::mainInstance()->addChildItem(item);
        ItemTreeView::mainInstance()->checkItem(item, true);
    }

    return true;
}

void PCLBar::onRGB()
{
    rgbmode = !rgbmode;

    if(rgbmode)
        os << "Colored point cloud" << endl;
    else
        os << "Uncolored point cloud" << endl;

    onClear();
}

void PCLBar::onClear()
{
#ifdef ENABLE_DRAW
    os <<  "Clear PCL" << endl;

    PointCloudHandler * pc = PointCloudHandler::instance();
    pc->cloud_xyz->clear();
    pc->cloud_xyzrgba->clear();

    DrawUtility::instance()->clear();
#endif
}

void PCLBar::onCapture()
{
	setTransMatrix();
    os <<  "Capture PCL" << endl;
    if (capture()) {
        displayPointCloud();
        os <<  "OK." << endl;
    } else {
        os <<  "NG." << endl;
    }
}

void PCLBar::onSegment()
{
    PointCloudHandler::instance()->leafsize = 0.001;
    PointCloudHandler::instance()->minCloudDistance = 0.02;

    os <<  "Segmentate PCL" << endl;
    if (segmentate(0)) {
        displayPointCloud();
        os <<  "OK." << endl;
    } else {
        os <<  "NG." << endl;
    }
}

void PCLBar::onEst()
{
    PoseEstimateDialog* peDialog = new PoseEstimateDialog();
//    if(capture()) {
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
//                cloud_output(new pcl::PointCloud<pcl::PointXYZRGBA>());
//        PointCloudHandler* ppc = PointCloudHandler::instance();
//        ppc->passThroughFilterRGBA(ppc->cloud_xyzrgba, cloud_output,
//                               -0.4, 0.4, string("x"));
//        ppc->passThroughFilterRGBA(cloud_output, ppc->cloud_xyzrgba,
//                               -0.4, 0.4, string("y"));
//        ppc->passThroughFilterRGBA(ppc->cloud_xyzrgba, cloud_output,
//                               0.5, 1.2, string("z"));
//        pcl::copyPointCloud(*cloud_output, *(ppc->cloud_xyzrgba));
//        displayPointCloud();
//    }
    peDialog->exec();
    delete peDialog;
}

void PCLBar::onEstNoDiag()
{
    PoseEstimateNoDialog* peND= PoseEstimateNoDialog::instance();
    // TODO
    // the bodyitems that correspond to the newly found objects
    // should be cleared at each estimation
    peND->estimate();
//    if(capture()) {
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
//                cloud_output(new pcl::PointCloud<pcl::PointXYZRGBA>());
//        PointCloudHandler* ppc = PointCloudHandler::instance();
//        ppc->passThroughFilterRGBA(ppc->cloud_xyzrgba, cloud_output,
//                               -0.4, 0.4, string("x"));
//        ppc->passThroughFilterRGBA(cloud_output, ppc->cloud_xyzrgba,
//                               -0.4, 0.4, string("y"));
//        ppc->passThroughFilterRGBA(ppc->cloud_xyzrgba, cloud_output,
//                               0.5, 1.2, string("z"));
//        pcl::copyPointCloud(*cloud_output, *(ppc->cloud_xyzrgba));
//        displayPointCloud();
//    }

}

void PCLBar::onGrabSwitch()
{
    PointCloudHandler *pc = PointCloudHandler::instance();
    if(pc->isGrabberActive()){
        pc->stopGrabber();
        os << "Stop grabber" << endl;
    }else{
        pc->startGrabber(rgbmode);
        os << "Start grabber" << endl;
    }
}

void PCLBar::onCapture2()
{
	setTransMatrix();
    PointCloudHandler *pc = PointCloudHandler::instance();
    pc->captureWithoutInit(rgbmode);
    displayPointCloud();
}

void PCLBar::onReadEnv()
{
    ObjectPoseEstimator* ope = new ObjectPoseEstimator();
    ope->readEnv();
    PointCloudDraw* draw = new PointCloudDraw();
    draw->addPointCloud(ope->env_cloud);
    draw->draw();
    grasp::PlanBase::instance()->pointCloudEnv.clear();
        vector<Vector3> cloud_vec;
        ObjectPoseEstimator::cloud2Vec(ope->env_cloud, cloud_vec);
        vector<Vector3> normal_vec;
        ObjectPoseEstimator::normal2Vec(ope->env_normal, cloud_vec);
        grasp::PlanBase::instance()->
                SetPointCloudEnvironment(cloud_vec, normal_vec);
    delete ope;
    delete draw;

}

void PCLBar::onCylinder()
{
    PointCloudHandler *pc = PointCloudHandler::instance();
    CylinderFeatures *cf = CylinderFeatures::instance();

    cf->cylinderParamSet.clear();

    Vector3 cameraPos = pc->t3 + (pc->R3)*(pc->t2 + (pc->R2)*(pc->t1));
    Vector3 vertical = trans(Matrix3((pc->R3)*(pc->R2)*(pc->R1)))*Vector3(0,0,1);

    pc->leafsize = 0.005;
    pc->minSensorDistance = 0.5;
    pc->maxSensorDistance = 1.0;
    pc->cameraPos << cameraPos(0), -cameraPos(2), cameraPos(1);
    pc->vertical  << vertical(0),  -vertical(2),  vertical(1);
    pc->minCloudDistance = 0.01;

    //Lemon eggplantan
/*
    pc->radiusTolerance = 0.04;
    pc->minRadius = 0.02;
    pc->maxRadius = 0.045;
    pc->desRadius = 0.035;
    pc->minLength = 0.01;
    pc->maxLength = 0.3;
*/
    //Banana
    pc->minCloudDistance = 0.005;
    pc->radiusTolerance = 0.025;
    pc->minRadius = 0.01;
    pc->maxRadius = 0.035;
    pc->desRadius = 0.0185;
    pc->minLength = 0.03;
    pc->maxLength = 0.3;

    //2Cup
/*
    pc->radiusTolerance = 0.04;
    pc->minRadius = 0.03;
    pc->maxRadius = 0.05;
    pc->desRadius = 0.05;
    pc->minLength = 0.005;
    pc->maxLength = 0.07;
*/

    os <<  "Segmentate PCL" << endl;
    if (segmentate(3)) {

            for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = pc->cloud_xyzrgba->begin(); it != pc->cloud_xyzrgba->end(); ++it)
                    PlanBase::instance()->pointCloud.push_back(pc->Trans(it->x, it->y, it->z));

            for(size_t i=0; i<cf->cylinderParamSet.size(); i++){

                    Vector3 c = cf->cylinderParamSet[i].pos;
                    Vector3 d = cf->cylinderParamSet[i].dir;

                    cf->cylinderParamSet[i].pos = pc->Trans(c(0), c(1), c(2));
                    cf->cylinderParamSet[i].dir = pc->Dir(d(0), d(1), d(2));
            }
            cf->selectGraspedObject(PlanBase::instance()->pointCloud);

#ifdef DISPLAY_APP_VEC
            displayApproachVec();
#endif
            displayPointCloud();
//			displayCylinder();
            os <<  "OK." << endl;
    } else {
            os <<  "NG." << endl;
    }


#if 0
    for(size_t i=0; i<CylinderFeatures::instance()->cylinderParamSet.size(); i++){
            Vector3 p = CylinderFeatures::instance()->cylinderParamSet[i].pos;
            cout << PointCloudHandler::instance()->Trans(p(0), p(2), -p(1)).transpose() << "/";
            Vector3 d = CylinderFeatures::instance()->cylinderParamSet[i].dir;
            cout << PointCloudHandler::instance()->Dir(d(0), d(2), -d(1)).transpose() << endl;
    }
#endif
}

void PCLBar::onTriangle()
{
    PointCloudHandler::instance()->leafsize = 0.01;

    os <<  "Trianglation PC" << endl;
    boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();

    if(segmentate(2))
            displayPolygon();

    boost::posix_time::ptime end = boost::posix_time::second_clock::local_time();
    os << end - start << endl;

}

void PCLBar::onTriangle2()
{
    PointCloudHandler::instance()->leafsize = 0.01;

    os <<  "Trianglation PC" << endl;
    boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();

    if(segmentate(3))
            displayPolygon();

    boost::posix_time::ptime end = boost::posix_time::second_clock::local_time();
    os << end - start << endl;

}

void PCLBar::onGendes()
{
    GenDescriptorDialog* gDialog = new GenDescriptorDialog();
    gDialog->exec();
    delete gDialog;
}

void PCLBar::onFitting()
{
    PlanBase* pb = PlanBase::instance();
    if (pb->targetObject == NULL) {
        os << "Please select target Object!" << endl;
        return;
    }
    FittingDialog* fDialog = new FittingDialog();
    fDialog->setAttribute(Qt::WA_DeleteOnClose);
    connect(fDialog, SIGNAL(finished(int)), fDialog, SLOT(deleteLater()));
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    fDialog->setYamlPath(pb->targetObject->bodyItemObject->lastAccessedFilePath());
#else
    fDialog->setYamlPath(pb->targetObject->bodyItemObject->filePath());
#endif
    fDialog->show();
}

void PCLBar::onReloadCM()
{
    this->onClear();
    PointCloudHandler::instance()->setTransMatrix();
}

void PCLBar::onCalibCap()
{
	Calibration::capture();
}

void PCLBar::onCalibration()
{
	std::string mat_path = Calibration::getDefaultMatrixFilePath();
	QString filepath = QFileDialog::getSaveFileName(this, tr("Save file as"), QString::fromStdString(mat_path), tr("Calibration matrix (*.txt)"));
	if (!filepath.isEmpty()) {
		CalibrationDialog* cDialog = new CalibrationDialog();
		cDialog->setAttribute(Qt::WA_DeleteOnClose);
		connect(cDialog, SIGNAL(finished(int)), cDialog, SLOT(deleteLater()));
		cDialog->setMatFilePath(filepath.toStdString());
		cDialog->show();
	}
}

void PCLBar::onCalibrate()
{
#ifdef ENABLE_DRAW
    os <<  "Calibration" << endl;
    CalibrateDialog dlg;
    std::ifstream fin("calib.txt");
    if (fin.fail() == false) {
            float xmin, xmax, ymin, ymax, zmin, zmax;
            fin >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;
            dlg.setXMin(xmin);
            dlg.setXMax(xmax);
            dlg.setYMin(ymin);
            dlg.setYMax(ymax);
            dlg.setZMin(zmin);
            dlg.setZMax(zmax);
    }
    if (dlg.exec()) {
            float xmin = dlg.getXMin();
            float xmax = dlg.getXMax();
            float ymin = dlg.getYMin();
            float ymax = dlg.getYMax();
            float zmin = dlg.getZMin();
            float zmax = dlg.getZMax();
            os << xmin << " < x < " << xmax << endl;
            os << ymin << " < y < " << ymax << endl;
            os << zmin << " < z < " << zmax << endl;
            std::ofstream fout("calib.txt");
            fout << xmin << endl << xmax << endl;
            fout << ymin << endl << ymax << endl;
            fout << zmin << endl << zmax << endl;

            bool rgbmode_org = rgbmode;
            rgbmode = false;

            DrawUtility::instance()->points.clear();
            DrawUtility::instance()->colors.clear();

            if (capture()) {

                    PointCloudHandler * pc = PointCloudHandler::instance();
                    int size = pc->cloud_xyz->points.size();
                    if(size==0) return;


                    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pc->cloud_xyz->begin(); it != pc->cloud_xyz->end(); it++)
                            if (!grasp::isnan(it->x) && !grasp::isnan(it->y) && !grasp::isnan(it->z)){
                                    DrawUtility::instance()->points.push_back(Vector3(it->x, it->z, -it->y));
                            }

                    //------------------

                    Calibrator calib(xmin, xmax, ymin, ymax, zmin, zmax);
                    Comparator lesser;

                    cout << "Original Points: " << DrawUtility::instance()->points.size() << endl;
                    vector<Vector3>::iterator rit = remove_if(DrawUtility::instance()->points.begin(), DrawUtility::instance()->points.end(), calib);
                    DrawUtility::instance()->points.erase(rit, DrawUtility::instance()->points.end());
                    rit = unique(DrawUtility::instance()->points.begin(), DrawUtility::instance()->points.end());
                    DrawUtility::instance()->points.erase(rit, DrawUtility::instance()->points.end());
                    cout << "Reduced Points: " << DrawUtility::instance()->points.size() << endl;

                    for(size_t i=0; i<DrawUtility::instance()->points.size(); i++)
                        DrawUtility::instance()->colors.push_back(Vector3(0.5, 0.5, 0.5));

                    DrawUtility::instance()->displayPoints();

                    rit = min_element(DrawUtility::instance()->points.begin(), DrawUtility::instance()->points.end(), lesser);
                    cout << "Min: " << rit->x() << ", " << rit->y() << ", " << rit->z() << endl;

                    BodyItemPtr item(new BodyItem());
                    string wrlhrpfile = PLUGIN_PATH + "extplugin/graspPlugin/Samples/Object/W0Hrp.wrl";
                    item->load(wrlhrpfile, "OpenHRP-VRML-MODEL");
                    addCube(rit->x(), rit->y(), rit->z(), item);

        }
        rgbmode = rgbmode_org;
    }
#endif
}


void PCLBar::onCube()
{
    os <<  "Capture PCL" << endl;
    bool rgbmode_org = rgbmode;
    rgbmode = false;

    PointCloudHandler * pc = PointCloudHandler::instance();

    using boost::gregorian::date;
    using boost::gregorian::day_clock;
    using boost::posix_time::ptime;
    using boost::posix_time::second_clock;

    if (capture()) {

        //displayPointCloud();

        os << "Points: " << pc->cloud_xyz->size() << endl;
        int size = pc->cloud_xyz->size();

        if(size==0) return;

        BodyItemPtr item(new BodyItem());
        string wrlhrpfile = PLUGIN_PATH + "extplugin/graspPlugin/Samples/Object/W0Hrp.wrl";
        item->load(wrlhrpfile, "OpenHRP-VRML-MODEL");

        ptime start = second_clock::local_time();

        for (int i = 1; i < 1000; i++) {
            if (i%100 == 0) cout << i << endl;
            int p = rand()%size;
            pcl::PointXYZ pt = pc->cloud_xyz->at(p);
            if (!grasp::isnan(pt.x) && !grasp::isnan(pt.y) && !grasp::isnan(pt.z)) {
                Vector3 v2 = pc->Trans(pt.x, pt.y, pt.z);
                addCube(v2(0), v2(1), v2(2), item);
            }
        }
        ptime end = second_clock::local_time();
        os << end - start << endl;
    }
    os <<  "Ok." << endl;
    rgbmode = rgbmode_org;
}

void
PCLBar::addCube(float x, float y, float z, BodyItemPtr item_)
{
    ItemPtr item = item_->duplicateAll();
    RootItem::mainInstance()->addChildItem(item);

    BodyItem* btemp = (BodyItem*)(item.get());
    btemp->body()->link(0)->R() = Matrix3::Identity();
    btemp->body()->link(0)->p() = Vector3(x, y, z);
    ItemTreeView::mainInstance()->checkItem(btemp,true);

    PlanBase::instance()->SetEnvironment(btemp);

}

void PCLBar::setTransMatrix()
{
		grasp::CameraPtr camera = grasp::CameraHandler::instance()->getTargetCamera();
		PointCloudHandler* pc = PointCloudHandler::instance();
		pc->t1 = camera->getCameraPosition();
		pc->t2 = cnoid::Vector3::Zero();
		pc->t3 = cnoid::Vector3::Zero();
		pc->R1 = camera->getCameraRotation();
		pc->R2 = cnoid::Matrix3::Identity();
		pc->R3 = cnoid::Matrix3::Identity();
}
