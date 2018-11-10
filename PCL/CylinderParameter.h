#ifndef _CYLINDERPARAM_H
#define _CYLINDERPARAM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>

#include <Eigen/Core>
#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(<, 1, 7, 0) && EIGEN_VERSION_AT_LEAST(3, 2, 0)
#include "EigenInternalMath.h"
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include "LeastSquare.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/VectorMath.h"
#include "../GeometryHandler/ClusterParameter.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/SceneShape>
#include "../Grasp/ColdetConverter.h"
#endif

//#define OBJECT_DISTRIBUTION_MODE

class Segment
{
public:
	int start, end;
	LeastSquare ls;
};

class ApproachParameter
{
public:
	cnoid::Vector3 dir;
	cnoid::Vector3 edge; //edge(0):cylinder axis dir, edge(1):cylinder radius dir, edge(2):approach distance
	cnoid::Vector3 pos;
	double length;
	std::vector<Segment> seg;
	std::vector<cnoid::Vector3> pt;
	double weight;
};

class CylinderParameter
{
public:
	CylinderParameter();

	cnoid::Vector3 pos;
	cnoid::Vector3 dir;
	cnoid::Vector3 com;
	double radius;
	double length;
	double stdDev;
	int connection[2];
	void setCylinderParameter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::ModelCoefficients::Ptr coefficient);
	void calcPointCloudDistribution(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
	void setDesRadius(double desRadius);

	std::vector<ApproachParameter> app;
	cnoid::Vector3 appEdge;
	double cpInterval;
};

class CylinderFeatures
{

public :
	CylinderFeatures();

	static CylinderFeatures* instance();
	bool extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::ModelCoefficients::Ptr coeffient, double desRadius);
	bool isSparseCloud(CylinderParameter& par, pcl::PointCloud<pcl::PointXYZ>::Ptr input, double ratio);
	bool includeHiddenPoints(CylinderParameter& par, pcl::PointCloud<pcl::PointXYZ>::Ptr input, double ratio);
	bool checkOverlap(CylinderParameter& par1, CylinderParameter& par2, double ratio);
	void selectGraspedObject(const std::vector<cnoid::Vector3>& pointCloud);
	void connectCylinders();
	void write_cylinder_vrml(std::string file, CylinderParameter& par);
	void writeYamlFile(int i, std::vector<grasp::ClusterParameter>& objClusters, std::vector<grasp::ClusterParameter>& envClusters, double r, double h );
	void calcPointCloudDistribution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl, pcl::ModelCoefficients::Ptr coef, double desRadius);
	std::vector<CylinderParameter> cylinderParamSet;
	std::vector<int> idList;
	double minLength, maxLength;
	cnoid::Vector3 cameraPos;
	bool cylinderGenerated;

	cnoid::Vector3 t1_, t2_;
	cnoid::Matrix3 R1_, R2_;

protected:
	int data_num;

};

#endif
