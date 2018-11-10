#include "PointCloudHandlerInterface.h"
#include "PointCloudHandler.h"

using namespace std;
using namespace cnoid;

void PointCloudHandlerInterface::cylinderSegmentation(const ColdetModelPtr object, vector<Cylinder>& cylinders, double dist_th, int max_num) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	PointCloudHandler::convertColdetModel2PointCloud(object, cloud);

	vector<Vector3> pos, dir;
	vector<double> radius, len;
	PointCloudHandler::instance()->radiusTolerance = dist_th;
	PointCloudHandler::instance()->cylidnerSegmentationPointCloud(cloud, pos, dir, radius ,len, max_num);

	cylinders.clear();
	for (unsigned int i = 0; i < pos.size(); i++) {
		Cylinder cy;
		cy.pos = pos[i];
		cy.dir = dir[i];
		cy.len = len[i];
		cy.radius = radius[i];
		cylinders.push_back(cy);
	}
}
