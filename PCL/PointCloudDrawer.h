/**
 * @file   PointCloudDrawer.h
 * @author Akira Ohchi
*/

#ifndef _PCL_POINTCLOUDDRAWER_H_
#define _PCL_POINTCLOUDDRAWER_H_

#include <boost/scoped_ptr.hpp>

#include <cnoid/EigenTypes>
#include <cnoid/ColdetModel>

#include "PointCloudTypes.h"

class PointCloudDrawer {
 public:
	static PointCloudDrawer* instance();
	virtual ~PointCloudDrawer();

	void draw();
	void clear();
	void addPointCloud(PointCloudTConstPtr cloud);
	void addPointCloud(PointCloudTConstPtr cloud, const cnoid::Vector3& color);
	void addPointCloud(ColorPointCloudConstPtr cloud);
	void addModel(cnoid::ColdetModelPtr model, const cnoid::Vector3& color);
	void setModelAlpha(double alpha);

 private:
	PointCloudDrawer();

	class PointCloudDrawerImpl;
	boost::scoped_ptr<PointCloudDrawerImpl> impl;
};

/** @deprecated */
class PointCloudDraw {
public:
	PointCloudDraw(){return;}
	virtual ~PointCloudDraw(){;}

	void draw() {PointCloudDrawer::instance()->draw();}
	void clear() {PointCloudDrawer::instance()->clear();}
	void addPointCloud(PointCloudTConstPtr _cloud) {PointCloudDrawer::instance()->addPointCloud(_cloud);}
	void addPointCloud(PointCloudTConstPtr _cloud, const cnoid::Vector3& color) {PointCloudDrawer::instance()->addPointCloud(_cloud, color);}
	void addPointCloud(ColorPointCloudConstPtr _cloud) {PointCloudDrawer::instance()->addPointCloud(_cloud);}
};

#endif /* _PCL_POINTCLOUDDRAWER_H_ */
