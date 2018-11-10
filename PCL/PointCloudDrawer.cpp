/**
 * @file   PointCloudDrawer.cpp
 * @author Akira Ohchi
*/

#include "PointCloudDrawer.h"

#include <vector>
#include <iostream>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#ifdef ENABLE_OSG
#define ENABLE_DRAW
#endif
#else
#define ENABLE_DRAW
#endif

#include "../Grasp/DrawUtility.h"

class PointCloudDrawer::PointCloudDrawerImpl {
 public:
	PointCloudDrawerImpl() :
		cloud_(new ColorPointCloud),
		model_alpha_(1.0) {;}
	~PointCloudDrawerImpl() {;}
	void draw();
	void clear();
	void addPointCloud(PointCloudTConstPtr cloud, const cnoid::Vector3& color);
	void addPointCloud(ColorPointCloudConstPtr cloud);
	void addModel(cnoid::ColdetModelPtr model, const cnoid::Vector3& color);

	ColorPointCloudPtr cloud_;
	std::vector<cnoid::ColdetModelPtr> models_;
	std::vector<cnoid::Vector3> model_colors_;
	double model_alpha_;

#ifdef ENABLE_DRAW
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	std::vector<cnoid::OSGSceneObjectPtr> obj_list_;
#else
	std::vector<cnoid::SgGroupPtr> obj_list_;
#endif
#endif
};

void PointCloudDrawer::PointCloudDrawerImpl::draw() {
#ifdef ENABLE_DRAW
	clear();
	for (ColorPointCloudConstIterator it = cloud_->begin(); it != cloud_->end(); ++it) {
		DrawUtility::instance()->points.push_back(Vector3(it->x, it->y, it->z));
		Vector3 rgb = it->getRGBVector3i().cast<double>() / 255.0;
		DrawUtility::instance()->colors.push_back(rgb);
	}

	cloud_->clear();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cnoid::OSGSceneObjectPtr obj;
#else
	cnoid::SgGroupPtr obj;
#endif
	DrawUtility::instance()->displayPoints(obj);
	obj_list_.push_back(obj);

	DrawUtility::instance()->points.clear();
	DrawUtility::instance()->colors.clear();
	int t[3];
	int idx_offset = 0;
	for (size_t i = 0; i < models_.size(); i++) {
		cnoid::ColdetModelPtr target_model = models_[i];

		int n_ver = target_model->getNumVertices();
		for (int j = 0; j < n_ver; j++) {
			float tx, ty, tz;
			target_model->getVertex(j, tx, ty, tz);
			DrawUtility::instance()->points.push_back(cnoid::Vector3(tx, ty, tz));
		}

		for(int j = 0; j < target_model->getNumTriangles(); j++) {
			std::vector<int> t(3);
			target_model->getTriangle(j, t[0], t[1], t[2]);
			for (int k = 0; k < 3; k++) {
				t[k] += idx_offset;
			}
			DrawUtility::instance()->triangles.push_back(t);
			DrawUtility::instance()->colors.push_back(model_colors_[i]);
		}
		idx_offset += n_ver;
	}
	if (!models_.empty()) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		cnoid::OSGSceneObjectPtr obj2;
#else
		cnoid::SgGroupPtr obj2;
#endif
		DrawUtility::instance()->displayTriangles(model_alpha_, obj2);
		obj_list_.push_back(obj2);
	}

	models_.clear();
	model_colors_.clear();
#endif
}

void PointCloudDrawer::PointCloudDrawerImpl::clear() {
#ifdef ENABLE_DRAW
	DrawUtility::instance()->points.clear();
	DrawUtility::instance()->colors.clear();
	DrawUtility::instance()->triangles.clear();
	for (size_t i = 0; i < obj_list_.size(); i++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		obj_list_[i]->hide();
#endif
		DrawUtility::instance()->deleteanobj(obj_list_[i]);
	}
	obj_list_.clear();
#endif
}

void PointCloudDrawer::PointCloudDrawerImpl::addPointCloud(PointCloudTConstPtr cloud, const cnoid::Vector3& color) {
	for (PointCloudTConstIterator it = cloud->begin(); it != cloud->end(); ++it) {
		pcl::PointXYZRGBA p;
		p.x = it->x;
		p.y = it->y;
		p.z = it->z;
		uint8_t r = floor(255 * color(0));
		uint8_t g = floor(255 * color(1));
		uint8_t b = floor(255 * color(2));
		uint32_t rgb = ((uint32_t) r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		p.rgb = *reinterpret_cast<float*>(&rgb);

		cloud_->push_back(p);
	}
}

void PointCloudDrawer::PointCloudDrawerImpl::addPointCloud(ColorPointCloudConstPtr cloud) {
	*cloud_ += *cloud;
}

void PointCloudDrawer::PointCloudDrawerImpl::addModel(cnoid::ColdetModelPtr model, const cnoid::Vector3& color) {
	models_.push_back(model);
	model_colors_.push_back(color);
}

PointCloudDrawer::PointCloudDrawer() :
	impl(new PointCloudDrawerImpl()) {
}

PointCloudDrawer::~PointCloudDrawer() {
}

PointCloudDrawer* PointCloudDrawer::instance() {
	static PointCloudDrawer* instance = new PointCloudDrawer();
	return instance;
}

/**
* Draw the point cloud.
*/
void PointCloudDrawer::draw() {
	impl->draw();
}

/**
* Clear the point cloud.
*/
void PointCloudDrawer::clear() {
	impl->clear();
}

/**
* Add the point cloud to be drawn.
* @param[in] a pointer to the point cloud.
*/
void PointCloudDrawer::addPointCloud(PointCloudTConstPtr cloud) {
	impl->addPointCloud(cloud, cnoid::Vector3(0.5, 0.5, 0.5));
}

/**
* Add the point cloud to be drawn.
* @param[in] a pointer to the point cloud.
* @param[in] color
*/
void PointCloudDrawer::addPointCloud(PointCloudTConstPtr cloud, const cnoid::Vector3& color) {
	impl->addPointCloud(cloud, color);
}

/**
* Add the point cloud to be drawn.
* @param[in] a pointer to the point cloud.
*/
void PointCloudDrawer::addPointCloud(ColorPointCloudConstPtr cloud) {
	impl->addPointCloud(cloud);
}

void PointCloudDrawer::addModel(cnoid::ColdetModelPtr model, const cnoid::Vector3& color) {
	impl->addModel(model, color);
}

void PointCloudDrawer::setModelAlpha(double alpha) {
	impl->model_alpha_ = alpha;
}
