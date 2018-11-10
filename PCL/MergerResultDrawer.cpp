#include "MergerResultDrawer.h"

#include "PointCloudDrawer.h"
#include "PointCloudHolder.h"

MergerResultDrawer::MergerResultDrawer() :
	prev_cloud_(new PointCloudT()),
	curr_cloud_(new PointCloudT()),
	prev_sampled_cloud_(new PointCloudT()),
	curr_sampled_cloud_(new PointCloudT()),
	cluster_cloud_(new ColorPointCloud()) {
}

MergerResultDrawer::~MergerResultDrawer() {
}

MergerResultDrawer* MergerResultDrawer::instance() {
	static MergerResultDrawer* instance = new MergerResultDrawer();
	return instance;
}

void MergerResultDrawer::drawClusters() {
	if (cluster_cloud_->empty()) return;
	if (curr_cloud_->empty()) return;
	PointCloudDrawer* draw = PointCloudDrawer::instance();
	draw->addPointCloud(cluster_cloud_);
	draw->addPointCloud(curr_cloud_);
	draw->addPointCloud(BoxCloudHolder::instance()->getBoxPtr(), cnoid::Vector3(0, 1, 0));
	draw->draw();
}

void MergerResultDrawer::drawPrevAndCurrPointCloud() {
	PointCloudDrawer* draw = PointCloudDrawer::instance();
	if (!prev_cloud_->empty()) {
		draw->addPointCloud(prev_cloud_, cnoid::Vector3(0.5, 0.8, 0.5));
	}
	draw->addPointCloud(curr_cloud_);

	for (size_t i = 0; i < models_.size(); i++) {
		draw->addModel(models_[i], model_colors_[i]);
	}
	draw->setModelAlpha(0.2);
	draw->draw();
}

void MergerResultDrawer::updatePointCloud(PointCloudTConstPtr cloud, ColorPointCloudPtr cluster_cloud, PointCloudTConstPtr sampled_cloud,
																					const std::vector<cnoid::ColdetModelPtr>& models, const std::vector<cnoid::Vector3>& model_colors,
																					const std::vector<PointCloudTPtr>& model_points) {
	pcl::copyPointCloud(*cluster_cloud, *cluster_cloud_);
	pcl::copyPointCloud(*curr_cloud_, *prev_cloud_);
	pcl::copyPointCloud(*cloud, *curr_cloud_);
	pcl::copyPointCloud(*curr_sampled_cloud_, *prev_sampled_cloud_);
	pcl::copyPointCloud(*sampled_cloud, *curr_sampled_cloud_);
	models_ = models;
	model_colors_ = model_colors;
	model_points_ = model_points;
}

PointCloudTPtr MergerResultDrawer::getPrevSampledCloud() const {
	return prev_sampled_cloud_;
}

PointCloudTPtr MergerResultDrawer::getCurrSampledCloud() const {
	return curr_sampled_cloud_;
}

ColorPointCloudPtr MergerResultDrawer::getClusterCloud() const {
	return cluster_cloud_;
}

void MergerResultDrawer::getClusterClouds(std::vector<PointCloudTPtr>& clouds) {
	clouds = model_points_;
}
