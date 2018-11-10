#ifndef _PCL_MERGERRESULTDRAWER_H_
#define _PCL_MERGERRESULTDRAWER_H_

#include <vector>
#include <cnoid/EigenTypes>
#include <cnoid/ColdetModel>

#include "PointCloudTypes.h"

class MergerResultDrawer {
 public:
	MergerResultDrawer();
	~MergerResultDrawer();

	static MergerResultDrawer* instance();

	void drawClusters();
	void drawPrevAndCurrPointCloud();

	void updatePointCloud(PointCloudTConstPtr cloud, ColorPointCloudPtr cluster_cloud, PointCloudTConstPtr sampled_cloud,
												const std::vector<cnoid::ColdetModelPtr>& models, const std::vector<cnoid::Vector3>& model_colors,
												const std::vector<PointCloudTPtr>& model_points);

	PointCloudTPtr getPrevSampledCloud() const;
	PointCloudTPtr getCurrSampledCloud() const;
	ColorPointCloudPtr getClusterCloud() const;
	void getClusterClouds(std::vector<PointCloudTPtr>& clouds);

 private:
	PointCloudTPtr prev_cloud_;
	PointCloudTPtr curr_cloud_;
	PointCloudTPtr prev_sampled_cloud_;
	PointCloudTPtr curr_sampled_cloud_;
	ColorPointCloudPtr cluster_cloud_;
	std::vector<cnoid::ColdetModelPtr> models_;
	std::vector<cnoid::Vector3> model_colors_;
	std::vector<PointCloudTPtr> model_points_;
};

#endif /* _PCL_MERGERRESULTDRAWER_H_ */
