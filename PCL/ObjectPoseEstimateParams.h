#ifndef _PCL_OBJECTPOSEESTIMATEPARAMS_H_
#define _PCL_OBJECTPOSEESTIMATEPARAMS_H_

#include <string>

#include <cnoid/EigenTypes>

struct ObjectPoseEstimateParams {
	bool           is_load_file;
	bool           is_save_file;
	bool           is_load_obj;
	bool           is_color;
	bool           is_show_scene;
	bool           is_show_segment;
	bool           is_show_moved;
	double         sampling_density;
	std::string    load_file_path;
	std::string    save_file_path;
	std::string    obj_file_path;
	double         normal_radius;
	double         feature_radius;
	int            normal_k;
	int            feature_k;
	bool           use_ksearch;
	int            iteration;
	double         plane_remove_dist;
	int            num_plane_remove;
	double         segment_tolerance;
	double         segment_vtolerance;
	double         segment_resegment_radius;
	double         segment_boundary_radius;
	int            num_candidate;
	double         xmax, xmin, ymax, ymin, zmax, zmin;
	bool           is_do_narrow;
	cnoid::Vector3 view_point;
	int            num_neighbor;
	bool           use_bbsimilarity;
	bool           is_handcamera;  ///< @deprecated
	int            handcamera_arm_id;  ///< @deprecated
	bool           is_mergecloud;
	bool           do_recog_cvfh;
	bool           use_prev_result;
	int            prev_grasped_cluster_id;
	double         segment_sv_seed_resolution;
	double         segment_sv_spatial_coeff;
	double         segment_sv_normal_coeff;
	double         segment_sv_color_coeff;
	double         segment_lccp_minsize;
	double         segment_lccp_concavitytolerance;
	bool           use_lccp_segmentation;
};

#endif /* _PCL_OBJECTPOSEESTIMATEPARAMS_H_ */
