#include "CVFHDescriptorManipulator.h"

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)

#include <limits>

#include <fstream>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

using std::string;
namespace fs = boost::filesystem;

CVFHDescriptorManipulator::CVFHDescriptorManipulator() :
	normal_estimator_(NULL) {
}

CVFHDescriptorManipulator::~CVFHDescriptorManipulator() {
}

/**
 * @brief compute target_cloud's CVFH descriptor
 * @param[in] target_cloud target cloud
 * @param[in] view view point
 * @param[in] max_length length of the diagonal of the target_cloud's bounding box
 * @param[out] des CVFH descriptor
 */
void CVFHDescriptorManipulator::computeDescriptor(PointCloudTConstPtr target_cloud, const Eigen::Vector3d& view, double max_length, CVFHDescriptors& des) {
	if (normal_estimator_ == NULL) {
		std::cerr << "ERROR(CVHFDescriptorManipulator): Normal Estimator is not set" << std::endl;
		return;
	}

	// compute normals
	NormalCloudPtr normal = NormalCloudPtr(new NormalCloud());
	normal_estimator_->computeNormals(target_cloud, normal, view);

	// compute CVFH descriptor
	std::vector<DescriptorPair> despair;
	CVFHEstimator<pcl::PointXYZ> cvfh;
	cvfh.setMaxLength(max_length);
	cvfh.estimate(target_cloud, normal, view, despair);

	// convert DescriptorPair vector to CVFHDesPose vector
	des.resize(despair.size());
	for (size_t j = 0; j < despair.size(); j++) {
		des[j].des_pair = despair[j];
		des[j].pose = Eigen::Matrix4f::Identity();
	}
}

/**
 * @brief generate CVFH descriptor files
 * @param[in] pcd_dir a directory path where partial model pcd files are located and generated files are saved.
 * @param[in] voxel_leaf leaf size
 */
bool CVFHDescriptorManipulator::generateDescriptors(const std::string& pcd_dir, double voxel_leaf) {
	if (normal_estimator_ == NULL) {
		std::cerr << "ERROR(CVHFDescriptorManipulator): Normal Estimator is not set" << std::endl;
		return false;
	}

	fs::path pcd_dir_path(pcd_dir);
	int i = 0;
	for (; ; i++) {
		string num_str = boost::lexical_cast<string>(i);

		// load partial model pcd
		string model_name = "model_" + num_str + ".pcd";
		fs::path model_file_path = pcd_dir_path / fs::path(model_name);
		if (!fs::exists(model_file_path)) break;
		PointCloudTPtr cloud = PointCloudTPtr(new PointCloudT());
		pcl::io::loadPCDFile(model_file_path.string(), *cloud);

		// sampling
		PointCloudTPtr sampled_cloud = PointCloudTPtr(new PointCloudT());
		PointCloudUtil::voxelFilter(cloud, sampled_cloud, voxel_leaf);

		// compute Bounding Box
		Eigen::Vector3f edge;
		std::string bb_name = "boundingbox_" + num_str + ".txt";
		fs::path  bb_file_path = pcd_dir_path / fs::path(bb_name);
		computeProjectedBB(sampled_cloud, edge);
		writeCentroid(bb_file_path.string(), edge);
		double max_length = edge.norm();

		// save sampled cloud
		std::string pcd_name = "pcd_" + num_str + ".pcd";
		fs::path pcd_file_path = pcd_dir_path / fs::path(pcd_name);
		pcl::io::savePCDFile(pcd_file_path.string(), *sampled_cloud, true);

		// compute descirptors
		CVFHDescriptors descriptor;
		computeDescriptor(sampled_cloud, Eigen::Vector3d::Zero(), max_length, descriptor);

		for (size_t j = 0; j < descriptor.size(); j++) {
			string des_num_str = boost::lexical_cast<string>(j);

			// save centroid
			std::string centroid_name = "centroid_" + num_str + "_" + des_num_str + ".txt";
			fs::path centroid_file_path = pcd_dir_path / fs::path(centroid_name);
			writeCentroid(centroid_file_path.string(), descriptor[j].des_pair.centroid);

			// save cvfh descriptor
			std::string cvfh_name = "cvfh_" + num_str + "_" + des_num_str + ".pcd";
			fs::path cvfh_file_path = pcd_dir_path / fs::path(cvfh_name);
			CVFHCloud cvfh_des;
			cvfh_des.push_back(descriptor[j].des_pair.cvfh_descriptor);
			pcl::io::savePCDFile(cvfh_file_path.string(), cvfh_des, true);

			// save crh descriptor
			std::string crh_name = "crh_" + num_str + "_" + des_num_str + ".pcd";
			fs::path crh_file_path = pcd_dir_path / fs::path(crh_name);
			CRHCloud crh_des;
			crh_des.push_back(descriptor[j].des_pair.crh_descriptor);
			pcl::io::savePCDFile(crh_file_path.string(), crh_des, true);

			// save crhext descriptor
			std::string crhext_name = "crhext_" + num_str + "_" + des_num_str + ".pcd";
			fs::path crhext_file_path = pcd_dir_path / fs::path(crhext_name);
			CRHCloud crhext_des;
			crhext_des.push_back(descriptor[j].des_pair.crhext_descriptor);
			pcl::io::savePCDFile(crhext_file_path.string(), crhext_des, true);
		}
	}
	if (i == 0) {
		std::cerr << "ERROR(CVHFDescriptorManipulator): There are no partial model files." << std::endl;
		return false;
	} else {
		std::string log_name = "generating_param.log";
		fs::path log_file_path = pcd_dir_path / fs::path(log_name);
		writeGeneratingParam(log_file_path.string(), voxel_leaf, normal_estimator_->getRadius());
	}
	return true;
}

/**
 * @brief load CVFH descriptor files
 * @param[in] pcd_dir a directory path where descriptor files are located.
 */
void CVFHDescriptorManipulator::readDescriptors(const std::string& pcd_dir) {
	descriptors_.clear();

	fs::path pcd_dir_path(pcd_dir);

	for (int i = 0; ; i++) {
		CVFHDesPose des_pose;

		string num_str = boost::lexical_cast<string>(i);

		std::string pcd_name = "pcd_" + num_str + ".pcd";
		fs::path pcd_file_path = pcd_dir_path / fs::path(pcd_name);
		if (!fs::exists(pcd_file_path)) break;

		// load pose
		std::string pose_name = "pose_" + num_str + ".txt";
		fs::path pose_file_path = pcd_dir_path / fs::path(pose_name);
		Eigen::Matrix4f pose;
		readMatrix(pose_file_path.string(), pose);

		// load bounding box
		std::string bb_name = "boundingbox_" + num_str + ".txt";
		fs::path bb_file_path = pcd_dir_path / fs::path(bb_name);
		Eigen::Vector3f bb;
		readCentroid(bb_file_path.string(), bb);

		for (int j = 0; ; j++) {
			CVFHDesPose des_pose;
			des_pose.pcd_filepath = pcd_file_path.string();
			des_pose.pose = pose;
			des_pose.bounding_box = bb;
			des_pose.id = i;

			string des_num_str = boost::lexical_cast<string>(j);

			// load centroid
			std::string centroid_name = "centroid_" + num_str + "_" + des_num_str + ".txt";
			fs::path centroid_file_path = pcd_dir_path / fs::path(centroid_name);
			if (!fs::exists(centroid_file_path)) break;
			readCentroid(centroid_file_path.string(), des_pose.des_pair.centroid);

			// load cvfh descriptor
			std::string cvfh_name = "cvfh_" + num_str + "_" + des_num_str + ".pcd";
			fs::path cvfh_file_path = pcd_dir_path / fs::path(cvfh_name);
			CVFHCloud cvfh_des;
			pcl::io::loadPCDFile(cvfh_file_path.string(), cvfh_des);
			des_pose.des_pair.cvfh_descriptor = cvfh_des[0];

			// load crh descriptor
			std::string crh_name = "crh_" + num_str + "_" + des_num_str + ".pcd";
			fs::path crh_file_path = pcd_dir_path / fs::path(crh_name);
			CRHCloud crh_des;
			pcl::io::loadPCDFile(crh_file_path.string(), crh_des);
			des_pose.des_pair.crh_descriptor = crh_des[0];

			// load crhext descriptor
			std::string crhext_name = "crhext_" + num_str + "_" + des_num_str + ".pcd";
			fs::path crhext_file_path = pcd_dir_path / fs::path(crhext_name);
			CRHCloud crhext_des;
			pcl::io::loadPCDFile(crhext_file_path.string(), crhext_des);
			des_pose.des_pair.crhext_descriptor = crhext_des[0];

			descriptors_.push_back(des_pose);
		}
	}
}

/**
 * @brief save 4x4 matrix to the file
 * @param[in] file_path
 * @param[in] mat 4x4 matrix
 */
bool CVFHDescriptorManipulator::writeMatrix(const std::string& file_path, const Eigen::Matrix4f& mat) {
	std::ofstream out(file_path.c_str());
	if (!out) {
		std::cerr << "Cannot open file." << file_path << std::endl;
		return false;
	}

	for (size_t i = 0; i < 4; i++) {
		for (size_t j = 0; j < 4; j++) {
			out << mat(i, j);
			if (!(i == 3 && j == 3))
				out << " ";
		}
	}
	out.close();

	return true;
}

/**
 * @brief load 4x4 matrix from the file
 * @param[in] file_path
 * @param[out] mat 4x4 matrix
 */
bool CVFHDescriptorManipulator::readMatrix(const std::string& file_path, Eigen::Matrix4f& mat) {
	std::ifstream in;
	in.open(file_path.c_str(), std::ifstream::in);

	char linebuf[1024];
	in.getline(linebuf, 1024);
	std::string line(linebuf);
	std::vector<std::string> strs_2;
	boost::split(strs_2, line, boost::is_any_of(" "));

	for (int i = 0; i < 16; i++) {
		mat(i / 4, i % 4) = boost::lexical_cast<float>(strs_2[i]);
	}

	return true;
}

/**
 * @brief save a vector of size 3 to the file
 * @param[in] file_path
 * @param[in] centroid
 */
bool CVFHDescriptorManipulator::writeCentroid(const std::string& file_path, const Eigen::Vector3f& centroid) {
	std::ofstream out(file_path.c_str());
	if (!out) {
		std::cerr << "Cannot open file." <<  file_path << std::endl;
		return false;
	}

	for (size_t i = 0; i < 3; i++) {
		out << centroid(i);
		if (i != 3)
			out << " ";
	}
	out.close();

	return true;
}

/**
 * @brief load a vector of size 3 from the file
 * @param[in] file_path
 * @param[out] centroid
 */
bool CVFHDescriptorManipulator::readCentroid(const std::string& file_path, Eigen::Vector3f& centroid) {
	std::ifstream in;
	in.open(file_path.c_str(), std::ifstream::in);

	char linebuf[1024];
	in.getline(linebuf, 1024);
	std::string line(linebuf);
	std::vector<std::string> strs_2;
	boost::split(strs_2, line, boost::is_any_of(" "));

	for (int i = 0; i < 3; i++) {
		centroid(i) = boost::lexical_cast<float>(strs_2[i]);
	}

	return true;
}

/**
 * @brief save parameters that are used for generating descriptors to the file
 * @param[in] file_path
 * @param[in] voxel_leaf voxel size
 * @param[in] normal_radius radius that is used for normal estiamtion
 */
bool CVFHDescriptorManipulator::writeGeneratingParam(const std::string& file_path, double voxel_leaf, double normal_radius) const {
	std::ofstream out(file_path.c_str());
	if (!out) {
		std::cerr << "Cannot open file." << file_path << std::endl;
		return false;
	}

	out << "sampling_density: " << voxel_leaf << std::endl;
	out << "normal_radius: " << normal_radius << std::endl;
	out << "generated on " << boost::gregorian::to_simple_string(boost::gregorian::day_clock::local_day()) << std::endl;
	return true;
}

/**
 * @brief compute a bounding box of projected point cloud
 *
 * edge[0] and edge[1] is the lengths of the boudning box of point cloud which is obtained by projecteing target_cloud onto x-y plane.
 * edge[2] is the length in the z-axis direction of target_cloud
 * @param[in] target_cloud target point cloud
 * @param[out] edge lengths of bounding box.
 */
void CVFHDescriptorManipulator::computeProjectedBB(PointCloudTConstPtr target_cloud, Eigen::Vector3f& edge) const {
	PointCloudTPtr projected_cloud(new PointCloudT);
  double max_z = -std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();

	// remove outliers
  PointCloudT filtered_cloud;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(target_cloud);
  outrem.setRadiusSearch(0.010);
  outrem.setMinNeighborsInRadius(13);
  outrem.filter(filtered_cloud);

  // project point cloud onto the x-y plane
  for (size_t i = 0; i < filtered_cloud.points.size(); i++) {
    pcl::PointXYZ p;
    p.x = filtered_cloud.points[i].x;
    p.y = filtered_cloud.points[i].y;
    p.z = 0;
		projected_cloud->push_back(p);
  }

	// compute a bounding box of projected point cloud
	cnoid::Vector3 center;
	cnoid::Matrix3 Rot;
	cnoid::Vector3 edge_d;
	PointCloudUtil::calcBoundingBox(projected_cloud, edge_d, center, Rot);

	// move minimum element to the edge[2]
	edge = edge_d.cast<float>();
  for (int i = 0; i < 2; i++) {
    if (fabs(edge[i]) < fabs(edge[i+1])) {
      float tmp = edge[i+1];
      edge[i+1] = edge[i];
      edge[i] = tmp;
    }
  }

	// compute depth (along z-axis) of target_cloud
  for (size_t i = 0; i < target_cloud->points.size(); i++) {
    if (max_z < target_cloud->points[i].z) {
      max_z = target_cloud->points[i].z;
    }
    if (min_z > target_cloud->points[i].z) {
      min_z = target_cloud->points[i].z;
    }
  }
  edge[2] = max_z - min_z;
}

PartialPointCloudCache::PartialPointCloudCache() :
	des_dir_("") {
}

PartialPointCloudCache::~PartialPointCloudCache() {
}

PartialPointCloudCache* PartialPointCloudCache::instance() {
	static PartialPointCloudCache* instance = new PartialPointCloudCache();
	return instance;
}

void PartialPointCloudCache::init(int size, const std::string& des_dir) {
	if (des_dir == des_dir_) return;
	pc_cache_.clear();
	pc_cache_.resize(size, PointCloudTPtr());
	des_dir_ = des_dir;
}

PointCloudTPtr PartialPointCloudCache::getPartialCloud(const CVFHDesPose& target_des_pose) {
	int idx = target_des_pose.id;
	if (!pc_cache_[idx]) {
		pc_cache_[idx] = PointCloudTPtr(new PointCloudT());
		pcl::io::loadPCDFile(target_des_pose.pcd_filepath, *(pc_cache_[idx]));
	}
	return pc_cache_[idx];
}

CVFHDescriptorCache::CVFHDescriptorCache() :
	des_dir_("") {
}

CVFHDescriptorCache::~CVFHDescriptorCache() {
}

CVFHDescriptorCache* CVFHDescriptorCache::instance() {
	static CVFHDescriptorCache* instance = new CVFHDescriptorCache();
	return instance;
}

void CVFHDescriptorCache::getCVFHDescriptors(const std::string& des_dir, CVFHDescriptors& descriptors) {
	if (des_dir != des_dir_) {
		des_dir_ = des_dir;
		CVFHDescriptorManipulator cvfh;
		cvfh.readDescriptors(des_dir_);
		des_cache_ = cvfh.getDescriptors();
	}
	descriptors = des_cache_;
	return;
}

#endif
