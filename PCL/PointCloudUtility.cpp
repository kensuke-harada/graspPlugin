#include "PointCloudUtility.h"

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

#include "../Grasp/VectorMath.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

using std::vector;
using cnoid::Vector3;
using cnoid::Matrix3;

namespace {
	void _initrand() {
#ifdef WIN32
		srand((unsigned)time(NULL));
#else
		srand48(time(0));
#endif
	}

	double _drand() {
#ifdef WIN32
		return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
#else
		return drand48();
#endif
	}
}

NormalEstimator::NormalEstimator() :
	radius_(0.01),
	k_(10),
	use_ksearch_(false),
	tree_(new KdTreePointT(false)) {
}

NormalEstimator::~NormalEstimator() {
}

void NormalEstimator::setRadius(double radius) {
	radius_ = radius;
}

void NormalEstimator::setK(unsigned int k) {
	k_ = k;
}

void NormalEstimator::useKSearch(bool use) {
	use_ksearch_ = use;
}

void NormalEstimator::useRadiusSearch(bool use) {
	use_ksearch_ = !use;
}

void NormalEstimator::setSearchMethod(KdTreePointTPtr tree) {
	tree_ = tree;
}

double NormalEstimator::getRadius() const {
	return radius_;
}

double NormalEstimator::getK() const {
	return k_;
}

void NormalEstimator::computeNormals(PointCloudTConstPtr input, NormalCloudPtr normals, const Vector3& viewpoint) {
	pcl::NormalEstimation<PointT, pcl::Normal> normal_est;
	normal_est.setViewPoint(viewpoint(0), viewpoint(1), viewpoint(2));
	normal_est.setInputCloud(input);
	normal_est.setSearchMethod(tree_);
	if (use_ksearch_) {
		normal_est.setKSearch(k_);
	} else {
		normal_est.setRadiusSearch(radius_);
	}
	normal_est.compute(*normals);
}

void NormalEstimator::computeNormals(PointCloudTConstPtr input, NormalCloudPtr normals, pcl::PointIndicesPtr indices,
																		 const Vector3& viewpoint) {
	pcl::NormalEstimation<PointT, pcl::Normal> normal_est;
	normal_est.setViewPoint(viewpoint(0), viewpoint(1), viewpoint(2));
	normal_est.setIndices(indices);
	normal_est.setInputCloud(input);
	normal_est.setSearchMethod(tree_);
	if (use_ksearch_) {
		normal_est.setKSearch(k_);
	} else {
		normal_est.setRadiusSearch(radius_);
	}
	normal_est.compute(*normals);
}

namespace {
	void modelToPointCloudProc(cnoid::ColdetModelPtr target_model,
														 PointCloudTPtr cloud,
														 double sampling_density,
														 bool all_triangles,
														 const Vector3& viewpoint) {
		cloud->clear();
		int t[3];
		_initrand();
		for (int k = 0; k < target_model->getNumTriangles(); k++) {
			target_model->getTriangle(k, t[0], t[1], t[2]);
			float tx, ty, tz;
			vector<Vector3> ver(3);
			for (int i = 0; i < 3; i++) {
				target_model->getVertex(t[i], tx, ty, tz);
				ver[i] = Vector3(tx, ty, tz);
			}
			cnoid::Vector3 ortho = grasp::cross(ver[1]-ver[0], ver[2]-ver[0]);
			if (!all_triangles) {
				Vector3 cent = (ver[0] + ver[1] + ver[2]) / 3.0;
				Vector3 dir = viewpoint - cent;
				if (grasp::dot(dir, ortho) < 0) continue;
			}
			double area = 0.5 * grasp::norm2(ortho);
			double num = area / sampling_density;
			int i_num = static_cast<int>(std::floor(num));
			if (_drand() < (num-static_cast<double>(i_num))) {
				i_num++;
			}
			for (int i = 0; i < i_num; i++) {
				double alpha = _drand();
				double beta = (_drand() * (1.0 - alpha));
				double gamma = 1.0 - alpha - beta;
				PointT p;
				Vector3 v_tmp = alpha * ver[0] + beta * ver[1] + gamma * ver[2];
				p.x = v_tmp(0);
				p.y = v_tmp(1);
				p.z = v_tmp(2);
				cloud->push_back(p);
			}
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = true;
	}
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
void modelToPointCloudProc(
	cnoid::SgNode* target_model, PointCloudTPtr cloud, double sampling_density, 
	bool all_triangles, const Vector3& viewpoint) {
		cloud->clear();
		int t[3];
		_initrand();
		cnoid::SgMeshPtr mesh = grasp::ColdetConverter::ExtractMesh(target_model);
		cnoid::SgVertexArrayPtr vertices = mesh->vertices();
		cnoid::SgIndexArray indices = mesh->triangleVertices();
		for (int k = 0; k < mesh->numTriangles(); k++) {
			vector<Vector3> ver(3);
			for (int i = 0; i < 3; i++) {
				cnoid::Vector3f vec = vertices->at(indices[k * 3 + i]);
				ver[i] = Vector3(vec[0], vec[1], vec[2]);
			}
			cnoid::Vector3 ortho = grasp::cross(ver[1]-ver[0], ver[2]-ver[0]);
			if (!all_triangles) {
				Vector3 cent = (ver[0] + ver[1] + ver[2]) / 3.0;
				Vector3 dir = viewpoint - cent;
				if (grasp::dot(dir, ortho) < 0) continue;
			}
			double area = 0.5 * grasp::norm2(ortho);
			double num = area / sampling_density;
			int i_num = static_cast<int>(std::floor(num));
			if (_drand() < (num-static_cast<double>(i_num))) {
				i_num++;
			}
			for (int i = 0; i < i_num; i++) {
				double alpha = _drand();
				double beta = (_drand() * (1.0 - alpha));
				double gamma = 1.0 - alpha - beta;
				PointT p;
				Vector3 v_tmp = alpha * ver[0] + beta * ver[1] + gamma * ver[2];
				p.x = v_tmp(0);
				p.y = v_tmp(1);
				p.z = v_tmp(2);
				cloud->push_back(p);
			}
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = true;
	}
#endif
}

void PointCloudUtil::modelToPointCloud(cnoid::ColdetModelPtr target_model, PointCloudTPtr cloud, double sampling_density) {
	modelToPointCloudProc(target_model, cloud, sampling_density, true, cnoid::Vector3::Zero());
}
void PointCloudUtil::modelToPointCloudVisibleRegion(cnoid::ColdetModelPtr target_model,
	PointCloudTPtr cloud, const cnoid::Vector3& viewpoint, double sampling_density) {
	modelToPointCloudProc(target_model, cloud, sampling_density, false, viewpoint);
}
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
void PointCloudUtil::modelToPointCloud(cnoid::SgNode* target_model, PointCloudTPtr cloud, double sampling_density) {
	modelToPointCloudProc(target_model, cloud, sampling_density, true, cnoid::Vector3::Zero());
}
void PointCloudUtil::modelToPointCloudVisibleRegion(cnoid::SgNode* target_model,
	PointCloudTPtr cloud, const cnoid::Vector3& viewpoint, double sampling_density) {
	modelToPointCloudProc(target_model, cloud, sampling_density, false, viewpoint);
}
#endif

void PointCloudUtil::makeBoxCloud(PointCloudTPtr cloud, const cnoid::Vector3& edge, double interval) {
	cloud->points.clear();

	for (int i = 0; i < 3; i++) {
		double hw = edge(i) / 2.0;
		double hl = edge((i+1) % 3) / 2.0;
		double hh = edge((i+2) % 3) / 2.0;
		for (double x = 0; x < hw; x += interval) {
			for (double y = 0; y < hl; y += interval) {
				for (unsigned int n = 0; n < 8; n++) {
					int sign_x = (n & 0x0001) ? 1 : -1;
					int sign_y = (n & 0x0002) ? 1 : -1;
					int sign_z = (n & 0x0004) ? 1 : -1;
					cnoid::Vector3 tmp(x, y, hh);
					pcl::PointXYZ p;
					p.x = sign_x * tmp((3-i) % 3);
					p.y = sign_y * tmp((4-i) % 3);
					p.z = sign_z * tmp((5-i) % 3);
					cloud->points.push_back(p);
				}
			}
		}
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
}

void PointCloudUtil::makeCylinderCloud(PointCloudTPtr cloud, double radius, double length, double interval) {
	cloud->points.clear();

	double angle_step = interval / radius;
	for (double theta = 0; theta < 2 * M_PI; theta += angle_step) {
		for (double l = 0; l < length/2.0; l += interval) {
			pcl::PointXYZ p;
			p.x = radius * cos(theta);
			p.y = radius * sin(theta);
			p.z = l;
			cloud->points.push_back(p);
			p.x = radius * cos(theta);
			p.y = radius * sin(theta);
			p.z = -l;
			cloud->points.push_back(p);
		}
	}

	for (double x = 0; x < radius; x += interval) {
		for (double y = 0; y < radius; y += interval) {
			if (x*x + y*y > radius * radius) continue;
			for (unsigned int n = 0; n < 8; n++) {
				int sign_x = (n & 0x0001) ? 1 : -1;
				int sign_y = (n & 0x0002) ? 1 : -1;
				int sign_z = (n & 0x0004) ? 1 : -1;
				pcl::PointXYZ p;
				p.x = sign_x * x;
				p.y = sign_y * y;
				p.z = sign_z * length / 2.0;
				cloud->points.push_back(p);
			}
		}
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
}

cnoid::Matrix4f PointCloudUtil::convM3VtoM4(const Matrix3& rot, const Vector3& trans) {
	cnoid::Matrix4 ret;
	ret.block(0, 0, 3, 3) << rot;
	ret.col(3).head(3) << trans;
	ret.row(3) << 0, 0, 0, 1;
	return ret.cast<float>();
}

void PointCloudUtil::convM4toM3V(const cnoid::Matrix4f& mat, Matrix3& rot, Vector3& trans) {
	rot = mat.block<3, 3>(0, 0).cast<double>();
	trans = mat.block<3, 1>(0, 3).cast<double>();
}

void PointCloudUtil::getMinMax(const PointCloudTPtr& cloud, cnoid::Vector3& min_p, cnoid::Vector3& max_p) {
	Eigen::Vector4f _min_p, _max_p;
	pcl::getMinMax3D(*cloud, _min_p, _max_p);
	min_p = _min_p.head<3>().cast<double>();
	max_p = _max_p.head<3>().cast<double>();
}

void PointCloudUtil::cloudToVec(PointCloudTConstPtr cloud, vector<Vector3>& vec) {
	vec.clear();
	vec.resize(cloud->points.size());
	PointCloudTConstIterator c_ite;
	vector<Vector3>::iterator v_ite;
	for (c_ite = cloud->begin(), v_ite = vec.begin(); c_ite != cloud->end(); ++c_ite, ++v_ite) {
		(*v_ite) = Vector3(c_ite->x, c_ite->y, c_ite->z);
	}
}

void PointCloudUtil::vecToCloud(const vector<Vector3>& vec, PointCloudTPtr cloud) {
	cloud->clear();
	cloud->resize(vec.size());
	PointCloudT::iterator c_ite;
	vector<Vector3>::const_iterator v_ite;
	for (c_ite = cloud->begin(), v_ite = vec.begin(); v_ite != vec.end(); ++c_ite, ++v_ite) {
		(*c_ite) = PointT(v_ite->x(), v_ite->y(), v_ite->z());
	}
}

void PointCloudUtil::normalCloudToVec(NormalCloudConstPtr normal, vector<Vector3>& vec) {
	vec.clear();
	vec.resize(normal->points.size());
	NormalCloudConstIterator c_ite;
	vector<Vector3>::iterator v_ite;
	for (c_ite = normal->begin(), v_ite = vec.begin(); c_ite != normal->end(); ++c_ite, ++v_ite) {
		(*v_ite) = Vector3(c_ite->normal_x, c_ite->normal_y, c_ite->normal_z);
	}
}

void PointCloudUtil::removeNan(PointCloudTPtr cloud) {
	std::vector<int> index;
	bool tmp_dense = cloud->is_dense;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
	cloud->is_dense = tmp_dense;
}

void PointCloudUtil::removeNan(ColorPointCloudPtr cloud) {
	std::vector<int> index;
	bool tmp_dense = cloud->is_dense;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
	cloud->is_dense = tmp_dense;
}

void PointCloudUtil::voxelFilter(PointCloudTConstPtr input, PointCloudTPtr output, double leaf_size) {
	pcl::VoxelGrid<PointT> vg;
	vg.setLeafSize(leaf_size, leaf_size, leaf_size);
	vg.setInputCloud(input);
	vg.filter(*output);
}

void PointCloudUtil::transCloud(PointCloudTPtr cloud, const cnoid::Matrix3& R, const cnoid::Vector3& p) {
	cnoid::Matrix4f trans_mat = convM3VtoM4(R, p);
	pcl::transformPointCloud(*cloud, *cloud, trans_mat);
}

void PointCloudUtil::transCloud(ColorPointCloudPtr cloud, const cnoid::Matrix3& R, const cnoid::Vector3& p) {
	cnoid::Matrix4f trans_mat = convM3VtoM4(R, p);
	pcl::transformPointCloud(*cloud, *cloud, trans_mat);
}

void PointCloudUtil::transCloud(PointCloudTConstPtr input, PointCloudTPtr output, const cnoid::Matrix3& R, const cnoid::Vector3& p) {
	cnoid::Matrix4f trans_mat = convM3VtoM4(R, p);
	pcl::transformPointCloud(*input, *output, trans_mat);
}

void PointCloudUtil::cropCloud(const PointCloudTConstPtr& input, PointCloudTPtr& output,
															 const cnoid::Vector3& min_p, const cnoid::Vector3& max_p) {
	pcl::CropBox<PointT> crop;
	Eigen::Vector4f min_pt(min_p.x(), min_p.y(), min_p.z(), 1);
	Eigen::Vector4f max_pt(max_p.x(), max_p.y(), max_p.z(), 1);
	crop.setInputCloud(input);
	crop.setMin(min_pt);
	crop.setMax(max_pt);
	crop.filter(*output);
}

void PointCloudUtil::calcBoundingBox(PointCloudTConstPtr cloud, Vector3& edge, Vector3& center, Matrix3& Rot) {
	Eigen::Matrix3f covariance_mat;
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);

	pcl::computeCovarianceMatrix(*cloud, centroid, covariance_mat);

	Eigen::Vector3f eval;
	Eigen::Matrix3f evec;

	pcl::eigen33(covariance_mat, evec, eval);

	vector<double> c;
	for (size_t i = 0; i < 3; i++) {
		c.push_back(fabs(eval(i)));
	}

	Rot = evec.cast<double>();
	int j = grasp::argmax(c);
	Vector3 v0 = evec.col(j).cast<double>();
	Vector3 v1 = evec.col((j+1)%3).cast<double>();

	double r = v0.dot(v1);
	v1 = (-r * v0 + v1)/sqrt(1 - r * r);
	Vector3 v2 = v0.cross(v1);

	for(size_t i = 0; i < 3; i++) {
		Rot(i, (j+1)%3) = v1(i);
		Rot(i, (j+2)%3) = v2(i);
	}

	Vector3 e[3];
	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			e[j][i] = Rot(i, j);
		}
	}

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for (size_t i = 0; i < cloud->points.size(); i++) {
		Vector3 pt = (cloud->points[i].getVector4fMap() - centroid).block<3, 1>(0, 0).cast<double>();
		for (size_t j = 0; j < 3; j++) {
			double tmp = e[j].dot(pt);
			if (tmp > pt_max[j]) pt_max[j] = tmp;
			if (tmp < pt_min[j]) pt_min[j] = tmp;
		}
	}

	edge =  (pt_max - pt_min);
	center =  centroid.block<3,1>(0,0).cast<double>() + 0.5 * Rot * (pt_max + pt_min);
}
