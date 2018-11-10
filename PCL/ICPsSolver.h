/**
 * @file   ICPsSolver.h
 * @author Akira Ohchi
*/

#ifndef _PCL_ICPSOLVER_H_
#define _PCL_ICPSOLVER_H_

#include <vector>

#include <Eigen/StdVector>

#include "PointCloudTypes.h"

class ICPsSolver {
 public:
	struct InputData {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Matrix4f init_matrix;  ///< initial transformation matrix
		Eigen::Matrix4f pose_matrix;  ///< from object coordinate to camera coordinate
		PointCloudTConstPtr target_cloud;  ///< input cloud (camera coordinate)
	};

	typedef std::vector<ICPsSolver::InputData, Eigen::aligned_allocator<ICPsSolver::InputData> > InputDataVec;
	typedef InputDataVec::const_iterator InputDataVecConstIte;

	ICPsSolver();
	virtual ~ICPsSolver();

	void setSceneCloud(PointCloudTConstPtr scene_cloud);
	void setObjModelCloud(PointCloudTConstPtr obj_cloud);

	virtual bool solveICPs(const InputDataVec& inputs) = 0;
	virtual void clear() = 0;

	void setMaxIteration(int max_iteration);
	void setTransEps(double trans_eps);
	void setFitEps(double fit_eps);
	void getBestMatrix(Eigen::Matrix4f& trans_matrix) const;
	double getBestScore() const;

 protected:
	PointCloudTConstPtr scene_cloud_;
	PointCloudTConstPtr obj_cloud_;
	int max_iteration_;  ///< icp convergence criteria (number of iterations)
	double trans_eps_;  ///< icp convergence criteria (transformation matrix difference)
	double fit_eps_;  ///< icp convergence criteria (relative MSE)

	Eigen::Matrix4f trans_matrix_;
	double best_score_;

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ICPsSolverSerial :
public ICPsSolver {
 public:
	ICPsSolverSerial();
	~ICPsSolverSerial();

	bool solveICPs(const InputDataVec& inputs);
	bool solveICPs(InputDataVecConstIte inputs_begin, InputDataVecConstIte inputs_end);
	void clear();

	bool hasSol() const;
	double getScore() const;

 private:
	double score_;
	bool has_sol_;

	int getNumOutlierPoints(const PointCloudTConstPtr& obj, const PointCloudTConstPtr& scene, double max_range) const;
	int getNumOutlierPoints(const PointCloudTConstPtr& obj, KdTreePointT* tree, double max_range) const;
	double calcScore(const PointCloudTConstPtr& obj, KdTreePointT* tree, double max_range) const;
};

class ICPsSolverParallel :
public ICPsSolver {
 public:
	ICPsSolverParallel();
	~ICPsSolverParallel();

	bool solveICPs(const InputDataVec& inputs);
	void clear();
};

#endif /* _PCL_ICPSOLVER_H_ */
