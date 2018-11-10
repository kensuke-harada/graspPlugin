#ifndef _GRASP_OBJECTPOSEESTIMATIONSOLUTION_H_
#define _GRASP_OBJECTPOSEESTIMATIONSOLUTION_H_

#include <vector>

#include "PointCloudEnv.h"

#include "exportdef.h"

namespace grasp {
	class TargetObject;
	
	class EXCADE_API ObjPoseEstimateSol {
	public:
		ObjPoseEstimateSol();
		virtual ~ObjPoseEstimateSol();

		cnoid::Vector3 p;
		cnoid::Matrix3 R;
		TargetObject* targetObject;
		std::vector<int> outlier_indices;
		PointCloudEnv* env_point_cloud;
		bool is_target;
		bool is_feasible;
		double score;
		// for output experimental result
		static std::string datafile_id;
	};

	class EXCADE_API ObjPoseEstimateSolHolder {
	public:
		static ObjPoseEstimateSolHolder* instance();
		virtual ~ObjPoseEstimateSolHolder();

		bool empty() const;
		int size() const;
		void resize(int i);
		ObjPoseEstimateSol& at(int i);
		const ObjPoseEstimateSol& at(int i) const;
		ObjPoseEstimateSol& back();
		const ObjPoseEstimateSol& back() const;
		void push_back(ObjPoseEstimateSol& sol);
		void clear();

	protected:
		ObjPoseEstimateSolHolder();

		std::vector<ObjPoseEstimateSol> pose_estimation_sols_;
	};
}

#endif /* _GRASP_OBJECTPOSEESTIMATIONSOLUTION_H_ */
