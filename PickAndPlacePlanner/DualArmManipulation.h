// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "ManipController.h"

#ifndef DUALARMMANIPULATION_H
#define DUALARMMANIPULATION_H

namespace grasp{
	namespace PickAndPlacePlanner{

		class DualArmManipulation : public grasp::PickAndPlacePlanner::ManipController
		{
		public:
			static DualArmManipulation* instance();
			DualArmManipulation();
			~DualArmManipulation();

			bool graspWithBothHand(bool same_direction=false,int keyposes=0);
			bool calcJointSeqBothHand(
				cnoid::Vector3& Po_ini,cnoid::Matrix3& Ro_ini,
				cnoid::Vector3& Pr_grasp,cnoid::Matrix3& Rr_grasp,cnoid::Vector3& Pl_grasp,cnoid::Matrix3& Rl_grasp,
				cnoid::Vector3& Pr_put,cnoid::Matrix3& Rr_put,cnoid::Vector3& Pl_put,cnoid::Matrix3& Rl_put,
				int keyposes);
			void poseInterpolation(cnoid::Vector3& Po_start,cnoid::Matrix3& Ro_start,cnoid::Vector3& Pr_start,cnoid::Matrix3& Rr_start,cnoid::Vector3& Pl_start,cnoid::Matrix3& Rl_start,
				cnoid::Vector3& Pr_end,cnoid::Matrix3& Rr_end,cnoid::Vector3& Pl_end,cnoid::Matrix3& Rl_end,
				int keyposes,
				std::vector<cnoid::Vector3>& Pr,std::vector<cnoid::Matrix3>& Rr,std::vector<cnoid::Vector3>& Pl,std::vector<cnoid::Matrix3>& Rl);

			cnoid::Matrix3 r_GRCR,l_GRCR;
		};
	}
}

#endif

