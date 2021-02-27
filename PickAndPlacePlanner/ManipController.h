// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef MANIPCONTROLLER_H
#define MANIPCONTROLLER_H

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

#ifndef WIN32
#include <dirent.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#ifndef WIN32
#include <sys/resource.h>
#endif

#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <boost/filesystem.hpp>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ModelNodeSet>
#include <cnoid/ColdetLinkPair>
#else
#include "../Grasp/ColdetLinkPair.h"
#endif

#include "../Grasp/VectorMath.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/readtext.h"
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"
#include "../GeometryHandler/ClusterParameter.h"
#include "RobotLocalFunctions.h"
#include "../ObjectPlacePlanner/PlacePlanner.h"
#include "GraspDatabaseManipulator.h"
#include "ObjectPosReader.h"
#include "../Grasp/OccupiedVoxel.h"

#include "exportdef.h"

class SweptVolume;

namespace grasp{
namespace PickAndPlacePlanner{

//#define MOVE_ARM_ONEBYONE
//#define PLAN_ONE_ARM
//#define EXPERIMENT_DATA_OUTPUT
//#define TRAINING_MODE
//#define LEARNING_DATA_OUTPUT
#ifdef LEARNING_DATA_OUTPUT
#define EXPERIMENT_DATA_OUTPUT
#endif

class EXCADE_API ManipController : public grasp::GraspController
{

public :
		static ManipController* instance();
		ManipController();
		virtual ~ManipController();

		virtual bool initial(TargetObject* targetObject, ArmFingers* targetArmFinger);
		virtual bool doGraspPlanning();
		bool doCylinderGraspPlanning();
#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
		bool doBinPickingGraspPlanning(int obj_id, OccupiedVoxelVec* voxel = NULL, bool include_negative = false);
#else
		bool doBinPickingGraspPlanning(OccupiedVoxelVec* voxel = NULL, bool include_negative = false);
#endif
		bool doBinPickingGraspPlanning(cnoid::Vector3& put_pos);
		void binPickingInitProc();
		bool binPickingMainProc(const grasp::ObjPoseEstimateSol& sol, int& sec, double& score, cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd& theta);
		void binPickingFinalProc(bool is_succeed, const cnoid::Vector3& Pp_grasp, const cnoid::Matrix3& Rp_grasp, const cnoid::Vector3& Pp_put, const cnoid::Matrix3& Rp_put, const cnoid::VectorXd& theta);
		bool searchPickAndPlaceMotion(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd &theta_, bool edge_grasp);
		bool searchPickAndPlaceMotionFromDB(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd &theta_, bool edge_grasp,std::string& db_path);
		bool searchPickAndPlaceMotionWithRegraspFromDB(cnoid::Vector3& Pp_grasp2, cnoid::Matrix3& Rp_grasp2, cnoid::Vector3& Pp_grasp_next2, cnoid::Matrix3& Rp_grasp_next2, cnoid::Vector3& Pp_regrasp2, cnoid::Matrix3& Rp_regrasp2, cnoid::Vector3& Pp_put2, cnoid::Matrix3& Rp_put2, cnoid::VectorXd& th_grasp2,cnoid::VectorXd& th_put2, bool edge_grasp,std::string& db_path_g, std::string& db_path_p, double& waist_q);
		bool searchPickAndPlaceWithRegrasp(cnoid::Vector3& Pp_grasp2, cnoid::Matrix3& Rp_grasp2, cnoid::Vector3& Pp_tmp_put, cnoid::Matrix3& Rp_tmp_put, cnoid::Vector3& Pp_regrasp, cnoid::Matrix3& Rp_regrasp, cnoid::Vector3& Pp_put2, cnoid::Matrix3& Rp_put2, cnoid::VectorXd& th_grasp2, cnoid::VectorXd& th_put2, bool edge_grasp);
		bool searchCylinderBinPicking(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd &theta_, int objId, bool edge_grasp);
#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
		bool searchBinPicking(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd& theta_, int obj_id, bool calc_negative = true, OccupiedVoxelVec* voxel = NULL);
#else
		bool searchBinPicking(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd& theta_, bool calc_negative = true, OccupiedVoxelVec* voxel = NULL);
#endif
		bool searchBinPickingNoML(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd& theta_);
		void clearTrajectories();

		bool searchSurfaceSurface(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::VectorXd &theta_, bool edge_grasp){
				cnoid::Vector3 Pp_put = cnoid::Vector3::Zero();
				cnoid::Matrix3 Rp_put = cnoid::Matrix3::Identity();
				return searchPickAndPlaceMotion(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta_, edge_grasp);
		};

		virtual void calcJointSeq(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put);
		bool calcJointSeq(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::Vector3& appVec);
		virtual bool calcJointSeqTest(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::Vector3& appVec);
		bool calcJointSeq2(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& appVec);
		bool calcJointSeqTest2(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& appVec);
		bool calcJointSeqRegrasp(cnoid::Vector3& P0, cnoid::Matrix3& R0, cnoid::Vector3& P1, cnoid::Matrix3& R1, cnoid::Vector3& P2, cnoid::Matrix3& R2, cnoid::Vector3& P3, cnoid::Matrix3& R3, cnoid::Vector3& appVec, cnoid::Vector3& appVec2, bool is_fixwaist = false, double waist_q = 0.0);
//		virtual bool calcJointSeqTestRegrasp(cnoid::Vector3& P0, cnoid::Matrix3& R0, cnoid::Vector3& P1, cnoid::Matrix3& R1, cnoid::Vector3& P2, cnoid::Matrix3& R2, cnoid::Vector3& P3, cnoid::Matrix3& R3, cnoid::Vector3& appVec, cnoid::Vector3& appVec2, bool is_fixwaist = false, double waist_q = 0.0);
		void chooseArmToGrasp();
		void chooseArmToPut();
		void selectManipStrategy();
//		void setArm(ArmFingers* grasp_arm,ArmFingers* put_arm);

		bool getPosture(std::string data,cnoid::Vector3& p,cnoid::Matrix3& R,cnoid::VectorXd& finger_angle);

		void displaySweptVolume(int sol_id);
		void clearSweptVolume();

		bool initPalm;
		bool putObject;
		int intention, strategy;
		std::vector<cnoid::Vector3> Po_put, Po_tmp; //Object putting position
		std::vector<cnoid::Matrix3> Ro_put, Ro_tmp;
		enum ManipStrategies { RIGHT_RIGHT, LEFT_LEFT, RIGHT_LEFT, LEFT_RIGHT, RIGHT_PUT_RIGHT, LEFT_PUT_LEFT, RIGHT_PUT_LEFT, LEFT_PUT_RIGHT, NOT_SELECTED};
		enum Hand { RIGHT, LEFT };
		enum Robots {PA10, HIRO, HRP2, OTHER};
		//		enum ManipMode {DefaultMode, RightGraspMode, LeftGraspMode, DefaultGraspMode};
		int graspingHand, robot; //, manipMode;

		std::ostream& os;

protected :
		void setPutArm(int j);
		void setGraspArm(int j);
		void calcArmJointSeq(cnoid::VectorXd& armJointSeq, ArmPtr& arm_);
		bool setMotionSeq(int graspingState, int contactState, const cnoid::Vector3& P, const cnoid::Matrix3& R, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], double time,bool fingerCloseUnderGrasp = false, bool is_fixwaist = false, double waist_q = 0.0, int graspingState2 = PlanBase::NOT_GRASPING);
		bool setMotionSeqDual(int graspingState, int graspingState2, int contactState, const cnoid::Vector3& P, const cnoid::Matrix3& R, const cnoid::Vector3& P2, const cnoid::Matrix3& R2, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], double time,bool fingerCloseUnderGrasp = false,bool fingerCloseUnderGrasp2 = false, bool is_fixwaist = false, double waist_q = 0.0);
		void setJointAngle(int graspingState, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[],bool fingerCloseUnderGrasp = false);
		void setGraspingStateSeq(int graspingState, int graspingState2, int contactState);
		void setTargetObjPos(cnoid::Vector3& P_ini, cnoid::Matrix3& R_ini, std::vector<cnoid::Vector3>& P_des, std::vector<cnoid::Matrix3>& R_des, int j);
		void calcFingerOpenPose(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Vector3& Nco1, const cnoid::Vector3& Nco2, const cnoid::Matrix3& Rp_grasp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, double margin);
		bool withinJointLimit();
		bool isColliding(int graspingState, int graspingState2, int contactState);
		bool isColliding2();
		bool searchLiftUpKeyPose(const cnoid::Vector3& P, const cnoid::Matrix3& R, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers[], const cnoid::Vector3& app, double time,double* heihgt, double min_height, int contactState);
		void clearSequence();
		void replaceLastPathDOFSeqOneArm(int graspingState, ArmPtr& arm_, FingerPtr fingers_[]);

		cnoid::BodyItemPtr envItem;
		ArmPtr arm_g, arm_p;
		FingerPtr fingers_g[3], fingers_p[3];
		int arm_g_n_fing, arm_p_n_fing;
		std::vector<double> offset_g, offset_p;
		bool firstPick;
		cnoid::Vector3 Pinit;
		//std::vector<double> motionTimeSeq;
		cnoid::Vector3 approachVec, approachVec2;
		cnoid::VectorXd  meanPointCloud, varPointCloud;
		double cylLength, cylRadius;
		RobotLocalFunctions *rb;
		CollisionPair *cp;
		ParameterFileData *pf;

		double lift_height,release_height,min_lift_height,min_relase_height;

		GraspDatabaseManipulator::GraspPoses grasp_poses;
		std::ofstream fout;

		//// for binpicking
		SweptVolume* sv_;
};

}
}

#endif
