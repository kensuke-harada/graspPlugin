#ifndef _GRASPDATAGENRATOR_WAISTPOSITIONSEARCHER_H_
#define _GRASPDATAGENRATOR_WAISTPOSITIONSEARCHER_H_

#include <string>
#include <vector>
#include <fstream>

#include <cnoid/EigenTypes>
#include <cnoid/BodyItem>

#include "../ConstraintIK/ConstraintIK.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/RobotBody.h"
#include "../PickAndPlacePlanner/GraspDatabaseManipulator.h"

#include "GraspableRegion.h"
#include "BoxInfo.h"
#include "exportdef.h"

//#define THREAD
//#define DEBUG_MODE

namespace grasp {

	class VirtualObjectHandler {
	public:
		enum LOCATION {FRONT, BACK, LEFT, RIGHT};
		enum CORNER {BACKLEFT, FRONTLEFT, FRONTRIGHT, BACKRIGHT, CENTER};

		VirtualObjectHandler();
		virtual ~VirtualObjectHandler(){;}

		void setObject(const cnoid::BodyItem& orig_obj, CORNER corner, LOCATION location, const cnoid::Vector3& p);
		void enable(RobotBodyPtr body_, CORNER corner);
		void disable(RobotBodyPtr body_, CORNER corner);
	protected:
		cnoid::BodyItemPtr body[5][4];

		PlanBase* tc;
	};

	class WaistSearchParam {
	public:
		WaistSearchParam() : debug_mode(0), fast_ver(false), not_narrow_down(false), with_camera(false){;}

		bool display_graspable_points;
		double grid_interval;
		std::vector<int> target_arm_ids;

		int debug_mode;
		bool fast_ver;
		bool not_narrow_down;
		bool with_camera;
		bool gaze_object;
		cnoid::Vector3 gaze_point;
	};

	class PoseHolder {
	public:
		void store(const cnoid::BodyPtr body);
		void restore(const cnoid::BodyPtr body);
	private:
		std::vector<double> q;
	};

	class WaistPosSearchResult {
	public:
		static WaistPosSearchResult* instance(WaistPosSearchResult *wpsr=NULL);

		BoxInfoArray getBoxes() {return region.getAllBoxes();}
		void showResult(std::vector<bool> right, std::vector<bool> left, bool route);

		cnoid::Vector3 start;
		GraspableRegion region;
		std::vector<Eigen::Vector2i> route_coords;
	private:
		WaistPosSearchResult(){;}
	};

  class EXCADE_API WaistPositionSearcher : public GraspController {
  public:	
		typedef GraspDatabaseManipulator::GraspPosture GraspPosture;
		enum CORNER {BACKLEFT, FRONTLEFT, FRONTRIGHT, BACKRIGHT};

    WaistPositionSearcher();
		virtual ~WaistPositionSearcher(){;}

    static WaistPositionSearcher* instance(WaistPositionSearcher *wps = NULL);
    
		void setXYtransJoint(unsigned int x_joint_id, unsigned int y_joint_id);

		bool searchWaistPos(const BoxInfoArray& box_infos, const WaistSearchParam& param = WaistSearchParam());
		void graspCornerObj(const BoxInfoArray& box_infos, const WaistSearchParam& param, int corner);

		static int getTargetArmID();
	private:
		bool searchWaistPosSingleBox(const BoxInfoPtr box_info, cnoid::Vector3& pos);
		bool searchWaistPosMultiBox(const BoxInfoArray& box_infos);
		void loadObject(BoxInfoPtr box_info, cnoid::BodyItemPtr& target_object_item);
		void unloadObject(cnoid::BodyItemPtr& target_object_item);
		bool searchWaistPosSub(const BoxInfoPtr box_info, cnoid::Vector3& pos);
		bool searchWaistPosProc(cnoid::Vector3& pos);
		bool initialPosSearch();
		bool searchCornerGraspablePos();
		bool searchGraspablePosParallel(RobotBodyPtr body, double coeff, bool is_fix, int corner, int* has_sol, cnoid::Vector3* pos);
		bool isGraspable(std::vector<bool>& is_graspable);
		bool isGraspable();
		bool isGraspableSub(RobotBodyPtr body, int corner, double x, double y, int* has_sol);
		bool searchGraspableEdgePos(cnoid::Vector3& curr_pos, cnoid::Vector3& prev_pos, std::vector<bool>& is_graspable, bool* found_sol);
		bool searchBinary(cnoid::Vector3& curr_pos, cnoid::Vector3& prev_pos, const std::vector<bool>& is_graspable);
		bool searchGraspPosition(cnoid::Vector3& pos, double coeff, bool is_fix);
		bool searchGraspPosition(cnoid::Vector3& pos, double coeff, bool is_fix, unsigned int corner);
		bool searchGraspPosition(RobotBodyPtr body, double coeff, bool is_fix, unsigned int corner, bool is_center, cnoid::Vector3& pos);


		void setObjCornerPos();
		void readGraspDB();
		void obtainAveragePos(cnoid::Vector3& pos) const;
		void makeGraspPostureCandidate(unsigned int max_num = 1000);
		bool isColliding(const RobotBodyPtr body) const;
		bool withinJointLimit(const cnoid::BodyPtr body) const;
		bool isColliding() const;
    bool withinJointLimit() const;
		std::string getCornerString(unsigned int i) const;
		void displayResult(const std::vector<GraspableRegion>& regions, const std::vector<int>& route, const std::vector<Eigen::Vector2i>& coords_id) const;

		void getGraspableRegion(const cnoid::Vector3& pos, GraspableRegion& region);
		bool isGraspableParallel(RobotBodyPtr body, double x, double y, int* has_sol);

    PlanBase* tc;
		VirtualObjectHandler voh;
		unsigned int x_trans_joint_id;
		unsigned int y_trans_joint_id;
		cnoid::Vector3 obj_corner_pos[4];
		GraspDatabaseManipulator::GraspPoses grasp_postures;
		std::vector<GraspPosture*> grasp_candidate[4];
		std::vector<std::vector<int> > infeasible_rid;
		cnoid::Vector3 graspable_pos[4];
		cnoid::Link* target_object;
		BoxInfoPtr target_box_info;
		PoseHolder original_pose;
		PoseHolder initial_pose;
		PoseHolder graspable_pose;
		int target_arm_id;

		WaistSearchParam params;

  };
}

#endif /* _GRASPDATAGENRATOR_WAISTPOSITIONSEARCHER_H_ */
