#ifndef MANIPFAILUREANALYZER_H
#define MANIPFAILUREANALYZER_H

#include "ManipController.h"

namespace grasp{
namespace PickAndPlacePlanner{
	
class ManipFailureAnalyzer : public ManipController {
public:
	static ManipFailureAnalyzer* instance();

	typedef size_t GraspID;
	typedef size_t PutID;
	typedef std::vector<GraspID> GraspIDVec;
	typedef std::pair<GraspID, PutID> IDPair;
	typedef std::vector<IDPair> IDPairVec;

	enum COLLISION_PART {FING = 0x1, ARM = 0x2, BODY = 0x4, OBJ = 0x8, ENV = 0x10};
	typedef std::vector<std::pair<COLLISION_PART, COLLISION_PART> > COLPAIR;
	typedef unsigned int COLLISION_PART_FLAG;

	enum RESULT_TYPE {NOT_TRY, SUCCESS, IKFAIL, COLLISION};

	class Result {
		public:
			RESULT_TYPE result;
			COLPAIR colpair;
			std::vector<cnoid::Vector3> col_p;

			void addCollisions(const std::vector<PlanBase::ColPair>& collisions);
			bool hasCollision(COLLISION_PART_FLAG col1, COLLISION_PART_FLAG col2) const;
		};

	struct GraspResult {
		Result approach;
		Result pregrasp;
		Result grasp;
		Result lift;
	};

	struct PlaceResult {
		Result approach;
		Result place;
		Result release;
	};

	void analyze();

	bool searchPutPos();
protected:
	ManipFailureAnalyzer() {;}
	ManipFailureAnalyzer(const ManipFailureAnalyzer&) {;}
	ManipFailureAnalyzer& operator=(const ManipFailureAnalyzer& r) {;}

	virtual ~ManipFailureAnalyzer() {;}

	void failuareCheck();
	void displayAnalysisResult();

	bool isCollidingAllCheck(int graspingState, int graspingState2, int contactState);

	void checkGraspPoses(GraspID gid);
	void checkPlacePoses(GraspID gid, PutID pid);

	bool isGraspAllIKFail() const;
	bool isGraspAllCollision() const;
	bool isApproachAllFail(const GraspIDVec& gids) const;
	bool isPreGraspAllCollision(const GraspIDVec& gids, GraspIDVec& col_ids) const;
	bool isLiftAllFail(const GraspIDVec& gids) const;
	bool isPlaceAllIKFail() const;
	bool isReleaseAllCollision(const IDPairVec& pids, IDPairVec& col_ids) const;
	bool isPlaceApproachAllFail(const IDPairVec& pids) const;
	GraspIDVec getGraspSuccessIDs() const;
	IDPairVec getPlaceSuccessIDs() const;

	void getColPoints(std::vector<cnoid::Vector3>& colpoints);

	void searchFeasiblePreGraspFingerAngle(const GraspIDVec& target_gids, std::vector<double>& feasible_offset);
	void searchFeasibleReleaseFingerAngle(const IDPairVec& target_pids, std::vector<double>& feasible_offset);

	bool interpolateGraspPose(GraspDatabaseManipulator::GraspPoses& inserted_poses);
	void interpolateGraspPoseSub(bool is_grasp, PutID pid, GraspDatabaseManipulator::GraspPoses& inserted_poses);

	void pickAndPlaceTest(const GraspDatabaseManipulator::GraspPoses& poses, GraspDatabaseManipulator::GraspPoses& feasible_poses);

	bool sortPutPos();

	std::vector<GraspResult> grasp_result;
	std::vector<std::vector<PlaceResult> > place_result;
private:
	cnoid::Vector3 Po_ini;
	cnoid::Matrix3 Ro_ini;
	std::vector<cnoid::Vector3> Po_des;
	std::vector<cnoid::Matrix3> Ro_des;
};

}
}
#endif
