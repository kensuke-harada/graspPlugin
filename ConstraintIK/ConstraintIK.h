#ifndef _CONSTRAINTIK_CONSTRAINTIK_H_
#define _CONSTRAINTIK_CONSTRAINTIK_H_

#include <fstream>
#include <vector>

#include <cnoid/BodyItem>

#include "ObstacleShape.h"
#include "../Grasp/Camera.h"
#include "exportdef.h"

namespace grasp {

	class AbstractIKPath;

  class EXCADE_API ConstraintIKSolver {
  public:
    enum DistCalculator {APPROXIMATE, STEEPESTDESCENT};

    explicit ConstraintIKSolver(ArmFingers* _arm);
		ConstraintIKSolver(cnoid::BodyPtr _body, ArmPtr _arm);
    virtual ~ConstraintIKSolver();
		
		bool init();
		bool ik(bool is_restore = true);
		bool ik(int& success_step, bool is_restore = true);

		bool ik(const cnoid::Vector3& p, const cnoid::Matrix3& R);
    bool ik(const cnoid::Vector3& p, const cnoid::Matrix3& R, int& success_step, bool is_restore = true);
		void setArmPtr(ArmPtr _arm);
    void setStep(unsigned int _step);
    void setMaxIteration(unsigned int _ite);
    void setTranslationalJointCoeff(double coeff);
    void setFixJoint(unsigned int joint_num);
    void unsetFixJoint(unsigned int joint_num);
    void setDistCalculator(DistCalculator calc);
    void addObstacleShapes(cnoid::BodyItemPtr item);

		void addIKArm(AbstractIKPath* ik_path) {target_paths.push_back(ik_path);}

  protected:
    bool ik_sub(const cnoid::Vector3& dp, const cnoid::Vector3& omega)const;
		bool ik_sub(const cnoid::VectorXd& dg,  bool enable_error = false, double error_weight = 1.0);
    void setAngleConstraint(cnoid::MatrixXd& CI, cnoid::VectorXd& ci0) const;
    void setCollisionConstraint(cnoid::MatrixXd& CI, cnoid::VectorXd& ci0) const;
    void calcJacobian(cnoid::MatrixXd& J, int target_joint_num, const cnoid::Vector3& target_point) const;
		double getAABBdist(EllipsoidShape* e1, EllipsoidShape* e2) const;
		
		bool isDownwardJoint(int target_joint_num, int tip_joint_num) const;
		cnoid::JointPathPtr getJointPath(int joint_id) const;

    void storeq();
    void restoreq() const;

		void readYaml();

		void mappingJacobian(int path_id, cnoid::MatrixXd& J);
		void mappingG0(int path_id, cnoid::VectorXd& g0);

		std::vector<AbstractIKPath*> target_paths;
		std::vector<std::vector<int> > lut_joint;
    unsigned int step;
    unsigned int max_adjustment_ite;
    double translation_coeff;
    ArmPtr arm;
		cnoid::BodyPtr body;

    ObstacleShapeVec obstacle_shapes;
    ArmShapeVec arm_shapes;
    AbstractEllipsoidDistCalculator* dist_calc;

    std::vector<double> qorg;
    std::vector<bool> is_joint_fix;
		std::vector<bool> weight;
		std::vector<int> target_jointid;

    std::ostream& os;
  };

	class AbstractIKPath {
	public:
		AbstractIKPath() {;}
		virtual ~AbstractIKPath() {;}

		virtual void calcJacobian(cnoid::MatrixXd& J) const = 0;
		virtual bool getGoalDiff(cnoid::VectorXd& g) const = 0;
		virtual void calcG0(cnoid::VectorXd& g) const = 0;

		void setJointPath(const cnoid::JointPathPtr joint_path) {path = joint_path;}
		cnoid::JointPathPtr getJointPath() const {return path;}
		int getDimension() const {return dimension;}
		void setG0Weght(double weight) {g0_weight = weight;}
	protected:
		cnoid::JointPathPtr path;
		cnoid::VectorXd goal;
		int dimension;
		double g0_weight;

		double maxIkErrorSqr;
	};

	class EXCADE_API ArmIKPath : 
		public AbstractIKPath {
	public:
		ArmIKPath() {dimension = 6; maxIkErrorSqr = 1.0e-6 * 1.0e-6;}
		~ArmIKPath() {;}

		void calcJacobian(cnoid::MatrixXd& J) const;
		bool getGoalDiff(cnoid::VectorXd& g) const;
		void calcG0(cnoid::VectorXd& g) const {g.resize(path->numJoints()); g.setZero();}

		void setGoal(const cnoid::Vector3& p, const cnoid::Matrix3& R);
		void setArm(ArmPtr _arm) {arm = _arm; path = arm->arm_path;}
	private:
		cnoid::Vector3 goal_p;
		cnoid::Matrix3 goal_R;
		ArmPtr arm;
	};

	class CameraIKPath : 
		public AbstractIKPath{
	public:
		CameraIKPath() {;}
		virtual ~CameraIKPath() {;}

		void setTargetPoint(const cnoid::Vector3& p) {target_point = p;}
		void setCamera(CameraPtr _camera) {camera = boost::dynamic_pointer_cast<AttachedCamera>(_camera); path = camera->getPath();}
	protected:
		cnoid::Vector3 target_point;
		AttachedCameraPtr camera;

		static cnoid::Vector3 omegaFromTwoVecs(const cnoid::Vector3& vec1, const cnoid::Vector3& vec2);
	};

	class EXCADE_API Camera2DimensionIKPath :
		public CameraIKPath {
	public:
		Camera2DimensionIKPath() {dimension = 2; maxIkErrorSqr = 1.0e-2 * 1.0e-2;}
		~Camera2DimensionIKPath() {;}

		void  calcJacobian(cnoid::MatrixXd& J) const;
		bool getGoalDiff(cnoid::VectorXd& g) const;
		void calcG0(cnoid::VectorXd& g) const {g.resize(path->numJoints()); g.setZero();}

	private:
		double getAngle(const cnoid::Vector3& v1, const cnoid::Vector3& v2, int axis = 2) const;
		double getAngleAxis(const cnoid::Vector3& v, int axis = 2) const;
	};

	class EXCADE_API Camera3DimensionIKPath :
		public CameraIKPath {
	public:
		Camera3DimensionIKPath() {dimension = 3;  maxIkErrorSqr = 1.0e-2 * 1.0e-2;}
		~Camera3DimensionIKPath() {;}

		void calcJacobian(cnoid::MatrixXd& J) const;
		bool getGoalDiff(cnoid::VectorXd& g) const;
		void calcG0(cnoid::VectorXd& g) const {g.resize(path->numJoints()); g.setZero();}
	};

	class EXCADE_API HandCameraIKPath :
		public CameraIKPath {
	public:
		HandCameraIKPath() {dimension = 4; maxIkErrorSqr = 1.0e-2 * 1.0e-2; g0_weight = 0.1;}
		~HandCameraIKPath() {;}

		void calcJacobian(cnoid::MatrixXd& J) const;
		bool getGoalDiff(cnoid::VectorXd& g) const;
		void calcG0(cnoid::VectorXd& g) const;
	};

}

#endif /* _CONSTRAINTIK_CONSTRAINTIK_H_ */
