#include "ConstraintIK.h"

#include <math.h>
#include <limits>

#include <cnoid/JointPath>
#include <cnoid/MessageView>

#include "../Grasp/VectorMath.h"
#include "../Grasp/DrawUtility.h"
#include "../Grasp/PlanBase.h"

#include "eiquadprog.hpp"

//#define DEBUG_MODE // only windows
#ifdef DEBUG_MODE
#include <windows.h>
#endif

using namespace std;
using namespace cnoid;
using namespace grasp;

ConstraintIKSolver::ConstraintIKSolver(ArmFingers* _arm) : os(MessageView::mainInstance()->cout()) {
  if (_arm == NULL) {
    arm = NULL;
  } else {
    arm = _arm->arm;
		body = _arm->bodyItemRobot->body();
    is_joint_fix = vector<bool>(body->numJoints(), false);
  }
  step = 20;
  max_adjustment_ite = 10;
  translation_coeff = 1.0;
  obstacle_shapes.clear();
  arm_shapes.clear();
	target_paths.clear();
  dist_calc = new ApproximateEllipsoidDistCalculator();
  readYaml();
}

ConstraintIKSolver::ConstraintIKSolver(BodyPtr _body, ArmPtr _arm) : os(MessageView::mainInstance()->cout()) {
  if (_arm == NULL) {
    arm = NULL;
  } else {
    arm = _arm;
		body = _body;
    is_joint_fix = vector<bool>(body->numJoints(), false);
  }
  step = 20;
  max_adjustment_ite = 10;
  translation_coeff = 1.0;
  obstacle_shapes.clear();
  arm_shapes.clear();
	target_paths.clear();
  dist_calc = new ApproximateEllipsoidDistCalculator();
  readYaml();
}

ConstraintIKSolver::~ConstraintIKSolver() {
  for (size_t i = 0; i < obstacle_shapes.size(); i++) {
    delete obstacle_shapes[i];
		obstacle_shapes[i] = NULL;
  }
  for (size_t i = 0; i < arm_shapes.size(); i++) {
    delete arm_shapes[i];
		arm_shapes[i] = NULL;
  }
  delete dist_calc;
	dist_calc = NULL;
}

/**
* Inverse kinematics under obstacle constraints.
* @param[in] p desired position
* @param[in] R desired rotation matrix
*/
bool ConstraintIKSolver::ik(const Vector3& p, const Matrix3& R) {
	int dummy;
	return ik(p, R, dummy, true);
}

bool ConstraintIKSolver::init() {
	target_jointid.clear();
	for (size_t i = 0; i < target_paths.size(); i++){
		lut_joint.push_back(std::vector<int>(target_paths[i]->getJointPath()->numJoints(),0));
		for (int j = 0; j < target_paths[i]->getJointPath()->numJoints(); j++) {
			bool included = false;
			for (size_t k = 0; k < target_jointid.size(); k++) {
				if (target_paths[i]->getJointPath()->joint(j)->jointId() == target_jointid[k]) {
					included = true;
					lut_joint[i][j] = k;
					break;
				}
			}
			if (!included) {
				target_jointid.push_back(target_paths[i]->getJointPath()->joint(j)->jointId());
				lut_joint[i][j] = target_jointid.size() - 1;
			}
		}
	}

	return true;
}

bool ConstraintIKSolver::ik(bool is_restore) {
	int dummy;
	return ik(dummy, is_restore);
}

bool ConstraintIKSolver::ik(int& success_step, bool is_restore) {
	int d = 0;
	for (size_t i = 0; i < target_paths.size(); i++){
		d += 	target_paths[i]->getDimension();
	}

  storeq();

	bool is_success = true;

  for (unsigned int i = 0; i < step; i++) {
		success_step = i;
		
		VectorXd dg(d);

		int cur = 0;
		for (size_t j = 0; j < target_paths.size(); j++){
			VectorXd g;
			target_paths[j]->getGoalDiff(g);
			dg.segment(cur, g.size()) = g;
			cur += g.size();
		}

    if (!ik_sub(dg/(step-i), true, 1000)) {
		//if (!ik_sub(dg/(step-i))) {
			if(is_restore)restoreq();
      return false;
    }
  }

  for (unsigned int i = 0; i < max_adjustment_ite; i++) {
		VectorXd dg(d);

		int cur = 0;
		bool converged = true;
		for (size_t j = 0; j < target_paths.size(); j++){
			VectorXd g;
			converged &= target_paths[j]->getGoalDiff(g);
			dg.segment(cur, g.size()) = g;
			cur += g.size();
		}

		if (converged) {
      return true;
    }

		//if (!ik_sub(dg, true, 100)) {
    if (!ik_sub(dg, false)) {
      if(is_restore)restoreq();
      return false;
    }
  }

  if(is_restore)restoreq();
  return false;
}

bool ConstraintIKSolver::ik(const Vector3& p, const Matrix3& R, int& success_step, bool is_restore) {
#ifdef DEBUG_MODE
  PlanBase* tc = PlanBase::instance();
#endif
  if (arm == NULL) {
    os << "error: Arm is not set!!(ConstraintIKSolver)" << endl;
    return false;
  }
  double maxIkErrorSqr = 1.0e-6 * 1.0e-6;

	const int n = arm->arm_path->numJoints();

	target_jointid.clear();
	for (int i = 0; i < n; i++) {
		target_jointid.push_back(arm->arm_path->joint(i)->jointId());
	}

  storeq();

  bool is_success = true;

	// approach phase
  for (unsigned int i = 0; i < step; i++) {
    Vector3 dp(p - arm->palm->p());
    Vector3 omega(arm->palm->R() * omegaFromRot((arm->palm->R()).transpose()* R));
		success_step = i;
    if (isnan(dot(omega, omega))) {  // To remove
      omega << 0, 0, 0;
    }

		if (!ik_sub(dp/(step-i), omega/(step-i))) {
			if(is_restore)restoreq();
      return false;
    }
#ifdef DEBUG_MODE
    tc->flush();
    Sleep(50);
#endif
  }

	// adjustment pahse
  for (unsigned int i = 0; i < max_adjustment_ite; i++) {
    Vector3 dp(p - arm->palm->p());
    Vector3 omega(arm->palm->R() * omegaFromRot((arm->palm->R()).transpose()* R));

    double errsqr = dot(dp, dp) + dot(omega, omega);

    if (isnan(dot(omega, omega))) {  // To remove
      errsqr = dot(dp, dp);
      omega << 0, 0, 0;
    }

    if (errsqr < maxIkErrorSqr) {
      return true;
    }

		if (!ik_sub(dp, omega)) {
      if(is_restore)restoreq();
      return false;
    }
#ifdef DEBUG_MODE
    tc->flush();
    Sleep(50);
#endif
  }

  if(is_restore)restoreq();
  return false;
}



void ConstraintIKSolver::setArmPtr(ArmPtr _arm) {
  arm = _arm;
}

/**
* Set the number of division from start position to desired position(default value:20).
* @param[in] _step number of division
*/
void ConstraintIKSolver::setStep(unsigned int _step) {
  step = _step;
}

/**
* Set the maximum iteration limit in the adjustment phase(default value:10).
* @param[in] _ite maximum iteration limit
*/
void ConstraintIKSolver::setMaxIteration(unsigned int _ite) {
  max_adjustment_ite = _ite;
}

/**
* Set the coefficent to translational joints.
* If this is set to large value, translational joints become difficult to move compared with other joints.
* @param[in] _ite maximum iteration limit
*/
void ConstraintIKSolver::setTranslationalJointCoeff(double coeff) {
  translation_coeff = coeff;
}

/**
* Set the joint is fixed during IK.
* @param[in] joint_num joint number
*/
void ConstraintIKSolver::setFixJoint(unsigned int joint_num) {
  if (joint_num < is_joint_fix.size()) {
    is_joint_fix[joint_num] = true;
  }
}

void ConstraintIKSolver::unsetFixJoint(unsigned int joint_num) {
  if (joint_num < is_joint_fix.size()) {
    is_joint_fix[joint_num] = false;
  }
}

void ConstraintIKSolver::setDistCalculator(DistCalculator calc) {
  if (dist_calc != NULL) delete dist_calc;
  switch (calc) {
  case APPROXIMATE:
    dist_calc = new ApproximateEllipsoidDistCalculator();
    break;
  case STEEPESTDESCENT:
    dist_calc = new SteepestDescentEllipsoidDistCalculator();
    break;
  default:
    dist_calc = new ApproximateEllipsoidDistCalculator();
  }
}

void ConstraintIKSolver::addObstacleShapes(BodyItemPtr item) {
  ObstacleShapeVec shapes = ObstacleShape::readYamlFile(item);
  for (ObstacleShapeVecConstIterator vi = shapes.begin(); vi != shapes.end(); ++vi) {
    obstacle_shapes.push_back(*vi);
  }
}

bool ConstraintIKSolver::ik_sub(const VectorXd& dg, bool enable_error, double error_weight) {
	const int n = target_jointid.size();

	int p = dg.size();

	int n_joint = (enable_error ? n + p : n);

	MatrixXd CE = MatrixXd::Zero(n_joint, p);
	VectorXd ce0(p);

	const int os_size = obstacle_shapes.size();
  const int as_size = arm_shapes.size();

  for (int i = 0; i < os_size; i++) {
    obstacle_shapes[i]->updatePos();
  }
  for (int i = 0; i < as_size; i++) {
    arm_shapes[i]->updatePos();
  }

	int cur = 0;
	for (size_t i=0; i<target_paths.size(); i++) {
		MatrixXd J;
		target_paths[i]->calcJacobian(J);
		mappingJacobian(i, J);
		int d = target_paths[i]->getDimension();
		
		CE.block(0, cur, n, d) = J.transpose();
		cur += d;
	}
	if (enable_error) {
		CE.bottomRows(p) = MatrixXd::Identity(p,p);
	}
	
  MatrixXd G;
  G = MatrixXd::Identity(n_joint, n_joint);
  for (int i = 0; i < n; i++) {
    if (body->joint(target_jointid[i])->jointType() == Link::SLIDE_JOINT) {
      G(i, i) = translation_coeff;
    }
  }

	if (enable_error) {
		for (int i = 0; i < p; i++) {
			G(n+i, n+i) = error_weight;
		}
	}

  VectorXd g0;
  g0 = VectorXd::Zero(n_joint);
	if (enable_error) {
		for (size_t i=0; i<target_paths.size(); i++) {
			VectorXd gtmp;
			target_paths[i]->calcG0(gtmp);
			mappingG0(i, gtmp);
			g0.head(n) += gtmp;
		}

		if (enable_error) {
			g0.tail(p).setZero();
		}
	}

  ce0 = -dg;

  MatrixXd CI(n_joint, 2*n+os_size*as_size);
  VectorXd ci0;
  CI = MatrixXd::Zero(n_joint, 2*n+os_size*as_size);
  ci0 = VectorXd::Zero(2*n+os_size*as_size);

  setAngleConstraint(CI, ci0);
  setCollisionConstraint(CI, ci0);
	
  VectorXd x;

	double q_norm = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
  if (q_norm == std::numeric_limits<double>::infinity()) {
    return false;
  }

	for (int j = 0; j < n; ++j) {
		body->joint(target_jointid[j])->q() += x(j);
		if (body->joint(target_jointid[j])->q() > body->joint(target_jointid[j])->q_upper()) {
			body->joint(target_jointid[j])->q() =  body->joint(target_jointid[j])->q_upper();
		}
		if (body->joint(target_jointid[j])->q() < body->joint(target_jointid[j])->q_lower()) {
			body->joint(target_jointid[j])->q() =  body->joint(target_jointid[j])->q_lower();
		}
	}
	body->calcForwardKinematics();
  return true;
}

void ConstraintIKSolver::mappingJacobian(int path_id, MatrixXd& J) {
	const int n = target_paths[path_id]->getJointPath()->numJoints();
	MatrixXd tmpJ = J;
	J.resize(target_paths[path_id]->getDimension(), target_jointid.size());
	J.setZero();
	for (int i = 0; i < n; i++) {
		J.col(lut_joint[path_id][i]) = tmpJ.col(i);
	}
}

void ConstraintIKSolver::mappingG0(int path_id, VectorXd& g0) {
	const int n = target_paths[path_id]->getJointPath()->numJoints();
	VectorXd tmpg = g0;
	g0.resize(target_jointid.size());
	g0.setZero();
	for (int i = 0; i < n; i++) {
		g0(lut_joint[path_id][i]) = tmpg(i);
	}
}

bool ConstraintIKSolver::ik_sub(const Vector3& dp, const Vector3& omega) const {
  const int n = arm->arm_path->numJoints();
  const int os_size = obstacle_shapes.size();
  const int as_size = arm_shapes.size();

  for (int i = 0; i < os_size; i++) {
    obstacle_shapes[i]->updatePos();
  }
  for (int i = 0; i < as_size; i++) {
    arm_shapes[i]->updatePos();
  }

  MatrixXd J(6, n);

  arm->arm_path->calcJacobian(J);

  VectorXd dq(n);
  VectorXd v(6);

  setVector3(dp   , v, 0);
  setVector3(omega, v, 3);

  MatrixXd G;
  G = MatrixXd::Identity(n, n);
  for (int i = 0; i < n; i++) {
    if (arm->arm_path->joint(i)->jointType() == Link::SLIDE_JOINT) {
      G(i, i) = translation_coeff;
    }
  }

  VectorXd g0;
  g0 = VectorXd::Zero(n);

  MatrixXd CE = J.transpose();
  VectorXd ce0 = -v;

  MatrixXd CI(n, 2*n+os_size*as_size);
  VectorXd ci0;
  CI  = MatrixXd::Zero(n, 2*n+os_size*as_size);
  ci0  = VectorXd::Zero(2*n+os_size*as_size);

  setAngleConstraint(CI, ci0);
  setCollisionConstraint(CI, ci0);

  VectorXd x;
  double q_norm = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
  if (q_norm == std::numeric_limits<double>::infinity()) {
    return false;
  }
  for (int j = 0; j < n; ++j) {
    arm->arm_path->joint(j)->q() += x(j);
		if (arm->arm_path->joint(j)->q()> arm->arm_path->joint(j)->q_upper()) {
			arm->arm_path->joint(j)->q()=  arm->arm_path->joint(j)->q_upper();
		}
		if (arm->arm_path->joint(j)->q() < arm->arm_path->joint(j)->q_lower()) {
			arm->arm_path->joint(j)->q() =  arm->arm_path->joint(j)->q_lower();
		}
  }
  arm->arm_path->calcForwardKinematics();
  return true;
}

void ConstraintIKSolver::setAngleConstraint(MatrixXd& CI, VectorXd& ci0) const {
  const int n = target_jointid.size();

  for (int i = 0; i < n; i++) {
    CI(i, i) = -1;
    CI(i, i+n) = 1;
  }

  for (int i = 0; i < n; i++) {
    if (is_joint_fix[target_jointid[i]]) {
      ci0(i) = 0;
      ci0(i+n) = 0;
    } else {
      ci0(i) = body->joint(target_jointid[i])->q_upper() - body->joint(target_jointid[i])->q();
      ci0(i+n) = body->joint(target_jointid[i])->q() - body->joint(target_jointid[i])->q_lower();
    }
  }
}

void ConstraintIKSolver::setCollisionConstraint(MatrixXd& CI, VectorXd& ci0) const {
  const int n = target_jointid.size();
  const int os_size = obstacle_shapes.size();
  const int as_size = arm_shapes.size();

  for (int i = 0; i < os_size; i++) {
    double min_interval = obstacle_shapes[i]->getMinInterval();
    double effective_dist = obstacle_shapes[i]->getEffectiveDist();
    double xi = obstacle_shapes[i]->getXi();
    for (int j = 0; j < as_size; j++) {
      Vector3 dir;
      double dist;
      Vector3 p1, p2;

      if (norm2(arm_shapes[j]->getP() - obstacle_shapes[i]->getP()) - arm_shapes[j]->getRadius().maxCoeff() - obstacle_shapes[i]->getRadius().maxCoeff()  > effective_dist) continue;
			if (getAABBdist(arm_shapes[j], obstacle_shapes[i]) > effective_dist) continue;
      dist = dist_calc->calcDistance(obstacle_shapes[i], arm_shapes[j], p1, p2);

      if (dist > effective_dist) continue;
      dir  = unit(p1 - p2);

      VectorXd nj;
      MatrixXd Jt;
      //calcJacobian(Jt, arm_shapes[j]->getJointNum(), p2);
			calcJacobian(Jt, arm_shapes[j]->getJointID(), p2);
      nj = -dir.transpose() * Jt;

      for (int k = 0; k < n; k++) {
        CI(k, 2*n+i*as_size+j) = nj(k);
      }
      ci0(2*n+i*as_size+j) = xi * (dist-min_interval)/(effective_dist - min_interval);
    }
  }
}

void ConstraintIKSolver::calcJacobian(MatrixXd& J, int target_joint_num, const Vector3& target_point) const {
  const int n = target_jointid.size();
  J.resize(3, n);

  if (n <= 0) return;

  for (int i = 0; i < n; ++i) {
		if (!isDownwardJoint(target_jointid[i], target_joint_num)) {
      J.col(i).setZero();
      continue;
    }
		
		Link* link = body->joint(target_jointid[i]);
		JointPathPtr jointpath = getJointPath(target_jointid[i]);

    switch (link->jointType()) {
    case Link::ROTATIONAL_JOINT:
      {
        Vector3 omega = link->R() * link->a();
        const Vector3 len = target_point - link->p();
        if (!jointpath->isJointDownward(jointpath->indexOf(link))) {
          omega = -omega;
        }
        J.col(i) << omega.cross(len);
      }
      break;
    case Link::SLIDE_JOINT:
      {
        Vector3 dp = link->R() * link->d();
        if (!jointpath->isJointDownward(jointpath->indexOf(link))) {
          dp = -dp;
        }
        J.col(i) << dp;
      }
      break;
    default:
      J.col(i).setZero();
    }
  }
}

double ConstraintIKSolver::getAABBdist(EllipsoidShape* e1, EllipsoidShape* e2) const {
	Vector3 aabb1_min = Vector3::Ones() * DBL_MAX;
	Vector3 aabb1_max = -Vector3::Ones() * DBL_MAX;
	Vector3 aabb2_min = -(e2->getRadius().cwiseAbs());
	Vector3 aabb2_max = e2->getRadius().cwiseAbs();

	const unsigned int x_bit = 0x0001;
  const unsigned int y_bit = 0x0002;
  const unsigned int z_bit = 0x0004;

  for (unsigned int i = 0; i < 8; i++) {
    int sign_x = (i & x_bit) ? 1 : -1;
    int sign_y = (i & y_bit) ? 1 : -1;
    int sign_z = (i & z_bit) ? 1 : -1;
    Vector3 ver = Vector3(sign_x * e1->a(), sign_y * e1->b(), sign_z * e1->c());
		Vector3 v = trans(e2->getR()) * ((e1->getR() * ver) + e1->getP() - e2->getP());
		for (int j = 0; j < 3; j++) {
			if (aabb1_min(j) > v(j)) aabb1_min(j) = v(j);
			if (aabb1_max(j) < v(j)) aabb1_max(j) = v(j);
		}
  }

	double dist = 0.0;

	for (int i = 0; i < 3; i++) {
		if (aabb1_min(i) > aabb2_max(i)) {
			double d = aabb1_min(i) - aabb2_max(i);
			dist += d * d;
		} else if (aabb1_max(i) < aabb2_min(i)) {
			double d = aabb2_min(i) - aabb1_max(i);
			dist += d * d;
		}
	}

	return std::sqrt(dist);
}

bool ConstraintIKSolver::isDownwardJoint(int target_joint_num, int tip_joint_num) const {
	int n;
	bool is_target_path;

	for (vector<AbstractIKPath*>::const_iterator vi = target_paths.begin(); vi != target_paths.end(); ++vi) {
		JointPathPtr target_path = (*vi)->getJointPath();
		n = target_path->numJoints();
		is_target_path = false;
		for (int i = n - 1; i >= 0; i--) {
			int joint_id = target_path->joint(i)->jointId();
			if (joint_id == tip_joint_num) {
				is_target_path = true;
			}
			if (is_target_path && (joint_id == target_joint_num)) {
				return true;
			}
		}
	}

	return false;
}

/**
* Get the JontPathPtr which includes joint id @c joint_id.
* @param[in] joint_id target joint id
* @return JointPathPtr
*/
JointPathPtr ConstraintIKSolver::getJointPath(int joint_id) const {
	for (vector<AbstractIKPath*>::const_iterator vi = target_paths.begin(); vi != target_paths.end(); ++vi) {
		JointPathPtr traget_path = (*vi)->getJointPath();
		for (int i = 0; i < traget_path->numJoints(); i++) {
			if (traget_path->joint(i)->jointId() == joint_id) {
				return traget_path;
			}
		}
	}
	return arm->arm_path;
}


void ConstraintIKSolver::storeq() {
  qorg.clear();
	for (int i = 0; i < body->numJoints(); i++) {
		qorg.push_back(body->joint(i)->q());
	}
}

void ConstraintIKSolver::restoreq() const {
	for (int i = 0; i < body->numJoints(); i++) {
		body->joint(i)->q() = qorg[i];
	}
	body->calcForwardKinematics();
}

void ConstraintIKSolver::readYaml() {
  ObstacleParameterReader opr;
	if (arm != NULL) {
		opr.readRobotYamlFile(body, arm);
	}
  opr.readEnvYamlFiles(body);
  obstacle_shapes = opr.getObstacleShapes();
  arm_shapes = opr.getArmShapes();
}

/**
* Calculate Jacobian matrix.
* @param[out] J Jacobian matrix
*/
void ArmIKPath::calcJacobian(MatrixXd& J) const {
	path->calcJacobian(J);
}

/**
* Get the difference between desired state and current state.
* @param[out] g 6-vector consisting of linear velocity vector and angular velocity vector.
* @return true if the difference is sufficently small.
*/
bool ArmIKPath::getGoalDiff(VectorXd& g) const {
	g.resize(dimension);
	Vector3 dp(goal_p - arm->palm->p());
  Vector3 omega(arm->palm->R() * omegaFromRot((arm->palm->R()).transpose()* goal_R));

	if(isnan(dot(omega, omega))){
		omega << 0, 0, 0;
	}
	g << dp, omega;
	return (maxIkErrorSqr > (dot(dp, dp) + dot(omega, omega)));
}

/**
* Set desired position and rotaion matrix.
* @param[in] p desired position
* @param[in] R desired rotation matrix
*/
void ArmIKPath::setGoal(const Vector3& p, const Matrix3& R) {
	goal_p = p;
	goal_R = R;
}

/**
* Calculate angular velocity between two vectors.
* Return 0 if angle between two vectors is nearly 0 or pi.
* @param[in] vec1,vec2 vector
* @return angular velocity
*/
Vector3 CameraIKPath::omegaFromTwoVecs(const Vector3& vec1, const Vector3& vec2) {
	const double epsilon = 1.0e-6;
	Vector3 v1 = unit(vec1);
	Vector3 v2 = unit(vec2);
	Vector3 omega = Vector3::Zero();

	double q = acos(dot(v1, v2));
	Vector3 axis = cross(v1, v2);
	double len = norm2(axis);
	if (len < epsilon) return omega;
	axis = axis/norm2(axis);
	omega = q * axis;
	return omega;
}

/**
* Calculrate Jacobian matrix.
* @param[out] J Jacobian matrix
*/
void Camera2DimensionIKPath::calcJacobian(MatrixXd& J) const {
	const int n = path->numJoints();

	J.resize(dimension, n);

	Vector3 camera_dir_ori = unit(camera->getCameraLink()->R() * camera->getDirection());
	Vector3 desired_dir_ori = unit(target_point - camera->getPosition());

	const double theta2_ori = getAngle(camera_dir_ori, desired_dir_ori);
	const double theta1_ori = getAngleAxis(camera_dir_ori) - getAngleAxis(desired_dir_ori);

	for (int i = 0; i < n; i++) {
		Link* link = path->joint(i);

		switch (link->jointType()) {
		case Link::ROTATIONAL_JOINT:
			{
				Vector3 omega = link->R() * link->a();
				if (!path->isJointDownward(i)) {
					omega = -omega;
				}
				Vector3 camera_dir = unit(rodrigues(omega, 1.0) * camera->getCameraLink()->R() * camera->getDirection());
				Vector3 len = camera->getPosition() - link->p();
				Vector3 desired_dir =unit(target_point - (camera->getPosition() + omega.cross(len)));

				int sign_cam = (fabs(getAngle(camera_dir_ori, camera_dir)) > M_PI/2.0) ? -1 : 1;
				int sign_tar = (fabs(getAngle(desired_dir_ori, desired_dir)) > M_PI/2.0) ? -1 : 1;
				double theta1 = sign_cam * getAngleAxis(camera_dir) - sign_tar * getAngleAxis(desired_dir) - theta1_ori;
				double theta2 = getAngle(camera_dir, desired_dir) - theta2_ori;
				if (theta2 > M_PI) theta2 -= 2 * M_PI;
				if (theta2 < -M_PI) theta2 += 2 * M_PI; 
				if (theta2 > M_PI/2.0) theta2 -= M_PI;
				if (theta2 < -M_PI/2.0) theta2 += M_PI; 
				if (theta1 > M_PI/2.0) theta1 -= M_PI;
				if (theta1 < -M_PI/2.0) theta1 += M_PI; 

				J.col(i) << theta1, theta2;
			}
			break;
		case Link::SLIDE_JOINT:
			{
				Vector3 dp = link->R() * link->d();
				if (!path->isJointDownward(i)) {
					dp = -dp;
				}
				Vector3 camera_dir = unit(camera->getCameraLink()->R() * camera->getDirection());
				Vector3 desired_dir = unit(target_point - (camera->getPosition() + dp));

				int sign_cam = (fabs(getAngle(camera_dir_ori, camera_dir)) > M_PI/2.0) ? -1 : 1;
				int sign_tar = (fabs(getAngle(desired_dir_ori, desired_dir)) > M_PI/2.0) ? -1 : 1;
				double theta1 = sign_cam * getAngleAxis(camera_dir) - sign_tar * getAngleAxis(desired_dir) - theta1_ori;
				double theta2 = getAngle(camera_dir, desired_dir) - theta2_ori;
				if (theta2 > M_PI) theta2 -= 2 * M_PI;
				if (theta2 < -M_PI) theta2 += 2 * M_PI; 
				if (theta2 > M_PI/2.0) theta2 -= M_PI;
				if (theta2 < -M_PI/2.0) theta2 += M_PI;
				if (theta1 > M_PI/2.0) theta1 -= M_PI;
				if (theta1 < -M_PI/2.0) theta1 += M_PI; 

				J.col(i) << theta1, theta2;
			}
			break;
		default:
			J.col(i).setZero();
		}
	}
}

/**
* Get the difference between desired state and current state.
* @param[out] g 2-vector consisting of angular velocity around horizontal axis and angular velocity around vertical axis.
* @return true if the difference is sufficently small.
*/
bool Camera2DimensionIKPath::getGoalDiff(VectorXd& g) const {
	g.resize(dimension);
	Vector3 camera_dir = unit(camera->getCameraLink()->R() * camera->getDirection());
	Vector3 desired_dir = unit(target_point - camera->getPosition());
	double theta1 = getAngleAxis(desired_dir) - getAngleAxis(camera_dir);
	double theta2 = getAngle(desired_dir, camera_dir);
	g << theta1, theta2;
	return (maxIkErrorSqr > theta1 * theta1 + theta2 * theta2);
}

/**
* Get angle between @c v1 and @c v2 around axis.
* @param[in] v1,v2 target vector
* @param[in] axis axis number (0:x, 1:y, 2:z)
* @return angle
*/
double Camera2DimensionIKPath::getAngle(const Vector3& v1, const Vector3& v2, int axis) const {
	Vector3 tmp_v1 = v1;
	tmp_v1(axis) = 0.0;
	tmp_v1 = unit(tmp_v1);

	Vector3 tmp_v2 = v2;
	tmp_v2(axis) = 0.0;
	tmp_v2 = unit(tmp_v2);

	if (tmp_v1 == tmp_v2) return 0.0;
	if (tmp_v1 == -tmp_v2) return M_PI;

	return atan2(cross(tmp_v1, tmp_v2)(axis), dot(tmp_v1, tmp_v2));
}

/**
* Get angle between vector and axis.
* @param[in] v target vector
* @param[in] axis axis number (0:x, 1:y, 2:z)
* @return angle
*/
double Camera2DimensionIKPath::getAngleAxis(const Vector3& v, int axis) const {
	Vector3 a = Vector3::Zero();
	a(axis) = 1.0;
	double t = asin(norm2(cross(v, a)));
	if (v(axis) > 0) t = M_PI-t;
	return t;
}

/**
* Calculrate Jacobian matrix.
* @param[out] J Jacobian matrix
*/
void Camera3DimensionIKPath::calcJacobian(MatrixXd& J) const {
	const int n = path->numJoints();

	J.resize(dimension, n);

	Vector3 camera_dir = camera->getCameraLink()->R() * camera->getDirection();
	Vector3 desired_dir = target_point - camera->getPosition();
	Vector3 omega_ori = omegaFromTwoVecs(camera_dir, desired_dir);

	for (int i = 0; i < n; i++) {
		Link* link = path->joint(i);

		switch (link->jointType()) {
		case Link::ROTATIONAL_JOINT:
			{
				Vector3 omega = link->R() * link->a();
				if (!path->isJointDownward(i)) {
					omega = -omega;
				}
				camera_dir = rodrigues(omega, 1.0) * camera->getCameraLink()->R() * camera->getDirection();
				Vector3 len = camera->getPosition() - link->p();
				desired_dir = target_point - (camera->getPosition() + omega.cross(len));
				
				J.col(i) << omegaFromTwoVecs(camera_dir, desired_dir) - omega_ori;
			}
			break;
		case Link::SLIDE_JOINT:
			{
				Vector3 dp = link->R() * link->d();
				if (!path->isJointDownward(i)) {
					dp = -dp;
				}
				camera_dir = camera->getCameraLink()->R() * camera->getDirection();
				desired_dir = target_point - (camera->getPosition() + dp);

				J.col(i) << omegaFromTwoVecs(camera_dir, desired_dir) - omega_ori;
			}
			break;
		default:
			J.col(i).setZero();
		}
	}
}

/**
* Get the difference between desired state and current state.
* @param[out] g angular velocity
* @return true if the difference is sufficently small.
*/
bool Camera3DimensionIKPath::getGoalDiff(VectorXd& g) const {
	g.resize(dimension);
	Vector3 camera_dir = camera->getCameraLink()->R() * camera->getDirection();
	Vector3 desired_dir = target_point - camera->getPosition();

	g << omegaFromTwoVecs(desired_dir, camera_dir);
	return (maxIkErrorSqr > dot(g,g));
}

/**
* Calculrate Jacobian matrix.
* @param[out] J Jacobian matrix
*/
void HandCameraIKPath::calcJacobian(MatrixXd& J) const {
	const int n = path->numJoints();

	J.resize(dimension, n);

	Vector3 camera_dir = camera->getCameraLink()->R() * camera->getDirection();
	Vector3 desired_dir = target_point - camera->getPosition();
	double dist = norm2(desired_dir);
	Vector3 omega_ori = omegaFromTwoVecs(camera_dir, desired_dir);

	for (int i = 0; i < n; i++) {
		Link* link = path->joint(i);

		switch (link->jointType()) {
		case Link::ROTATIONAL_JOINT:
			{
				Vector3 omega = link->R() * link->a();
				if (!path->isJointDownward(i)) {
					omega = -omega;
				}
				camera_dir = rodrigues(omega, 1.0) * camera->getCameraLink()->R() * camera->getDirection();
				Vector3 len = camera->getPosition() - link->p();
				desired_dir = target_point - (camera->getPosition() + omega.cross(len));
				
				J.col(i) << dist - norm2(desired_dir), omegaFromTwoVecs(camera_dir, desired_dir) - omega_ori;
			}
			break;
		case Link::SLIDE_JOINT:
			{
				Vector3 dp = link->R() * link->d();
				if (!path->isJointDownward(i)) {
					dp = -dp;
				}
				camera_dir = unit(camera->getCameraLink()->R() * camera->getDirection());
				desired_dir = target_point - (camera->getPosition() + dp);

				J.col(i) << dist - norm2(desired_dir), omegaFromTwoVecs(camera_dir, desired_dir) - omega_ori;
			}
			break;
		default:
			J.col(i).setZero();
		}
	}
}

/**
* Get the difference between desired state and current state.
* @param[out] g 4-vector consisting of linear velocity axis and angular velocity vector.
* @return true if the difference is sufficently small.
*/
bool HandCameraIKPath::getGoalDiff(VectorXd& g) const {
	g.resize(dimension);
	Vector3 camera_dir = camera->getCameraLink()->R() * camera->getDirection();
	Vector3 desired_dir = target_point - camera->getPosition();
	double d_dist = norm2(desired_dir) - camera->getFocalDistance();

	g << d_dist, omegaFromTwoVecs(desired_dir, camera_dir);
	return (maxIkErrorSqr > g.dot(g));
}

/**
* Calcurate g0 parameter of quadratic programming.
* This method sets the value to accerelate moving up camera in g0.
* @param[out] g g0 parameter.
*/
void HandCameraIKPath::calcG0(VectorXd& g) const {
	const int n = path->numJoints();

	g.resize(n);

	for (int i = 0; i < n; i++) {
		Link* link = path->joint(i);

		switch (link->jointType()) {
		case Link::ROTATIONAL_JOINT:
			{
				Vector3 omega = link->R() * link->a();
				if (!path->isJointDownward(i)) {
					omega = -omega;
				}
				Vector3 len = camera->getPosition() - link->p();
				
				g(i) = -len.z();
			}
			break;
		case Link::SLIDE_JOINT:
			{
				Vector3 dp = link->R() * link->d();
				if (!path->isJointDownward(i)) {
					dp = -dp;
				}
				
				g(i) = -dp.z();
			}
			break;
		default:
			g(i) = 0;
		}
	}

	g *= g0_weight;
}
