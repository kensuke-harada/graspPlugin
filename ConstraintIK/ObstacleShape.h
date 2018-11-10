#ifndef _CONSTRAINTIK_OBSTACLESHAPE_H_
#define _CONSTRAINTIK_OBSTACLESHAPE_H_

#include <vector>
#include "../Grasp/PlanBase.h"

namespace grasp {
	class ObjectShape;

	class EllipsoidShape;
	class ObstacleShape;
	class ArmShape;

	typedef std::vector<ArmShape*>           ArmShapeVec;
	typedef ArmShapeVec::iterator            ArmShapeVecIterator;
	typedef ArmShapeVec::const_iterator      ArmShapeVecConstIterator;
	typedef std::vector<ObstacleShape*>      ObstacleShapeVec;
	typedef ObstacleShapeVec::iterator       ObstacleShapeVecIterator;
	typedef ObstacleShapeVec::const_iterator ObstacleShapeVecConstIterator;

	class ObstacleShapeDrawer {
	public:
		ObstacleShapeDrawer(){;}
		virtual ~ObstacleShapeDrawer(){;}

		void show() const;
	};

	class ObstacleShapeHandler {
	public:
		ObstacleShapeHandler();
		virtual ~ObstacleShapeHandler();

		static ObstacleShapeHandler* instance(ObstacleShapeHandler *osh = NULL);

		void showObstacleShapes(bool is_boundingbox, bool is_output);

		void setNumCluster(int _num_cluster) {num_cluster = _num_cluster;}
		void setVolRatioTh(double _vol_ratio_threshold) {vol_ratio_threshold = _vol_ratio_threshold;}
		void setScale(double _scale) {scale = _scale;}

	protected:
		void addObstacleEllipsoidShapes(ObjectShape* obj);
		void addObstacleBoundingboxShapes(ObjectShape* obj);
		void removeDuplicatedEllipsoids();
		void writeObstacleShapes() const;
		void displayObstacleShapes() const;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ObjectShape* createObjectShape(cnoid::ColdetModelPtr c) const;
#else
		ObjectShape* createObjectShape(cnoid::SgNode* c) const;
#endif
		std::ostream& os;

		PlanBase* tc;
		int num_cluster;                        ///< number of clusters
		double vol_ratio_threshold;             ///< maximum allowable overlap ratio of bounding boxes
		double scale;                           ///< scaleing factor of the target object
		std::vector<EllipsoidShape> ellipsoids;

		const double security_distance;         ///< minimum allowable distace between ellipsoids
		const double effective_distance;        ///< distance that constraint condition becomes effective
		const double xi;                        ///< xi parameter
		const double ellip_radius;              ///< minimum radius of an ellpsoid which is placed on an edge of bounding box
		cnoid::Vector3 color;                   ///< color for display
		double alpha;
	};

	class ObstacleParameterReader {
	public:
		ObstacleParameterReader();
		virtual ~ObstacleParameterReader();

		void readRobotYamlFile(cnoid::BodyPtr body, ArmPtr arm);
		void readEnvYamlFiles(cnoid::BodyPtr body);
		void readEnvYamlFiles();
		void readEnvYamlFile(cnoid::BodyItemPtr item);

		void clear_arms();
		void clear_obstacles();

		ArmShapeVec      getArmShapes() const;
		ObstacleShapeVec getObstacleShapes() const;

	private:
		ArmShapeVec      arm_shapes;
		ObstacleShapeVec obstacle_shapes;
	};

	class EllipsoidShape {
	public:
		EllipsoidShape() {
			relR = cnoid::Matrix3::Identity();
			R = cnoid::Matrix3::Identity();
			p = cnoid::Vector3::Zero();
			radius = cnoid::Vector3::Zero();
			E = cnoid::Matrix3::Identity();
			invE = cnoid::Matrix3::Identity();
			sqrtE = cnoid::Matrix3::Identity();
			inv_sqrtE = cnoid::Matrix3::Identity();
		}
		virtual  ~EllipsoidShape() {;}

		void           setRelR(const cnoid::Matrix3& _relR) {relR = _relR;}
		cnoid::Matrix3 getRelR() const {return relR;}
		void           setR(const cnoid::Matrix3& _R) {R = _R;}
		cnoid::Matrix3 getR() const {return R;}
		void           setP(const cnoid::Vector3& _p) {p = _p;}
		cnoid::Vector3 getP() const {return p;}
		void           setRadius(const cnoid::Vector3& _radius) {radius = _radius;}
		cnoid::Vector3 getRadius() const {return radius;}
		double         a() const {return radius[0];}
		double         b() const {return radius[1];}
		double         c() const {return radius[2];}

		cnoid::Matrix3 getE() const {return E;}
		cnoid::Matrix3 getInvE() const {return invE;}
		cnoid::Matrix3 getSqrtE() const {return sqrtE;}
		cnoid::Matrix3 getInvSqrtE() const {return inv_sqrtE;}

	protected:
		void updateParameters();

		cnoid::Matrix3 relR;      ///< rotation matrix between link local and ellipsoid
		cnoid::Matrix3 R;         ///< rotation matrix between world and ellipsoid
		cnoid::Vector3 p;         ///< center position
		cnoid::Vector3 radius;    ///< radius

		cnoid::Matrix3 E;         ///< ellipsoid matrix
		cnoid::Matrix3 invE;      ///< inverse of the ellipsoid matrix
		cnoid::Matrix3 sqrtE;     ///< square root of the ellipsoid matrix
		cnoid::Matrix3 inv_sqrtE; ///< inverse square root of the ellipsoid matrix
	};

	class ObstacleShape : public EllipsoidShape {
	public:
		explicit ObstacleShape(cnoid::Link* _obj);
		virtual ~ObstacleShape();

		static ObstacleShapeVec readYamlFile(cnoid::BodyItemPtr item);

		cnoid::Vector3 getPos() const;
		void           setCenter(const cnoid::Vector3& _offset);
		cnoid::Vector3 getCenter() const {return offset;}
		double         getMinInterval() const {return min_interval;}
		void           setMinInterval(double _min_interval) {min_interval = _min_interval;}
		double         getEffectiveDist() const {return effective_dist;}
		void           setEffectiveDist(double _effective_dist) { effective_dist = _effective_dist;}
		double         getXi() const {return xi;}
		void           setXi(double _xi) {xi = _xi;}

		void updatePos();

	protected:
		cnoid::Vector3 offset;         ///< center position
		double         min_interval;   ///< minimum allowable distance between obstacle shapes
		double         effective_dist; ///< distance that constraint condition becomes effective
		double         xi;             ///< xi parameter
		cnoid::Link*   obj;            ///< target object link
	};

	class ArmShape : public EllipsoidShape {
	public:
		explicit ArmShape(ArmPtr _arm);
		virtual ~ArmShape();

		static ArmShapeVec readYamlFile(cnoid::BodyPtr body, ArmPtr arm_ptr);

		cnoid::Vector3 getPos() const;
		void           setCenter(const cnoid::Vector3& _offset);
		cnoid::Vector3 getCenter() const {return offset;}
		int            getJointNum() const {return joint_num;}
		int            getJointID() const {return arm->arm_path->joint(joint_num)->jointId();}
		void           setJointNum(int _joint_num) {joint_num = _joint_num;}

		void updatePos();

	protected:
		cnoid::Vector3 offset;     ///< center position
		int            joint_num;  ///< corresponding joint number
		ArmPtr         arm;        ///< target ArmPtr
	};

	class AbstractEllipsoidDistCalculator {
	public:
		AbstractEllipsoidDistCalculator();
		virtual ~AbstractEllipsoidDistCalculator();

		virtual double calcDistance(const EllipsoidShape* e1, const EllipsoidShape* e2, cnoid::Vector3& p1, cnoid::Vector3& p2) const = 0;
	};

	class ApproximateEllipsoidDistCalculator : public AbstractEllipsoidDistCalculator {
	public:
		ApproximateEllipsoidDistCalculator();
		virtual ~ApproximateEllipsoidDistCalculator();

		double calcDistance(const EllipsoidShape* e1, const EllipsoidShape* e2, cnoid::Vector3& p1, cnoid::Vector3& p2) const;
	};

	class SteepestDescentEllipsoidDistCalculator : public AbstractEllipsoidDistCalculator {
	public:
		SteepestDescentEllipsoidDistCalculator();
		virtual ~SteepestDescentEllipsoidDistCalculator();

		double calcDistance(const EllipsoidShape* e1, const EllipsoidShape* e2, cnoid::Vector3& p1, cnoid::Vector3& p2) const;

	private:
		void getInitSol(const EllipsoidShape* shp1, const EllipsoidShape* shp2, double& theta, double& phi) const;
		void calcDelta(const EllipsoidShape* shp1, const EllipsoidShape* shp2, const cnoid::VectorXd& x, cnoid::VectorXd& delta) const;
		double getDistance(const EllipsoidShape* shp1, const EllipsoidShape* shp2, const cnoid::VectorXd& x) const;
		cnoid::Vector3 getSurfacePoint(const EllipsoidShape* shp, double theta, double phi) const;
	};
}

#endif /* _CONSTRAINTIK_OBSTACLESHAPE_H_ */
