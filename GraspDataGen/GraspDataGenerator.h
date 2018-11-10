/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _GraspDataGenerator_H
#define _GraspDataGenerator_H

#include <stdlib.h>
#include <time.h>

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/PlanBase.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/VectorMath.h"
#include "../GeometryHandler/GeometryHandle.h"
#include "../SoftFingerStability/SoftFingerStabilityHandler.h"
#include <fstream>

#include "exportdef.h"

namespace grasp{

	class EXCADE_API GraspDataGenerator : public grasp::GraspController
	{

	public :
		GraspDataGenerator();
		~GraspDataGenerator();

		static GraspDataGenerator* instance(GraspDataGenerator *gc=NULL);

		enum GRASP_TYPE{T_SINGLE,T_LIFTUP,T_SIDEDUAL};
		enum ROTATION_TYPE{R_OCP,R_STABLE};
		enum CLUSTERING_TYPE{C_BOUNDINGBOX,C_CYLINDER};

		class DataPosturePos{
		public:
			cnoid::Vector3 p;
			cnoid::Matrix3 R;
			std::vector<double> hand_angle;
			double q;
			std::vector<cnoid::Vector3> contact_pos;
			std::vector<cnoid::Vector3> contact_N;
			int cluster_num;

			std::vector<double> en;
			std::vector<double> area;
			std::vector<cnoid::Vector3> fing_pos;
			std::vector<cnoid::Vector3> fing_N;

			static bool sortPosturePosData(const DataPosturePos& l,const DataPosturePos& r) {return (l.q>r.q);}
		};

		int num_cluster;
		double vol_ratio_threshold;
		double grid_interval;
		double depth_grid_interval;
		double max_edge_len;
		double scale;
		double size_coeff;
		bool is_show_all_cluster;
		double dist_th;
		GRASP_TYPE grasp_type;
		ROTATION_TYPE rotation_type;
		CLUSTERING_TYPE clustering_type;
		ObjectShape* obj_shape;
		std::vector<ObjectShape*> finger_shape;

		bool generateGraspPattern();
		void getBoundingboxParameters(int num_cluster,double vol_ratio_threshold,double scale,std::vector<cnoid::Vector3>& edges, std::vector<cnoid::Vector3>& centers, std::vector<cnoid::Matrix3>& rots);
		void searchGraspPattern(std::vector<cnoid::Vector3>& edges, std::vector<cnoid::Vector3>& centers, std::vector<cnoid::Matrix3>& rots,std::vector<DataPosturePos>& postures);
		void writePreplanning(std::string filename,std::vector<DataPosturePos>& postures);
		void writeContactpoint(std::string filename,std::vector<DataPosturePos>& postures);
		void writeEnvalue(std::string filename,std::vector<DataPosturePos>& postures);
		void generateGraspPostures(int cluster_num,cnoid::Vector3& edge,cnoid::Vector3& center,cnoid::Matrix3& rot,std::vector<DataPosturePos>& postures);
		void generateGraspPosturesStablePut(cnoid::Vector3& edge,cnoid::Vector3& center,cnoid::Matrix3& rot,std::vector<DataPosturePos>& postures);
		void searchGraspPosture(int cluster_num,Homogenous& Palm,std::vector<double*>& link_org_q,std::vector<DataPosturePos>& postures);
		void searchGraspPostureLiftUp(int cluster_num,Homogenous& Palm,const cnoid::Vector3& edge,std::vector<double*>& link_org_q,std::vector<DataPosturePos>& postures);
		void searchGraspPostureFromSide(int cluster_num,Homogenous& Palm,const cnoid::Vector3& edge,std::vector<double*>& link_org_q,std::vector<DataPosturePos>& postures);
		int num_success,num_approach_points,num_search_point;
		int sum_success,sum_approach_points,sum_search_point;
		
		void addClusterBox(std::vector<cnoid::Vector3>& edges,std::vector<cnoid::Vector3>& centers,std::vector<cnoid::Matrix3>& rots);
		void addApproachPoint(std::vector<cnoid::Vector3>& edges,std::vector<cnoid::Vector3>& centers,std::vector<cnoid::Matrix3>& rots,std::vector<DataPosturePos>& postures);
		void addContactPoint(std::vector<DataPosturePos>& postures);
		void addContactPointonBox(std::vector<cnoid::Vector3>& edges,std::vector<cnoid::Vector3>& centers,std::vector<cnoid::Matrix3>& rots,std::vector<DataPosturePos>& postures);
		cnoid::Vector3 getColor(double value);
		void showClusterMesh();
		void showBoundingBox();
		void showCylinder();
		void showContactPoint();
		void showApproachPoint();

		ObjectShape* createObjectShape(cnoid::ColdetModelPtr c);

		enum SearchMode {SCALE_X,SCALE_Y,SCALE_Z,SCALE_XYZ};
		bool searchHandScale(double start_x,double start_y,double start_z,double end,double step,SearchMode mode);
		bool loadScaledTrobot(double scale_x,double scale_y,double scale_z,cnoid::BodyItemPtr& bodyitem);

		void seqGenerateGraspPattern(std::string filename);
		bool grasp(int index);
		bool grasp(int index, bool is_move_arm);
#if 0
		void testGraspSide();
#endif
	protected :

		std::ostream& os;
	};


	class GraspDatabaseManipulator;

	class GraspDataAppender : public grasp::GraspController
	{
	public :
		GraspDataAppender();
		virtual ~GraspDataAppender();

		static GraspDataAppender* instance(GraspDataAppender *gda=NULL);

		enum GRASP_STATE{S_GRASP, S_SUCTION, S_NOTCONTACT, S_NOTTIPCONTACT};

		bool init();
		void calcBoundingBoxCluster(int num_cluster, double vol_ratio_th, double scale);
		void calcBoundingBoxCylinder(int num_cluster, double dist_th);

		virtual void moveInit();
		virtual void move(const cnoid::Vector3& diff_pos, const cnoid::Vector3& diff_rpy);
		void nextFace();
		void nextBB();
		void append();
		void showTargetBB(int bbid);
		double getCurQ() const {return cur_q;}
		bool isCollide() const {return is_collide;}
		GRASP_STATE getState() const {return cur_state;}
	protected:
		std::vector<Box> boundingboxes;
		cnoid::Vector3 cur_pos;
		cnoid::Vector3 cur_rpy;
		int cur_bbid;
		int cur_fid;
		double cur_q;
		GRASP_STATE cur_state;
		bool is_collide;

		void sortBoundingbox();
		double calcStability();

		GraspDatabaseManipulator* gdm;
	};

	class GraspDataUpdater :
		public GraspDataAppender {
	public:
		~GraspDataUpdater();
		static GraspDataUpdater* instance();

		virtual void moveInit(int gid);
		virtual void move(const cnoid::Vector3& diff_pos, const cnoid::Vector3& diff_rpy);
		void update();
		void updateDataBase();

	private:
		GraspDataUpdater();

		cnoid::Vector3 ori_pos_;
		cnoid::Matrix3 ori_R_;
		int gid_;
	};

	class GraspControllerWithInterface : public grasp::GraspController {
	public:
		static GraspControllerWithInterface* instance(GraspControllerWithInterface *gc=NULL);
	
		void settingPrehensionParam(const PrehensionParameter& param) {settingPrehension(param);}
		bool doGrasping(const std::vector<std::vector<double> >& fing_angles);
	};
}

#endif
