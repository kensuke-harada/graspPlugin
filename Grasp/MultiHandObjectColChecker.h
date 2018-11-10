#ifndef _GRASP_MULTIHANDOBJECTCOLCHECKER_H_
#define _GRASP_MULTIHANDOBJECTCOLCHECKER_H_

#include <vector>
#include <map>
#include <list>
#include <iostream>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <cnoid/BodyItem>

#include "ColdetLinkPair.h"
#include "ObjectManager.h"
#include "PointCloudEnv.h"

namespace grasp {
	class TargetObject;
	class ColCheckPair;
	class ColCheckPairPointCloud;

	class MultiHandObjectColChecker {
		public:
		MultiHandObjectColChecker();
		virtual ~MultiHandObjectColChecker();

		void setObjectManager(ObjectManager* obj_manager);
		void setTargetObject(TargetObject* object);

		void initialCollisionSelf();
		void initialRoughClearanceRobot();
		void initialCollisionSelf(ObjectBase* object);
		void initialCollision(std::list<cnoid::BodyItemPtr>& env_body_items,
													std::list<PointCloudEnv*>& env_pointcloud);
		void initialCollisionAlwaysRemake(std::list<cnoid::BodyItemPtr>& env_body_items,
																			std::list<PointCloudEnv*>& env_pointcloud);
		bool isColliding() const;
		double clearance(double tolerance) const;

		void clearAll();

	protected:
		ObjectManager* obj_manager_;
		TargetObject* object_;

		std::vector<ColCheckPair*> checkers_;
		std::vector<ColCheckPair*> self_checkers_;
		std::vector<ColCheckPairPointCloud*> point_cloud_checkers_;
		std::vector<ColCheckPair*> checkers_cache_;

		ColCheckPair* findColCheckPair(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2) const;
		ColCheckPair* createColCheckPairEnv(ObjectBase* obj, cnoid::BodyItemPtr& env) const;
		ColCheckPair* createColCheckPair(ObjectBase* obj1, ObjectBase* obj2) const;
		ColCheckPairPointCloud* createColCheckPairPointCloud(ObjectBase* obj, PointCloudEnv* pointcloud) const;
	};

	class ColCheckPair {
	public:
		ColCheckPair();
		virtual ~ColCheckPair();

		virtual bool isColliding(std::ostream& out = std::cout) const;
		virtual double clearance(double tolerance) const;
		virtual bool isSame(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2) const;
		virtual void initialCollision(const std::string& default_debug_msg);

	protected:
		cnoid::BodyItemPtr bodyitem1_;
		cnoid::BodyItemPtr bodyitem2_;
		std::vector<ColdetLinkPairVector> col_link_pairs_vec_;
		std::vector<boost::function<bool (void)> > col_test_check_func_vec_;
		std::vector<std::string> debug_messages_;
		const double min_sep_;
		bool is_env_pair_;

		void addColdetLinkPairs(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2);
		void addColdetLinkPairs(const cnoid::BodyItemPtr& bodyitem1, const std::vector<cnoid::Link*>& target_links,
														const cnoid::BodyItemPtr& bodyitem2);
		void addColdetLinkPairsSafe(const ObjectBase* obj, const cnoid::BodyItemPtr& env);
		bool alwaysCheck() const {return true;}
	};

	class SelfColCheckPair :
		public ColCheckPair {
	public:
		explicit SelfColCheckPair(ObjectBase* obj);
		~SelfColCheckPair();

		virtual bool isColliding(std::ostream& out = std::cout) const;

	private:
		ObjectBase* obj_;
	};

	class RobotEnvColCheckPair :
		public ColCheckPair {
	public:
		RobotEnvColCheckPair(ObjectBase* robot, cnoid::BodyItemPtr& env);
		~RobotEnvColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		RobotBody* robot_;

		bool isUseSafeBoundingBox() const;
		bool isNotUseSafeBoundingBox() const;
	};

	class ObjEnvColCheckPair :
		public ColCheckPair {
	public:
		ObjEnvColCheckPair(ObjectBase* obj, cnoid::BodyItemPtr& env);
		~ObjEnvColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		AssemblyObject* object_;

		bool isUseSafeBoundingBox() const;
		bool isNotUseSafeBoundingBox() const;
	};

	class HandEnvColCheckPair :
		public ColCheckPair {
	public:
		HandEnvColCheckPair(ObjectBase* hand, cnoid::BodyItemPtr& env);
		~HandEnvColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		RobotHand* hand_;

		bool isUseSafeBoundingBox() const;
		bool isNotUseSafeBoundingBox() const;
	};

	class ObjObjColCheckPair :
		public ColCheckPair {
	public:
		ObjObjColCheckPair(ObjectBase* obj1, ObjectBase* obj2);
		~ObjObjColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		AssemblyObject* obj1_;
		AssemblyObject* obj2_;

		bool isCheckCollision() const;
	};

	class HandHandColCheckPair :
		public ColCheckPair {
	public:
		HandHandColCheckPair(ObjectBase* hand1, ObjectBase* hand2);
		~HandHandColCheckPair();

	private:
		RobotHand* hand1_;
		RobotHand* hand2_;
	};

	class HandRobotColCheckPair :
		public ColCheckPair {
	public:
		HandRobotColCheckPair(ObjectBase* hand, ObjectBase* robot);
		~HandRobotColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		RobotHand* hand_;
		RobotBody* robot_;

		bool isCheckCollision(int arm_id) const;
	};

	class ObjRobotColCheckPair :
		public ColCheckPair {
	public:
		ObjRobotColCheckPair(ObjectBase* obj, ObjectBase* robot);
		~ObjRobotColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		AssemblyObject* obj_;
		RobotBody* robot_;

		bool isCheckCollision(int arm_id) const;
	};

	class HandObjColCheckPair :
		public ColCheckPair {
	public:
		HandObjColCheckPair(ObjectBase* obj, ObjectBase* hand);
		~HandObjColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		RobotHand* hand_;
		AssemblyObject* obj_;

		bool isCheckCollision() const;
	};

	class HandTargetObjColCheckPair :
		public ColCheckPair {
	public:
		HandTargetObjColCheckPair(TargetObject* obj, RobotHand* hand);
		~HandTargetObjColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);

	private:
		RobotHand* hand_;
		TargetObject* obj_;

		bool isCheckCollision() const;
	};

	class ColCheckPairPointCloud {
	public:
		ColCheckPairPointCloud();
		virtual	~ColCheckPairPointCloud();

		virtual bool isColliding(double tolerance = 0.001, std::ostream& out = std::cout) const;
		virtual double clearance(double tolerance) const;
		virtual void initialCollision(const std::string& default_debug_msg);
		
	protected:
		cnoid::BodyItemPtr bodyitem_;
		PointCloudEnv* pointcloud_;
		std::vector<std::vector<std::pair<cnoid::Link*, PointCloudEnv*> > > col_link_pairs_vec_;
		std::vector<boost::function<bool (void)> > col_test_check_func_vec_;
		std::vector<std::string> debug_messages_;
		const double min_sep_;

		bool alwaysCheck() const {return true;}
	};

	class RobotPointCloudColCheckPair :
		public ColCheckPairPointCloud {
	public:
		RobotPointCloudColCheckPair(ObjectBase* obj, PointCloudEnv* pointcloud);
		~RobotPointCloudColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);
		
	private:
		RobotBody* robot_;

		bool isCheckFingerPointCloudPair();
	};

	class HandPointCloudColCheckPair :
		public ColCheckPairPointCloud {
	public:
		HandPointCloudColCheckPair(ObjectBase* obj, PointCloudEnv* pointcloud);
		~HandPointCloudColCheckPair();

		virtual void initialCollision(const std::string& default_debug_msg);
	private:
		bool isCheckFingerPointCloudPair();
	};

	class ObjPointCloudColCheckPair :
		public ColCheckPairPointCloud {
	public:
		ObjPointCloudColCheckPair(ObjectBase* obj, PointCloudEnv* pointcloud);
		~ObjPointCloudColCheckPair();
	};
	
}

#endif
