/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef MPK_ROBOTS_H
#define MPK_ROBOTS_H

#include "mpkConfig.h"

#include <cnoid/Body>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Link>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/WorldItem>	/* modified by qtconv.rb 0th rule*/

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ModelNodeSet>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ColdetLinkPair>	/* modified by qtconv.rb 0th rule*/  
#ifdef CNOID_ENABLE_OSG
  #include <cnoid/OSGSceneBody>
  #include <cnoid/OSGSceneBodyManager>
#endif
#else
#include "../Grasp/ColdetLinkPair.h"
//#include <cnoid/SceneBody>
//#include <cnoid/SceneBodyManager>
#endif


#include "exportdef.h"
#include "../Grasp/RobotBody.h"

using namespace std;
//using namespace cnoid;

class mpkRobots{
       public:
               mpkRobots(cnoid::BodyItemPtr robots);
							 mpkRobots(grasp::RobotBodyPtr robotbody);
               ~mpkRobots();

                cnoid::BodyItemPtr robots;
								grasp::RobotBodyPtr robotbodys;
	//cnoid::JointPathPtr arm_path;
                int nJoints;
		bool map;

                void get_config(mpkConfig &q);
                void set_config(mpkConfig &q);
                void compute_forward_kinematics();

                struct param_info {
		  param_info() {
		    default_val = 0.5; is_cyclic = false; is_passive = false;
		    is_frozen = false; weight = 1;
		  };
		  double default_val; // default parameter value (usually 0.5)
		  bool is_cyclic;     // cyclic parameter (currently not used)
		  bool is_passive;    // passive parameters should not be sampled by planner
		  bool is_frozen;     // to temporarily make non-passive parameters passive
		  double weight;      // weighting parameters is useful for animation etc.
		};
  
                param_info *param_opts;

};
	
#endif
