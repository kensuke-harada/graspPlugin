/**
   @author Takashi Kitagawa (AIST)
*/

#ifndef _MOTION_FILE_CONTROL_H_INCLUDED
#define _MOTION_FILE_CONTROL_H_INCLUDED

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/GraspController.h"
#include "../PRM/TrajectoryPlanner.h"
#include "../../graspPlugin/Grasp/exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

using namespace cnoid;

	namespace grasp {

		//class EXCADE_API MotionFileControl : public cnoid::ToolBar, public boost::signals::trackable
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		class EXCADE_API MotionFileControl : public boost::signals::trackable
#else
		class EXCADE_API MotionFileControl
#endif
		{
			public:

			static MotionFileControl* instance();

			virtual ~MotionFileControl();

			std::string objectBasePath;

			protected:

			private:

			MotionFileControl();

			MessageView& mes;
			std::ostream& os;

			public:

			Vector3		InitBasePos, InitBaseRPY;
			Vector3		InitObjPos, InitObjRPY;
			Vector3		StartObjPos, StartObjRPY;
			VectorXd	InitJnt;
			MotionState	PauseState;

			MotionState tmpState;

			int			init;
			int			pause;
			int			eof;
			int			CommandCnt;
			int			MotionCnt;
			int			ArmType;

			bool		headForward;

			void loadObjectData( void );
			void LoadFromMotionFile( std::string motionfilename );
			void ClearMotionFile( void );
			void SavePositionToDB( void );

			private:

			bool JointAbs( double Ts, double Te, VectorXd Value );
			bool JointRel( double Ts, double Te, VectorXd Value );
			bool ArmXYZAbs( int LR_flag, double Ts, double Te, VectorXd Value );
			bool ArmXYZRel( int LR_flag, double Ts, double Te, VectorXd Value );
			bool ArmJntAbs( int LR_flag, double Ts, double Te, VectorXd Value );
			bool ArmJntRel( int LR_flag, double Ts, double Te, VectorXd Value );
			bool HandJntAbs( int LR_flag, double Ts, double Te, VectorXd Value , int state);
			bool HandJntRel( int LR_flag, double Ts, double Te, VectorXd Value , int state);
			bool ArmObject( int LR_flag, double Ts, double Te, VectorXd Value );
			bool HandJntOpen( int LR_flag, double Ts, double Te );
			bool HandJntClose( int LR_flag, double Ts, double Te, VectorXd Value );
			bool HandGrasp( int LR_flag, double Ts, double Te);
			bool LinearAbs( int LR_flag, double Ts, double Te, VectorXd Value );
			bool LinearRel( int LR_flag, double Ts, double Te, VectorXd Value );
			bool setObject( int id);

			void TrackObjectPose( void );
			void setMotionState(MotionState gm, MotionState gm_prev);
			bool HeadForwardFunc( void );
		};
   }

#endif

