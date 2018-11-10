/**
   @author Takashi Kitagawa (AIST)
*/

#ifndef _MOTION_FILE_BAR_H_INCLUDED
#define _MOTION_FILE_BAR_H_INCLUDED

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../PRM/TrajectoryPlanner.h"
#include "../../graspPlugin/Grasp/exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#endif

using namespace cnoid;

	namespace grasp {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		class EXCADE_API MotionFileBar : public cnoid::ToolBar, public boost::signals::trackable
#else
		class EXCADE_API MotionFileBar : public cnoid::ToolBar
#endif
		{
			public:

			static MotionFileBar* instance();

			virtual ~MotionFileBar();

			std::string objectBasePath;

			protected:

			private:

			MotionFileBar();

			MessageView& mes;
			std::ostream& os;

			void onLoadButtonClicked();
			void onClearButtonClicked();
			void onSavePositionButtonClicked();
        };
   }

#endif

