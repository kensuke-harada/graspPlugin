/**
   @author Shin'ichiro Nakaoka
*/

#ifndef EXCADE_ROBOTICS_VVV_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_VVV_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

#include "RobotInterface.h"

using namespace cnoid;

    
    namespace grasp {

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API RobotInterfaceBar : public cnoid::ToolBar, public boost::signals::trackable
#else
        class EXCADE_API RobotInterfaceBar : public cnoid::ToolBar
#endif
        {
          public:

            static RobotInterfaceBar* instance();
            virtual ~RobotInterfaceBar();


          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

	    RobotInterfaceBar();
	    
	    MessageView& mes;
	    std::ostream& os;
	    //   VVVInterface* vvvInterface_;
	    
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	    boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
	    boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#else
	    Signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
	    Signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif

	    void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
	    void onBodyItemDetachedFromRoot();
	    void onReadFromFileButtonClicked();
	    void onRecognitionButtonClicked();
	    void onMultiButtonClicked();
	    
	    void onCaptureButtonClicked();
	    void onJointCalibButtonClicked();
	    void onSrvOnButtonClicked();
	    void onSrvOffButtonClicked();
	    void onHomeButtonClicked();
	    void onMoveButtonClicked();
		void onSetButtonClicked();
		void onOffPoseButtonClicked();
		  };
    }

#endif
