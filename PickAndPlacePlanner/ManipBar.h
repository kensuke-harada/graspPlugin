/**
   c) Kensuke Harada (AIST)
*/

#ifndef MANIPBAR_H
#define MANIPBAR_H

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/
#include "exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

#include <cnoid/MainWindow>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <QDialog>
#include "RobotLocalFunctions.h"

#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif

//#define EXPERIMENT_DATA_OUTPUT

using namespace cnoid;

namespace cnoid {
    class MessageView;
}


namespace grasp {

    namespace PickAndPlacePlanner {

			class StartDialog : public QDialog
			{
			public:
				cnoid::RadioButton rb_default,rb_right, rb_left, rb_cylinder, rb_bothhand, rb_recog;
				QCheckBox* cb_sameDirection;
				QLabel* l_keyposes;
				cnoid::SpinBox sb_keyposes;
#ifdef EXPERIMENT_DATA_OUTPUT
				cnoid::SpinBox sb_targetobject;
#endif
				StartDialog();
				void okClicked();
				void rbBothhandToggled();
			};

			class DebugDialog : public QDialog
			{
			public:
				cnoid::RadioButton rb_default,rb_right, rb_left;
				DebugDialog();
				void okClicked();
			};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API ManipBar : public cnoid::ToolBar, public boost::signals::trackable
#else
        class EXCADE_API ManipBar : public cnoid::ToolBar
#endif
        {
          public:

            static ManipBar* instance();

            virtual ~ManipBar();
	    /*
            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
                return sigBodyItemSelectionChanged_;
            }

            SignalProxy< boost::signal<void(BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
                return sigCurrentBodyItemChanged_;
            }

            const ItemList<BodyItem>& selectedBodyItems() {
                return selectedBodyItems_;
            }

            BodyItem* currentBodyItem() {
                return currentBodyItem_.get();
            }

            bool makeSingleSelection(BodyItemPtr bodyItem);
	    */
          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            ManipBar();

            MessageView& mes;
	    std::ostream& os;

	    //BodyItemPtr currentBodyItem_;
            //ItemList<BodyItem> selectedBodyItems_;
            //ItemList<BodyItem> targetBodyItems;

            //boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#else
            cnoid::Signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            cnoid::Signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif

            //void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
            //void onBodyItemDetachedFro            void onFirstButtonClicked();mRoot();
            void onAllButtonClicked();
            void onFirstButtonClicked();
            void onSecondButtonClicked();
	    void onThirdButtonClicked();
	    void onFourButtonClicked();


            void onStartButtonClicked();
            //void onStopButtonClicked();
	    //void onTrajectoryPlanButtonClicked();
            //void onArmButtonClicked();
            void onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy);
            void onZmpCmButtonClicked();
            //void setZmp(BodyItem::ZmpPosition position);

						void onDebugButtonClicked();
						void onSweptVolumeButtonClicked();

						void onLearnClicked();
        };
    }
}

#endif
