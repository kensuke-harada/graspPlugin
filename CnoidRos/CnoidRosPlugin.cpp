#include <cnoid/Plugin>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>

#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <sys/mman.h>
#include "../Grasp/GraspController.h"

using namespace boost;
using namespace cnoid;

#define RIGHT_ARM_INDEX 0
#define BUFFER_SIZE 1024*1024

namespace grasp {

	class CommandStrGenerator {
		public:
		CommandStrGenerator() {;}
		virtual ~CommandStrGenerator() {;}

		void appendJointCommand(int size, const VectorXd& angles, double time) {
			ss_ << "joints[";
			ss_ << angles[0];
			for (int i = 1; i < size; i++) {
				ss_ << ',' << angles[i];
			}
			ss_ << "]:" << time << ";";
		}

		void appendRightHandCommand(int gstate, int last_gstate) {
			if (gstate == PlanBase::GRASPING || gstate == PlanBase::UNDER_GRASPING) {
				ss_ << "rhandClose;";
			} else if( gstate == PlanBase::NOT_GRASPING) {
				ss_ << "rhandOpen;";
			}
		}

		void appendLeftHandCommand(int gstate, int last_gstate) {
			if (gstate == PlanBase::GRASPING) {
				ss_ << "lhandClose;";
			} else if (gstate == PlanBase::UNDER_GRASPING && last_gstate == PlanBase::NOT_GRASPING) {
				ss_ << "lhandClose;";
			} else if (gstate == PlanBase::UNDER_GRASPING && last_gstate == PlanBase::GRASPING) {
				ss_ << "lhandOpen;";
			} else if (gstate == PlanBase::NOT_GRASPING) {
				ss_ << "lhandOpen;";
			}
		}

		void appendTerminator() {
			ss_ << '\n';
		}

		std::string getCommandStr() const {
			return ss_.str();
		}

		private:
		std::stringstream ss_;
	};

class CnoidRosPlugin : public Plugin
{
public:
    CnoidRosPlugin() : Plugin("Cnoid-ROS")
    {
        require("Grasp");
    }
    
    virtual bool initialize()
    {
        ToolBar* bar = new ToolBar("Cnoid-ROS");
        {
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	    bar->setVisibleByDefault(true);
#endif
            bar->addLabel(("=Cnoid-ROS="));
            bar->addButton("Home", "Robot moves standard position")
                ->sigClicked().connect(bind(&CnoidRosPlugin::onHomeClicked, this));
            bar->addButton("OffPose", "Robot moves offpose position")
                ->sigClicked().connect(bind(&CnoidRosPlugin::onOffPoseClicked, this));
            bar->addButton("Move", "Robot moves as setted motion")
                ->sigClicked().connect(bind(&CnoidRosPlugin::onMoveClicked, this));
        }
        addToolBar(bar);

        /* Initialize mapped file */
        initMappedFile();

        return true;
    }

    virtual ~CnoidRosPlugin()
    {
        closeMappedFile();
    }

private:

    /* MMap fileID */
    int fileId;

    /* Mapped pointer */
    char* mappedPtr;

    void onHomeClicked()
    {
        strcpy(mappedPtr, "goInitial()\n");
    }
    void onMoveClicked()
    {
        //strcpy(mappedPtr, getTestString().c_str());
        strcpy(mappedPtr, getMovePlanString().c_str());
    }
    void onOffPoseClicked()
    {
        strcpy(mappedPtr, "goOff()\n");
    }
    bool initMappedFile()
    {
        /* File Creation. */
        const char* filepath = strcat(getenv("HOME"), "/.CnoidRosMappedFile");
        fileId = open(filepath, O_RDWR|O_CREAT, 0666);
        if (fileId < 0) {
            MessageView::mainInstance()->putln("Error : initMappedFile(), open");
            return false;
        }

        /* Initialization. */
        long psize = sysconf(_SC_PAGE_SIZE);
        long size = (BUFFER_SIZE / psize + 1) * psize;
        if (lseek(fileId, size, SEEK_SET) < 0) {
            MessageView::mainInstance()->putln("Error : initMappedFile(), lseek");
            return false;
        }
        char c;
        if (read(fileId, &c, sizeof(char)) < 0){
            c = '\0';
        }
        if(write(fileId, &c, sizeof(char)) < 0){
            MessageView::mainInstance()->putln("Error : initMappedFile(), write");
            return false;
        }

        /* Mapping */
        mappedPtr = (char*) mmap(0, size, PROT_READ|PROT_WRITE,MAP_SHARED, fileId, 0);
        if (mappedPtr == NULL) {
            MessageView::mainInstance()->putln("Error : initMappedFile(), mmap");
            return false;
        }

        return true;
    }

    void closeMappedFile()
    {
        /* Unmap */
        munmap(mappedPtr, BUFFER_SIZE);
        /* file close */
        close(fileId);
    }

    bool isSingleArm() { return PlanBase::instance()->armsList.size() == 1; }

    std::string getMovePlanString()
    {
        PlanBase* plan = PlanBase::instance();

        int planSize = plan->graspMotionSeq.size();
        if (planSize == 0) {
            MessageView::mainInstance()->putln("PlanSize = 0");
            return "\n";
        }

        int angleSize = plan->graspMotionSeq[0].jointSeq.size();

        int lastState = plan->NOT_GRASPING;
        int lastState2 = plan->NOT_GRASPING;

				CommandStrGenerator command;

        for(int i = 0; i < planSize; i++){

            MotionState* state = &plan->graspMotionSeq[i];
            VectorXd& angles = state->jointSeq;
            double time = state->motionTime;

						command.appendJointCommand(angleSize, angles, time);

						int gstate;
						int gstate2;
						if (plan->targetArmFinger == plan->armsList[RIGHT_ARM_INDEX]) {
							gstate = state->graspingState;
							gstate2 = state->graspingState2;
						} else {
							gstate = state->graspingState2;
							gstate2 = state->graspingState;
						}


						if (isSingleArm()) {
							command.appendRightHandCommand(gstate, lastState);
						} else {
							command.appendRightHandCommand(gstate, lastState);
							command.appendLeftHandCommand(gstate2, lastState2);
						}

						lastState = gstate;
						lastState2 = gstate2;
        }
				command.appendTerminator();
				return command.getCommandStr();
    }
    
    std::string getTestString()
    {
	std::stringstream sstream;
        sstream << "rarm[1.4377544509919726,-1.3161643133168621,-2.126307271452489,1.4335761224859305,0.02359653211486051,0.55989121526186,]:2;";
        sstream << "rhandOpen:0;";
        sstream << "rhandClose:0;";
        sstream << "torso[1.0,]:2;";
        sstream << "torso[0.0,]:2;";
        sstream << "head[1,1,]:1;";
        sstream << "head[0,0,]:1;";
        sstream << '\n';
        return sstream.str();
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(CnoidRosPlugin)

}


