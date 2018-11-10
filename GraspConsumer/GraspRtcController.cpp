#include "GraspConsumer.h"
#include "GraspRtcController.h"

#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  


using namespace grasp;
using namespace std;
using namespace cnoid;

GraspRtcController* GraspRtcController::instance()
{
	static GraspRtcController* instance = new GraspRtcController();
	return instance;
}

int GraspRtcController::RtcStart()
{
	int argc=1;
	char *argv[] = {(char *)("GraspConsumerComp")};
  
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  //  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  manager->runManager(true);

  return 0;
}


void GraspRtcController::MyModuleInit(RTC::Manager* manager)
{
	
  GraspConsumerInit(manager);

  // Create a component
  instance()->comp_ = (GraspConsumer *)manager->createComponent("GraspConsumer");;

  if (instance()->comp_ ==NULL)
  {
    std::cerr << "Component create failed." << std::endl;
    abort();
  }

  return;
}

bool GraspRtcController::graspPlanStart(int mode){
	
	PlanBase* gc = PlanBase::instance();

	if( !gc->initial()){
		return false;
	}
	

	Vector3 po = gc->objVisPos();
	Matrix3 Ro = gc->objVisRot();
	
	basePos = gc->base()->p(); //smartPal‚àOK
	baseOri = gc->base()->R();
	
	
	Matrix3 oRb (baseOri.transpose()*Ro);
	Vector3  opb ( baseOri.transpose()*(po-basePos) )  ;
	

	CORBA::ULong ObjId=0;
	GraspPlanStart::DblSequence3 objPos;
	GraspPlanStart::DblSequence9 objOri;
	CORBA::ULong state;
	
	string obj_data   = gc->bodyItemRobotPath() + string("/data/obj_list.txt");
	ifstream fin_obj(obj_data.c_str());
	string objname =  gc->targetObject->name();

	if(!fin_obj){
		cout << "obj_list.txt not found" << obj_data<< endl;
		return false;
	}
	
	while(fin_obj){
		string tmp_obj;
		stringstream li2;
		string line2;
		int qtmp;


		getline(fin_obj,line2);
		li2 << line2;

		li2 >> qtmp;
		li2 >> tmp_obj;
		
		if(tmp_obj.find(objname,0) != string::npos){
			ObjId = qtmp;	
			cout << qtmp << tmp_obj << endl;
			break;
		}
	}
	
	objPos.length(3);
	objOri.length(9);
	

	for(unsigned int i=0; i<3; i++){
		objPos[i] = opb[i];
	    for(unsigned int j=0; j<3; j++){
		    objOri[3*i+j] = oRb(i,j);
	    }
	}
	
	if(mode) comp_->m_PlanStart->GraspPlanningStart(ObjId, objPos,objOri,state);
	else comp_->m_PlanStart->ReleasePlanningStart(ObjId, objPos,objOri,state);
	
	return true;
}


bool GraspRtcController::graspPlanResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, const GraspPlanResult::DblSequence& angle){
	
	PlanBase* gc = PlanBase::instance();
	
	Vector3 graspPos_(GraspPos[0],GraspPos[1],GraspPos[2]);
	Matrix3 graspOri_;
	graspOri_ << GraspOri[0],GraspOri[1],GraspOri[2],GraspOri[3],GraspOri[4],GraspOri[5],GraspOri[6],GraspOri[7],GraspOri[8];

	Vector3 approachPos_(ApproachPos[0],ApproachPos[1],ApproachPos[2]);
	Matrix3 approachOri_;
	approachOri_ << ApproachOri[0],ApproachOri[1],ApproachOri[2],ApproachOri[3],ApproachOri[4],ApproachOri[5],ApproachOri[6],ApproachOri[7],ApproachOri[8];

	
	cout << graspPos_ << graspOri_ << basePos << baseOri <<endl;
	
	
	graspPos_ = Vector3(baseOri*graspPos_ + basePos);
	graspOri_ = Matrix3(baseOri*graspOri_);
	
	approachPos_ = Vector3(baseOri*approachPos_ + basePos);
	approachOri_ = Matrix3(baseOri*approachOri_);
	
	
	bool ikConv = gc->arm()->IK_arm(approachPos_, approachOri_);
	bool limitCheck = gc->arm()->checkArmLimit();
	int n_fing = gc->nFing();
	std::vector<int> fing_n_joint(n_fing);
	for (int i = 0; i < n_fing; i++) {
		fing_n_joint[i] = gc->fingers(i)->nJoints;
	}
	int k = 0;
	for (int i = 0; i < n_fing; i++) {
		for (int j = 0; j < fing_n_joint[i]; j++) {
			gc->fingers(i)->fing_path->joint(j)->q() = angle[k++];
		}
	}
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->flush();

	if(ikConv && limitCheck){
		
	}else{
		return false;
	}

	ikConv = gc->arm()->IK_arm(graspPos_, graspOri_);
	limitCheck = gc->arm()->checkArmLimit();
	k = 0;
	for (int i = 0; i < n_fing; i++) {
		for (int j = 0; j < fing_n_joint[i]; j++) {
			gc->fingers(i)->fing_path->joint(j)->q() = angle[k++];
		}
	}
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->flush();

	if(ikConv && limitCheck){
		return true;
	}else{
		return false;
	}
	
}
