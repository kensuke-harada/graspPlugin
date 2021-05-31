/**
   c) Tokuo Tsuji (Kyushu univ./AIST)
*/

#include "VisionRecognitionTrigger.h"
#include "VisionTriggerRtcController.h"

#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/


using namespace std;
using namespace cnoid;
using namespace grasp;

VisionTriggerRtcController* VisionTriggerRtcController::instance()
{
	static VisionTriggerRtcController* instance = new VisionTriggerRtcController();
	return instance;
}


// MyModuleInit is common name
void VisionTriggerRtcController::VisionTriggerMyModuleInit(RTC::Manager* manager)
{

 VisionRecognitionTriggerInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = (VisionRecognitionTrigger *)manager->createComponent("VisionRecognitionTrigger");

  VisionTriggerRtcController::instance()->comp_ = (VisionRecognitionTrigger *)comp;

  if (comp==NULL)
  {
    std::cerr << "Component create failed." << std::endl;
    abort();
  }

  return;
}

bool VisionTriggerRtcController::VisionRecoginitionStart(){

	stopFlag=false;

	if( !PlanBase::instance()->initial()){
		return false;
	}

	CORBA::ULong ObjId=0;

	string obj_data   = PlanBase::instance()->bodyItemRobotPath() + string("/data/obj_list.txt");
	ifstream fin_obj(obj_data.c_str());
	string objname =  PlanBase::instance()->targetObject->bodyItemObject->name();

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

	RTC::CorbaConsumer<RecognitionService>* m_recogTrigger = VisionTriggerRtcController::instance()->comp_->m_recogTrigger_();
	if (::CORBA::is_nil(m_recogTrigger->getObject()) == false) {
		(*m_recogTrigger)->setModelID(ObjId);
	}
	else{
		cout << "RTC not connected" << endl;
		return false;
	}

//	while( VisionTriggerRtcController::instance()->comp_->results.size() < 20){
//		MessageView::mainInstance()->flush();
//		if(stopFlag) return false;
//	}

	RTC::InPort<RTC::TimedDoubleSeq>* m_recogResultIn = &VisionTriggerRtcController::instance()->comp_->m_recogResultIn;



	while ( !m_recogResultIn->isNew() ) {
		MessageView::mainInstance()->flush();
		if(stopFlag) return false;
	}
	m_recogResultIn->read();

  	TimedRecognitionResult* m_recogResult =  &VisionTriggerRtcController::instance()->comp_->m_recogResult;


   /* 		m_recogResultIn.read();
    printf("length: %ld\n", m_recogResult.data.length());

	  results.clear();
	  for(unsigned int i=0; i<  m_recogResult.data.length(); i++){
		  results.push_back(m_recogResult.data[i]);
	  }

    for (size_t i = 0; i < m_recogResult.data.length()/4; ++i) {
      for (size_t j = 0; 4*i+j < m_recogResult.data.length() && j < 4; ++j) {
	printf("% 7.4f ", m_recogResult.data[4*i+j]);
      }
      printf("\n");

      if (i % 5 == 4) {
	printf("\n");
      }
    }
    printf("\n");
*/
 // }
	if( m_recogResult->data.length()==0 ){
		MessageView::mainInstance()->cout() << "Vision Error: No data" << endl;
		return false;
	}
	if( m_recogResult->data[5] ){
		MessageView::mainInstance()->cout() << "Vision Error: Return Error Code" << endl;
		return false;
	}


	double* result =  &m_recogResult->data[0];
//	int size = VisionTriggerRtcController::instance()->comp_->results.size();

	Vector3 objPos ( result[11]/1000.0,result[15]/1000.0,result[19]/1000.0);
	Matrix3 objOri;
	objOri << result[8],result[9],result[10],result[12],result[13],result[14],result[16],result[17],result[18];

	PlanBase::instance()->setObjPos(objPos, objOri);
	PlanBase::instance()->flush();

	VisionTriggerRtcController::instance()->comp_->results.clear();

	return true;
}

bool VisionTriggerRtcController::switchExtModeFlag() {
	comp_->ext_mode_flag = !(comp_->ext_mode_flag);
	return comp_->ext_mode_flag;
}

int VisionTriggerRtcController::RtcStart()
{
	int argc=1;
	char *argv[] = {(char *)("VisionRecognitionTriggerComp")};

  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(VisionTriggerMyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
//  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  manager->runManager(true);

  return 0;
}
