/**
   @author Shin'ichiro Nakaoka
*/

#include "RobotInterfaceBar.h"
//#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <omniORB4/CORBA.h>
#include "../Grasp/GraspController.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

RobotInterfaceBar* RobotInterfaceBar::instance()
{
	static RobotInterfaceBar* instance = new RobotInterfaceBar();
	return instance;
}

RobotInterfaceBar::RobotInterfaceBar()
	: ToolBar("RobotInterfaceBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{

	addSeparator();

	addLabel(("=Interface="));

	addButton(("CapImage"), ("Capture image from camera"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onCaptureButtonClicked, this));

	//addButton(_("ReadFile"), _("Read from File"))->
	//	sigClicked().connect(bind(&RobotInterfaceBar::onReadFromFileButtonClicked, this));

	addButton(("Recog"), ("Recoginition and read from file"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onRecognitionButtonClicked, this));

	addButton(("Init-Calb"), ("Setup and Calib"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onJointCalibButtonClicked, this));

	addButton(("SrvOn"), ("Servo On"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onSrvOnButtonClicked, this));

	addButton(("SrvOff"), ("Servo Off"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onSrvOffButtonClicked, this));

	addButton(("Home"), ("Robot moves standard position"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onHomeButtonClicked, this));

	addButton(("Move"), ("Robot moves as setted motion"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onMoveButtonClicked, this));

    addButton(("Set"), ("Set the robot to the same configuration as the view"))->
        sigClicked().connect(bind(&RobotInterfaceBar::onSetButtonClicked, this));

    addButton(("OffPose"), ("Robot moves offpose position"))->
        sigClicked().connect(bind(&RobotInterfaceBar::onOffPoseButtonClicked, this));

	addButton(("MultiMode"), ("Toggle Multi Mode"))->
		sigClicked().connect(bind(&RobotInterfaceBar::onMultiButtonClicked, this));

	addSeparator();
}


RobotInterfaceBar::~RobotInterfaceBar()
{
}

void RobotInterfaceBar::onReadFromFileButtonClicked()
{
//	if(GraspController::instance()->bodyItemRobot==NULL) return;
	if(PlanBase::instance()->targetObject==NULL){
		os << "Please set object" << endl;
		return;
	}

	//RobotInterface_ = new RobotInterface();

	RobotInterface::instance()->doReadFromFile();

	os <<  "Read from File" << endl;
}


void RobotInterfaceBar::onRecognitionButtonClicked()
{
//	if(GraspController::instance()->bodyItemRobot==NULL) return;
	if(PlanBase::instance()->targetObject==NULL){
		os << "Please set object" << endl;
		return;
	}

	//RobotInterface_ = new RobotInterface();

	RobotInterface::instance()->doRecoginitionAndRead();

	os <<  "Recog and Read " << endl;
}

void RobotInterfaceBar::onMultiButtonClicked(){
	RobotInterface::instance()->isMulti = !RobotInterface::instance()->isMulti;
	if( RobotInterface::instance()->isMulti ) os <<"Multi Recognition Mode" << endl;
	else os <<"Single Recognition Mode" << endl;
}


void RobotInterfaceBar::onCaptureButtonClicked(){
	RobotInterface::instance()->doCapture();
}

void RobotInterfaceBar::onJointCalibButtonClicked(){
	try {
		RobotInterface::instance()->doJntCalib();
	} catch (CORBA::INV_OBJREF e) {
		showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
	} catch (CORBA::OBJECT_NOT_EXIST e) {
		showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
	} catch (CORBA::NO_IMPLEMENT e) {
		showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
	} catch (CORBA::UNKNOWN e) {
		showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
	} catch (CORBA::SystemException& e) {
		char msg[1024];
		sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
		showWarningDialog(msg);
	} catch (CORBA::Exception& e) {
		char msg[1024];
		sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
		showWarningDialog(msg);
	}
}

void RobotInterfaceBar::onSrvOnButtonClicked(){
	try {
		RobotInterface::instance()->doSrvOn();
	} catch (CORBA::INV_OBJREF e) {
		showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
	} catch (CORBA::OBJECT_NOT_EXIST e) {
		showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
	} catch (CORBA::NO_IMPLEMENT e) {
		showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
	} catch (CORBA::UNKNOWN e) {
		showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
	} catch (CORBA::Exception& e) {
		char msg[1024];
		sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
		showWarningDialog(msg);
	}
}

void RobotInterfaceBar::onSrvOffButtonClicked(){
	try {
		RobotInterface::instance()->doSrvOff();
	} catch (CORBA::INV_OBJREF e) {
		showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
	} catch (CORBA::OBJECT_NOT_EXIST e) {
		showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
	} catch (CORBA::NO_IMPLEMENT e) {
		showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
	} catch (CORBA::UNKNOWN e) {
		showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
	} catch (CORBA::SystemException& e) {
		char msg[1024];
		sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
		showWarningDialog(msg);
	} catch (CORBA::Exception& e) {
		char msg[1024];
		sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
		showWarningDialog(msg);
	}
}

void RobotInterfaceBar::onHomeButtonClicked(){
	try {
		RobotInterface::instance()->doHome();
	} catch (CORBA::INV_OBJREF e) {
		showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
	} catch (CORBA::OBJECT_NOT_EXIST e) {
		showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
	} catch (CORBA::NO_IMPLEMENT e) {
		showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
	} catch (CORBA::UNKNOWN e) {
		showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
	} catch (CORBA::SystemException& e) {
		char msg[1024];
		sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
		showWarningDialog(msg);
	} catch (CORBA::Exception& e) {
		char msg[1024];
		sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
		showWarningDialog(msg);
	}
}

void RobotInterfaceBar::onMoveButtonClicked(){
	try {
		RobotInterface::instance()->doMove();
	} catch (CORBA::INV_OBJREF e) {
		showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
	} catch (CORBA::OBJECT_NOT_EXIST e) {
		showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
	} catch (CORBA::NO_IMPLEMENT e) {
		showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
	} catch (CORBA::UNKNOWN e) {
		showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
	} catch (CORBA::SystemException& e) {
		char msg[1024];
		sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
		showWarningDialog(msg);
	} catch (CORBA::Exception& e) {
		char msg[1024];
		sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
		showWarningDialog(msg);
	}
}

void RobotInterfaceBar::onSetButtonClicked()
{
    try {
        RobotInterface::instance()->doSet();
    } catch (CORBA::INV_OBJREF e) {
        showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
    } catch (CORBA::OBJECT_NOT_EXIST e) {
        showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
    } catch (CORBA::NO_IMPLEMENT e) {
        showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
    } catch (CORBA::UNKNOWN e) {
        showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
    } catch (CORBA::SystemException& e) {
        char msg[1024];
        sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
        showWarningDialog(msg);
    } catch (CORBA::Exception& e) {
        char msg[1024];
        sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
        showWarningDialog(msg);
    }
}

void RobotInterfaceBar::onOffPoseButtonClicked()
{
    try {
        RobotInterface::instance()->doOffPose();
    } catch (CORBA::INV_OBJREF e) {
        showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
    } catch (CORBA::OBJECT_NOT_EXIST e) {
        showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
    } catch (CORBA::NO_IMPLEMENT e) {
        showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
    } catch (CORBA::UNKNOWN e) {
        showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
    } catch (CORBA::SystemException& e) {
        char msg[1024];
        sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
        showWarningDialog(msg);
    } catch (CORBA::Exception& e) {
        char msg[1024];
        sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
        showWarningDialog(msg);
    }
}

bool RobotInterfaceBar::storeState(Archive& archive)
{
	return true;
}

bool RobotInterfaceBar::restoreState(const Archive& archive)
{
	return true;
}
