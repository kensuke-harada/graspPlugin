/**
   @author Takashi Kitagawa (AIST)
*/

#include "MotionFileControl.h"
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <iostream>
#include <fstream>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/ViewManager>
#endif

#define ARM_NONE			0								// 腕無しロボット
#define ONE_ARM			1								// 片腕ロボット
#define DOUBLE_ARM		2								// 双腕ロボット
#define RIGHT_FLAG		0								// 右腕フラグ
#define LEFT_FLAG			1								// 左腕フラグ
#define RAD_CONV			(3.141592/180.0)				// radian変換
#define RadConv(x)		((x)*(3.141592)/(180.0))
#define DEG_CONV			(180.0/3.141592)				// degree変換
#define DegConv(x)		((x)*(180.0)/(3.141592))

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

MotionFileControl* MotionFileControl::instance()
{
	static MotionFileControl* instance = new MotionFileControl();
	return instance;
}


MotionFileControl::MotionFileControl()
	: mes(*MessageView::mainInstance()),
	os (MessageView::mainInstance()->cout() )
{
	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	InitBasePos = Vector3( 0.0, 0.0, 0.0 );
	InitBaseRPY = Vector3( 0.0, 0.0, 0.0 );

	init		= 0;
	pause		= 0;
	eof			= 0;
	CommandCnt	= 0;
	MotionCnt	= 0;
	ArmType		= ARM_NONE;
	headForward = false;

	loadObjectData();
}


MotionFileControl::~MotionFileControl()
{

}

//-----------------------------------------------------------------------------
// Motion File メイン処理
//-----------------------------------------------------------------------------

void MotionFileControl::loadObjectData()
{
	ifstream ifs("ext/graspPlugin/MotionFile/data/objList.txt");
	string str;

	if(ifs.fail())
		return;

	PlanBase::instance()->multiTargetObject.clear();

	vector<string> file_name;

	while(getline(ifs,str))
		file_name.push_back(str);

	PlanBase::instance()->multiTargetObject.resize(file_name.size());

	for(int i=0; i<file_name.size(); i++){

		BodyItemPtr item = new BodyItem();

		if(item->load(file_name[i], "OpenHRP-VRML-MODEL")) {
			RootItem::mainInstance()->addChildItem(item);
			ItemTreeView::mainInstance()->checkItem(item, true);
		}

		item->body()->link(0)->p() << Vector3::Zero();
		item->body()->link(0)->setAttitude(Matrix3::Identity() );
		item->notifyKinematicStateChange();

		PlanBase::instance()->multiTargetObject[i] = new TargetObject(item);
	}

	return;
}

void MotionFileControl::LoadFromMotionFile(std::string motionfilename)
{
	// ロボットが設定されているか？
	if ( PlanBase::instance()->targetArmFinger == NULL ) {
		MessageView::mainInstance()->cout() << "No data of the robot. (SetRobot)" << endl;
		cout << "targetArmFinger=" << PlanBase::instance()->targetArmFinger << endl;
		return;
	}

	// ロボットの腕タイプを取得
	ArmType = PlanBase::instance()->armsList.size();
	if ( ArmType == ARM_NONE ) {
		MessageView::mainInstance()->cout() << "Arm none." << endl;
		cout << PlanBase::instance()->armsList.size() << endl;
		return;
	}

	// モーションシーケンスのクリア
	PlanBase::instance()->graspMotionSeq.clear();

	if ( !pause ) {
		// ------------------------------------------------------------------------
		// 初期状態(位置/姿勢)から動作開始しするモーションデータの設定
		// ------------------------------------------------------------------------
		if ( !init ) {
			init = 1;
			// --------------------------------------------------------------------
			// プロジェクトファイルを読み込んだ時点の初期モーションデータの取得
			// --------------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "ロボットのボディアイテム数(BodyItemNum) : " << PlanBase::instance()->bodyItemRobot()->body()->numJoints() << endl;
#endif
			int num = PlanBase::instance()->bodyItemRobot()->body()->numJoints();
			InitJnt.resize( num );

#if DEBUG_MOTIONFILE_INPUT
			cout << "ロボットの初期関節角度 : ";
#endif
			for ( int i=0; i < PlanBase::instance()->bodyItemRobot()->body()->numJoints(); i++ ) {
                InitJnt(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
#if DEBUG_MOTIONFILE_INPUT
				cout << DegConv(InitJnt(i)) << " ";
#endif
			}
#if DEBUG_MOTIONFILE_INPUT
			cout << endl;

			cout << "ロボットのボディの初期位置(XYZ) : ";
            cout << PlanBase::instance()->bodyItemRobot()->body()->link(0)->p().transpose() << endl;
			cout << "ロボットのボディの初期姿勢(RPY) : ";
			cout << DegConv(rpyFromRot(PlanBase::instance()->bodyItemRobot()->body()->link(0)->attitude())).transpose() << endl;
#endif
            InitBasePos = PlanBase::instance()->bodyItemRobot()->body()->link(0)->p();
			InitBaseRPY = rpyFromRot(PlanBase::instance()->bodyItemRobot()->body()->link(0)->attitude());

			if ( PlanBase::instance()->targetObject != NULL ) {
                InitObjPos = PlanBase::instance()->object()->p();
				InitObjRPY = rpyFromRot(PlanBase::instance()->object()->attitude());
			}
		}

		// ------------------------------------------------------------------------
		// 初期モーションデータの設定
		// ------------------------------------------------------------------------
		CommandCnt = 0;
		MotionCnt  = 0;

		MotionState tmpState;
		tmpState.pos		= InitBasePos;
		tmpState.rpy		= InitBaseRPY;
		tmpState.jointSeq	= InitJnt;
		tmpState.motionTime	= 1.0;
		tmpState.pathPlanDOF.clear();
		for(int j=0;j<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); j++)
	           tmpState.pathPlanDOF.push_back(PlanBase::instance()->bodyItemRobot()->body()->joint(j)->jointId());

		PlanBase::instance()->graspMotionSeq.push_back(tmpState);

		MessageView::mainInstance()->cout() << "motion.dat(LOAD)" << endl;
	}
	else {
		// ------------------------------------------------------------------------
		// PAUSEから動作開始しするモーションデータの設定
		// ------------------------------------------------------------------------
		PauseState.motionTime	= 1.0;
		PlanBase::instance()->setMotionState( PauseState );
		PlanBase::instance()->graspMotionSeq.push_back( PauseState );
		if ( PlanBase::instance()->targetObject != NULL ) {
            PlanBase::instance()->object()->p() = StartObjPos;
			PlanBase::instance()->object()->setAttitude(rotFromRpy(StartObjRPY));
		}
	}
/*
	// motion.datのファイルパス
	objectBasePath = cnoid::executableTopDirectory() + "/extplugin/graspPlugin/MotionFile/data/motion.dat";
	// 読み込むモーションファイルのパスをmotionfile.datで指定
	FILE *fp;
	if( (fp = fopen((objectBasePath+"motionfile.dat").c_str(), "r")) != NULL ){
		ifstream motionfilePath((objectBasePath+"motionfile.dat").c_str());
		string line;
		while(getline(motionfilePath, line)){
			if(line.empty() || line.compare(0,1,"#")==0) continue;   // 空行と#から始まる行を飛ばす
			motionfilename = line;
			break;
		}
		fclose(fp);
	}
*/
	cout << "load motion file : " << motionfilename << endl;
	ifstream objListFile( (motionfilename).c_str() );
	char line[1024];
	int PauseCnt = 0;
	while ( objListFile ) {
		objListFile.getline( line,1024 );
		if ( objListFile.eof() ) {
			pause = 0;
			MessageView::mainInstance()->cout() << "motion.dat(EOF) " << PlanBase::instance()->graspMotionSeq.size()-1 <<
													":" << CommandCnt <<
													":" << MotionCnt << endl;
			if(headForward) HeadForwardFunc();

			break;
		}

		if ( line[0] == '#' || !strlen( line ) ) { if ( PauseCnt >= pause ) { MotionCnt++; } continue; }

		stringstream ss;
		ss << line;

		// 動作開始時刻
		double Ts, Te;
		ss >> Ts;
		if ( ss.eof() ) {
			MessageView::mainInstance()->cout() << "illigal format! line:" << MotionCnt << " [" << line << "]" << endl;
			continue;
		}

		// 動作終了時刻
		string sTe;
		ss >> sTe;
		if ( sTe.empty() == true ) {
			MessageView::mainInstance()->cout() << "illigal format! line:" << MotionCnt << " [" << line << "]" << endl;
			continue;
		}
		stringstream is;
		is.str( sTe );
		is >> Te;

		// --------------------------------------------------------------------
		// Head Forward Mode
		// --------------------------------------------------------------------
		if ( sTe == "HEAD_FORWARD" ){
				cout << "Head forward mode" << endl;
				headForward = true;
				continue;
		}

		// --------------------------------------------------------------------
		// PAUSE処理
		// --------------------------------------------------------------------
		if ( sTe == "PAUSE" ) {
			PauseCnt++;
			if ( PauseCnt <= pause ) {
				continue;
			}
			else {
				pause++;
				MotionCnt++;

				// PAUSE時点のモーションデータを取得
				PauseState = PlanBase::instance()->graspMotionSeq.back();

				cout << "PAUSE:" << pause << endl;
				MessageView::mainInstance()->cout() << "motion.dat(" << Ts << " PAUSE) " << PlanBase::instance()->graspMotionSeq.size()-1 <<
														":" << CommandCnt <<
														":" << MotionCnt << endl;
				if(headForward) HeadForwardFunc();

				// 一旦抜ける
				break;
			}
		}
		else {
			if ( PauseCnt < pause ) {
				continue;
			}
			if ( ss.eof() ) {
				MessageView::mainInstance()->cout() << "illigal format! line:" << MotionCnt << " [" << line << "]" << endl;
				continue;
			}
		}


		// 動作指令名
		string NAME;
		ss >> NAME;
		if ( ss.eof() ) {
			//MessageView::mainInstance()->cout() << "illigal format! line:" << MotionCnt << " [" << line << "]" << endl;
			//continue;
		}

		// 数値
		double dval;
		vector<double> vdVal;
		while ( !ss.eof() ) {
			ss >> dval;
			vdVal.push_back( dval );
		}
		VectorXd Value(vdVal.size());
		for ( int i = 0; i < vdVal.size(); i++ ) {
			Value(i) = vdVal[i];
		}
#if DEBUG_MOTIONFILE_INPUT
		cout << "motion.dat =>" << line << "<=" << endl;
		cout << "動作開始時刻：" << Ts << endl;
		cout << "動作終了時刻：" << Te << endl;
		cout << "動作指令名　：" << NAME << endl;
		cout << "数値　　　　：";
		for ( int i = 0; i < vdVal.size(); i++ ) {
			if ( i ) cout << ", " << vdVal[i];
			else	 cout << vdVal[i];
		}
		cout << endl;
#endif
		// 片腕コマンド(ARM_)を双腕コマンドの右腕コマンド(RARM_)に変換
		if ( NAME[0] != 'R' && NAME[0] != 'L' && NAME[0] != 'J' ) {
			NAME.insert( 0, "R" );
		}

		CommandCnt++;
		MotionCnt++;

		if ( NAME == "JOINT_ABS" ) {
			// ------------------------------------------------------------
			// JOINT_ABS : 関節角度の絶対値
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : JOINT_ABS" << endl;
#endif
			JointAbs( Ts, Te, Value );
			continue;
		}
		else if ( NAME == "JOINT_REL" ) {
			// ------------------------------------------------------------
			// JOINT_REL : 関節角度の相対値
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : JOINT_REL" << endl;
#endif
			JointRel( Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RARM_XYZ_ABS" ) {
			// ------------------------------------------------------------
			// RARM_XYZ_ABS : 右腕の絶対座標系から見た位置・姿勢
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_XYZ_ABS" << endl;
#endif
			ArmXYZAbs( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RARM_XYZ_REL" ) {
			// ------------------------------------------------------------
			// RARM_XYZ_REL : 右手の位置・姿勢の変位量
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_XYZ_REL" << endl;
#endif
			ArmXYZRel( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RARM_LINEAR_ABS" ) {
			// ------------------------------------------------------------
			// RARM_LINEAR_ABS : 右腕の絶対座標系から見た位置・姿勢(途中を線形補間)
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_LINEAR_ABS" << endl;
#endif
			LinearAbs( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RARM_LINEAR_REL" ) {
			// ------------------------------------------------------------
			// RARM_LINEAR_REL : 右手の位置・姿勢の変位量(途中を線形補間)
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_LINEAR_REL" << endl;
#endif
			LinearRel( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RARM_JNT_ABS" ) {
			// ------------------------------------------------------------
			// RARM_JNT_ABS : 右腕の絶対座標系から見た位置・姿勢
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_JNT_ABS" << endl;
#endif
			ArmJntAbs( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RARM_JNT_REL" ) {
			// ------------------------------------------------------------
			// RARM_JNT_REL : 右手の位置・姿勢の変位量
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_JNT_REL" << endl;
#endif
			ArmJntRel( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RHAND_JNT_ABS" ) {
			// ------------------------------------------------------------
			// RHAND_JNT_ABS : 右のハンドの関節角の絶対値
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RHAND_JNT_ABS" << endl;
#endif
			HandJntAbs( 0, Ts, Te, Value, PlanBase::NOT_GRASPING);
			continue;
		}
		else if ( NAME == "RHAND_JNT_REL" ) {
			// ------------------------------------------------------------
			// RHAND_JNT_REL : 右のハンドの関節角の相対値
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RHAND_JNT_REL" << endl;
#endif
			HandJntRel( 0, Ts, Te, Value, PlanBase::NOT_GRASPING );
			continue;
		}
		else if ( NAME == "RARM_OBJECT" ) {
			// ------------------------------------------------------------
			// RARM_OBJECT : 右ハンドの対象物からの相対位置への移動
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RARM_OBJECT" << endl;
#endif
			ArmObject( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RHAND_JNT_OPEN" ) {
			// ------------------------------------------------------------
			// RHAND_JNT_OPEN : 右ハンドの対象物を離す
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RHAND_JNT_OPEN" << endl;
#endif
			HandJntOpen( 0, Ts, Te );
			continue;
		}
		else if ( NAME == "RHAND_JNT_CLOSE" ) {
			// ------------------------------------------------------------
			// RHAND_JNT_CLOSE : 右ハンドを閉じる
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RHAND_JNT_CLOSE" << endl;
#endif
			HandJntClose( 0, Ts, Te, Value );
			continue;
		}
		else if ( NAME == "RHAND_GRASP" ) {
			// ------------------------------------------------------------
			// RHAND_GRASP : 右ハンドで対象物を掴む
			// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
			cout << "NAME : RHAND_GRASP" << endl;
#endif
			HandGrasp( 0, Ts, Te );
			continue;
		}

		// 双腕ロボットの場合は左腕コマンドを実行する
		if ( ArmType == DOUBLE_ARM ) {
			if ( NAME == "LARM_XYZ_ABS" ) {
				// ------------------------------------------------------------
				// LARM_XYZ_ABS : 左腕の絶対座標系から見た位置・姿勢
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_XYZ_ABS" << endl;
#endif
				ArmXYZAbs( 1, Ts, Te, Value );
			}
			else if ( NAME == "LARM_XYZ_REL" ) {
				// ------------------------------------------------------------
				// LARM_XYZ_REL : 左手の位置・姿勢の変位量
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_XYZ_REL" << endl;
#endif
				ArmXYZRel( 1, Ts, Te, Value );
			}
			else if ( NAME == "LARM_LINEAR_ABS" ) {
				// ------------------------------------------------------------
				// LARM_LINEAR_ABS : 左腕の絶対座標系から見た位置・姿勢(途中を線形補間)
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_LINEAR_ABS" << endl;
#endif
				LinearAbs( 1, Ts, Te, Value );
			}
			else if ( NAME == "LARM_LINEAR_REL" ) {
				// ------------------------------------------------------------
				// LARM_LINEAR_REL : 左手の位置・姿勢の変位量(途中を線形補間)
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_LINEAR_REL" << endl;
#endif
				LinearRel( 1, Ts, Te, Value );
			}
			else if ( NAME == "LARM_JNT_ABS" ) {
				// ------------------------------------------------------------
				// LARM_JNT_ABS : 左腕の絶対座標系から見た位置・姿勢
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_JNT_ABS" << endl;
#endif
				ArmJntAbs( 1, Ts, Te, Value );
			}
			else if ( NAME == "LARM_JNT_REL" ) {
				// ------------------------------------------------------------
				// LARM_JNT_REL : 左手の位置・姿勢の変位量
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_JNT_REL" << endl;
#endif
				ArmJntRel( 1, Ts, Te, Value );
			}
			else if ( NAME == "LHAND_JNT_ABS" ) {
				// ------------------------------------------------------------
				// LHAND_JNT_ABS : 左のハンドの関節角の絶対値
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LHAND_JNT_ABS" << endl;
#endif
				HandJntAbs( 1, Ts, Te, Value, PlanBase::NOT_GRASPING );
			}
			else if ( NAME == "LHAND_JNT_REL" ) {
				// ------------------------------------------------------------
				// LHAND_JNT_REL : 左のハンドの関節角の相対値
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LHAND_JNT_REL" << endl;
#endif
				HandJntRel( 1, Ts, Te, Value, PlanBase::NOT_GRASPING );
			}
			else if ( NAME == "LARM_OBJECT" ) {
				// ------------------------------------------------------------
				// LARM_OBJECT : 左ハンドの対象物からの相対位置への移動
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LARM_OBJECT" << endl;
#endif
				ArmObject( 1, Ts, Te, Value );
			}
			else if ( NAME == "LHAND_JNT_OPEN" ) {
				// ------------------------------------------------------------
				// LHAND_JNT_OPEN : 左ハンドの対象物を離す
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LHAND_JNT_OPEN" << endl;
#endif
				HandJntOpen( 1, Ts, Te );
			}
			else if ( NAME == "LHAND_JNT_CLOSE" ) {
				// ------------------------------------------------------------
				// LHAND_JNT_CLOSE : 左ハンドを閉じる
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LHAND_JNT_CLOSE" << endl;
#endif
				HandJntClose( 1, Ts, Te, Value );
			}
			else if ( NAME == "LHAND_GRASP" ) {
				// ------------------------------------------------------------
				// LHAND_JNT_CLOSE : 左ハンドで対象物を掴む
				// ------------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
				cout << "NAME : LHAND_GRASP" << endl;
#endif
				HandGrasp( 1, Ts, Te );
			}
			// エラー
			else {
				MessageView::mainInstance()->cout() << "Illegal Command! [" << NAME << "]" << endl;
			}
		}
		else if ( ArmType == ONE_ARM ) {
			MessageView::mainInstance()->cout() << "Target Robot is One Arm. Illegal Command! [" << NAME << "]" << endl;
		}
	}

	// ----------------------------------------------------------
	// モーション・ファイルの反映
	// ----------------------------------------------------------
#if DEBUG_MOTIONFILE_INPUT
	cout << "モーション・シーケンス数 : " << PlanBase::instance()->graspMotionSeq.size() << endl;

	for ( int i=0; i < PlanBase::instance()->graspMotionSeq.size(); i++ ) {
		cout << "R[" << i << "]=" << PlanBase::instance()->graspMotionSeq[i].graspingState << ", ";
		cout << "L[" << i << "]=" << PlanBase::instance()->graspMotionSeq[i].graspingState2 << endl;
	}
#endif

	if(PlanBase::instance()->graspMotionSeq.size()>1){
		if(PlanBase::instance()->graspMotionSeq[0].graspingState == PlanBase::GRASPING && PlanBase::instance()->graspMotionSeq[1].graspingState == PlanBase::UNDER_GRASPING)
			PlanBase::instance()->graspMotionSeq[0].graspingState = PlanBase::UNDER_GRASPING;
		if(PlanBase::instance()->graspMotionSeq[0].graspingState2 == PlanBase::GRASPING && PlanBase::instance()->graspMotionSeq[1].graspingState2 == PlanBase::UNDER_GRASPING)
			PlanBase::instance()->graspMotionSeq[0].graspingState2 = PlanBase::UNDER_GRASPING;
	}

    PlanBase::instance()->setObjPos(PlanBase::instance()->object()->p(), PlanBase::instance()->object()->attitude());

	TrajectoryPlanner tp;
	bool success = tp.doTrajectoryPlanning();

	PlanBase::instance()->object()->p() = PlanBase::instance()->objVisPos();
	PlanBase::instance()->object()->R() = PlanBase::instance()->objVisRot();
#if DEBUG_MOTIONFILE_INPUT
	if ( !success ) {
		cout << "FALSE! => doTrajectoryPlanning()" << endl;
	}
	else {
		cout << "TRUE! => doTrajectoryPlanning()" << endl;
	}
#endif

	if ( PlanBase::instance()->targetObject != NULL )
		TrackObjectPose();

}


//-----------------------------------------------------------------------------
// Clearボタンの処理
//-----------------------------------------------------------------------------
void MotionFileControl::ClearMotionFile( void )
{
	PlanBase::instance()->graspMotionSeq.clear();

	int num = PlanBase::instance()->bodyItemRobot()->body()->numJoints();

	for ( int i = 0; i < num; i++ )
            PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q()  = InitJnt(i);

    PlanBase::instance()->object()->p() = InitObjPos;
	PlanBase::instance()->object()->setAttitude(rotFromRpy(InitObjRPY));

	PlanBase::instance()->setGraspingState( PlanBase::NOT_GRASPING );
	PlanBase::instance()->setGraspingState2( PlanBase::NOT_GRASPING );

	PlanBase::instance()->calcForwardKinematics();
	PlanBase::instance()->flush();

	init		= 0;
	pause		= 0;
	eof			= 0;
	CommandCnt	= 0;
	MotionCnt	= 0;
	ArmType		= ARM_NONE;
	headForward = false;
}


// --------------------------------------------------------------------------------------
// JOINT_ABS : 関節角度の絶対値
// --------------------------------------------------------------------------------------
bool MotionFileControl::JointAbs( double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> JointAbs()" << endl;
#endif

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= RadConv(Value);
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();
	for(int j=0;j<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); j++)
           tmpState.pathPlanDOF.push_back(PlanBase::instance()->bodyItemRobot()->body()->joint(j)->jointId());

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- JointAbs()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// JOINT_REL : 関節角度の相対値
// --------------------------------------------------------------------------------------
bool MotionFileControl::JointRel( double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> JointRel()" << endl;
#endif

	int num = PlanBase::instance()->bodyItemRobot()->body()->numJoints();

	for ( int i = 0; i < num; i++ )
		if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
			Value(i) += PlanBase::instance()->graspMotionSeq.back().jointSeq(i) * DEG_CONV;
		else
            Value(i) += PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q() * DEG_CONV;

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= RadConv(Value);
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();
	for(int j=0;j<num; j++)
           tmpState.pathPlanDOF.push_back(PlanBase::instance()->bodyItemRobot()->body()->joint(j)->jointId());

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- JointRel()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RARM_XYZ_ABS:右腕の絶対座標系から見た位置・姿勢
// LARM_XYZ_ABS:左腕の絶対座標系から見た位置・姿勢
// --------------------------------------------------------------------------------------
bool MotionFileControl::ArmXYZAbs( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> ArmXYZAbs(" << LR_flag << ")" << endl;
#endif

	Vector3 pos = Vector3( Value(0), Value(1), Value(2) );
	Vector3 rpy = Vector3( Value(3), Value(4), Value(5) );

	// ロール・ピッチ・ヨーの角度をラジアンに変換
	rpy = RadConv(rpy);

	// 腕の関節数を取得する
	int num = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();

#if 0
	if ( LR_flag == RIGHT_FLAG ) {
		cout << "右腕関節数 = " << num << endl;
		cout << "右腕の絶対座標系の" << endl;
	}
	else {
		cout << "左腕関節数 = " << num << endl;
		cout << "左腕の絶対座標系の" << endl;
	}
	cout << "位置(XYZ):" << endl;
    cout << PlanBase::instance()->arm(LR_flag)->palm->p() << endl;
	cout << "姿勢(RPY):" << endl;
	cout << DegConv(rpyFromRot(PlanBase::instance()->arm(LR_flag)->palm->attitude())) << endl;
	cout << "===手先位置を関節角に変換前===" << endl;
	for ( int i = 0; i < num; i++ ) {
        cout << " joint(" << i << ")->q()=" << DegConv(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->q()) << endl;
	}
#endif

	//手先位置を関節角に変換
	PlanBase::instance()->arm(LR_flag)->IK_arm(pos, PlanBase::instance()->arm(LR_flag)->arm_path->joint(num-1)->calcRfromAttitude(rotFromRpy(rpy)));

#if 0
	cout << "===手先位置を関節角に変換後===" << endl;
	for ( int i = 0; i < num; i++ ) {
        cout << " joint(" << i << ")->q()=" << DegConv(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->q()) << endl;
	}
#endif

	/* ここで、arm変数の引数は右の場合は0で左の場合は1 */

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();
	VectorXd Value2(num);

	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		Value2 = PlanBase::instance()->graspMotionSeq.back().jointSeq;
	else {
		for ( int i = 0; i < num2; i++ )
            Value2(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
	}

	for ( int i=0; i < num; i++ )
		//計算された関節角を代入
        Value2(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId()) = PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->q();

#if 0
	for ( int i = 0; i < Value2.size(); i++ ) {
		cout << "Val(" << i << ")=" << DegConv(Value2(i)) << endl;
	}
#endif

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= Value2;
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();
	for(int i=0;i<PlanBase::instance()->arm(LR_flag)->nJoints;i++)
            tmpState.pathPlanDOF.push_back(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId());

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- ArmXYZAbs()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RARM_XYZ_REL:右腕の絶対座標系から見た位置・姿勢の変位量
// LARM_XYZ_REL:左腕の絶対座標系から見た位置・姿勢の変位量
// --------------------------------------------------------------------------------------
bool MotionFileControl::ArmXYZRel( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> ArmXYZRel(" << LR_flag << ")" << endl;
#endif

	Vector3 pos = Vector3( Value(0), Value(1), Value(2) );
	Vector3 rpy = Vector3( Value(3), Value(4), Value(5) );

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();

	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		for ( int i = 0; i < num2; i++ )
            PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q() = PlanBase::instance()->graspMotionSeq.back().jointSeq(i);

	PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

    pos += PlanBase::instance()->arm(LR_flag)->palm->p();

	rpy  = RadConv(rpy);
	rpy += rpyFromRot(PlanBase::instance()->arm(LR_flag)->palm->attitude());

	// 腕の関節数を取得する
	int num = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();

#if 0
	if ( LR_flag == RIGHT_FLAG ) {
		cout << "右腕関節数 = " << num << endl;
		cout << "右腕の絶対座標系の" << endl;
	}
	else {
		cout << "左腕関節数 = " << num << endl;
		cout << "左腕の絶対座標系の" << endl;
	}
	cout << "位置(XYZ):" << endl;
    cout << PlanBase::instance()->arm(LR_flag)->palm->p() << endl;
	cout << "姿勢(RPY):" << endl;
	cout << DegConv(rpyFromRot(PlanBase::instance()->arm(LR_flag)->palm->attitude())) << endl;
	cout << "===手先位置を関節角に変換前===" << endl;
	for ( int i = 0; i < num; i++ ) {
        cout << " joint(" << i << ")->q()=" << DegConv(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->q()) << endl;
	}
#endif

	//手先位置を関節角に変換
	PlanBase::instance()->arm(LR_flag)->IK_arm(pos, PlanBase::instance()->arm(LR_flag)->arm_path->joint(num-1)->calcRfromAttitude(rotFromRpy(rpy)));

	VectorXd Value2(num2);
	for ( int i = 0; i < num2; i++ )
            Value2(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();

#if 0
	cout << "===手先位置を関節角に変換後===" << endl;
	for ( int i = 0; i < num; i++ ) {
        cout << " joint(" << i << ")->q()=" << DegConv(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->q()) << endl;
	}
	for ( int i = 0; i < Value2.size(); i++ ) {
		cout << " Value2(" << i << ")=" << DegConv(Value2(i)) << endl;
	}
#endif

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= Value2;
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();
	for(int i=0;i<PlanBase::instance()->arm(LR_flag)->nJoints;i++)
            tmpState.pathPlanDOF.push_back(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId());

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- ArmXYZRel()" << endl;
#endif

	return true;
}

bool MotionFileControl::LinearAbs( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> LinearAbs(" << LR_flag << ")" << endl;
#endif

	Vector3 pos_dest = Vector3( Value(0), Value(1), Value(2) );
	Vector3 rpy_dest = RadConv(Vector3( Value(3), Value(4), Value(5) ));
	int dev_cnt = Value(6);
	if(dev_cnt<1) dev_cnt = 1;

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();
	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		for ( int i = 0; i < num2; i++ )
			PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q() = PlanBase::instance()->graspMotionSeq.back().jointSeq(i);

	PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

	Vector3 pos = PlanBase::instance()->arm(LR_flag)->palm->p();
	Vector3 rpy = rpyFromRot(PlanBase::instance()->arm(LR_flag)->palm->attitude());
	Vector3 pos_offset = pos_dest - pos;
	Vector3 rpy_offset = rpy_dest - rpy;

	// 腕の関節数を取得する
	int num = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();

	for( int frame = 0; frame < dev_cnt; ++frame){
		PlanBase::instance()->arm(LR_flag)->IK_arm(pos+pos_offset*(double)(frame+1)/dev_cnt, PlanBase::instance()->arm(LR_flag)->arm_path->joint(num-1)->calcRfromAttitude(rotFromRpy(rpy+rpy_offset*(double)(frame+1)/dev_cnt)));

		VectorXd Value2(num2);

		if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
			Value2 = PlanBase::instance()->graspMotionSeq.back().jointSeq;
		else {
			for ( int i = 0; i < num2; i++ ){
				Value2(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
			}
		}

		for ( int i=0; i < num; i++ ) {
			//計算された関節角を代入
			Value2(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId()) = PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->q();
		}

		tmpState.pos = InitBasePos;
		tmpState.rpy = InitBaseRPY;
		tmpState.jointSeq = Value2;
		tmpState.startTime = Ts+(Te-Ts)*(double)frame/dev_cnt;
		tmpState.endTime = Ts+(Te-Ts)*(double)(frame+1)/dev_cnt;
		tmpState.motionTime = tmpState.endTime - tmpState.startTime;

		tmpState.pathPlanDOF.clear();
		for(int i=0;i<PlanBase::instance()->arm(LR_flag)->nJoints;i++)
	            tmpState.pathPlanDOF.push_back(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId());

		PlanBase::instance()->graspMotionSeq.push_back(tmpState);
	}

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- LinearAbs()" << endl;
#endif

	return true;
}

bool MotionFileControl::LinearRel( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> LinearRel(" << LR_flag << ")" << endl;
#endif

	Vector3 pos_offset = Vector3( Value(0), Value(1), Value(2) );
	Vector3 rpy_offset = RadConv(Vector3( Value(3), Value(4), Value(5) ));
	int dev_cnt = Value(6);
	if(dev_cnt<1) dev_cnt = 1;

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();

	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		for ( int i = 0; i < num2; i++ )
			PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q() = PlanBase::instance()->graspMotionSeq.back().jointSeq(i);

	PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

	Vector3 pos = PlanBase::instance()->arm(LR_flag)->palm->p();
	Vector3 rpy = rpyFromRot(PlanBase::instance()->arm(LR_flag)->palm->attitude());

	// 腕の関節数を取得する
	int num = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();

	for ( int frame = 0; frame < ceil(dev_cnt); ++frame){
		if(frame==1||frame==2) cout << (double)(frame+1)/dev_cnt << endl;
		//手先位置を関節角に変換
		PlanBase::instance()->arm(LR_flag)->IK_arm(pos+pos_offset*(double)(frame+1)/dev_cnt, PlanBase::instance()->arm(LR_flag)->arm_path->joint(num-1)->calcRfromAttitude(rotFromRpy(rpy+rpy_offset*(double)(frame+1)/dev_cnt)));

		VectorXd Value2(num2);
		for ( int i = 0; i < num2; i++ )
			Value2(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();

		tmpState.pos = InitBasePos;
		tmpState.rpy = InitBaseRPY;
		tmpState.jointSeq = Value2;
		tmpState.startTime = Ts+(Te-Ts)*(double)frame/dev_cnt;
		tmpState.endTime = Ts+(Te-Ts)*(double)(frame+1)/dev_cnt;
		tmpState.motionTime = tmpState.endTime - tmpState.startTime;

		tmpState.pathPlanDOF.clear();
		for(int i=0;i<PlanBase::instance()->arm(LR_flag)->nJoints;i++)
	            tmpState.pathPlanDOF.push_back(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId());

		PlanBase::instance()->graspMotionSeq.push_back(tmpState);
	}

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- Linear()" << endl;
#endif

	return true;
}

bool MotionFileControl::setObject(int id)
{
	PlanBase::instance()->SetGraspedObject(PlanBase::instance()->multiTargetObject[id]->bodyItemObject);

	return true;
}

// --------------------------------------------------------------------------------------
// RARM_JNT_ABS:右腕の関節角での位置・姿勢
// LARM_JNT_ABS:左腕の関節角での位置・姿勢
// --------------------------------------------------------------------------------------
bool MotionFileControl::ArmJntAbs( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> ArmJntAbs(" << LR_flag << ")" << endl;
#endif

	int num = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();

	/* ここで、arm変数の引数は右の場合は0で左の場合は1 */

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();
	VectorXd wholeJnt(num2);

	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		wholeJnt = PlanBase::instance()->graspMotionSeq.back().jointSeq;
	else {
		for ( int i = 0; i < num2; i++ )
            wholeJnt(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
	}

	for ( int i = 0; i < num; i++ )
        wholeJnt(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId()) = RadConv(Value(i));

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= wholeJnt;
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();
	for(int i=0;i<PlanBase::instance()->arm(LR_flag)->nJoints;i++)
            tmpState.pathPlanDOF.push_back(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId());

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- ArmJntAbs()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RARM_JNT_REL:右腕の関節角での位置・姿勢の変位量
// LARM_JNT_REL:左手の関節角での位置・姿勢の変位量
// --------------------------------------------------------------------------------------
bool MotionFileControl::ArmJntRel( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> ArmJntRel(" << LR_flag << ")" << endl;
#endif

	int num = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();

	/* ここで、arm変数の引数は右の場合は0で左の場合は1 */

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();
	VectorXd wholeJnt(num2);

	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		wholeJnt = PlanBase::instance()->graspMotionSeq.back().jointSeq;
	else {
		for ( int i = 0; i < num2; i++ )
            wholeJnt(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
	}

	for ( int i = 0; i < num; i++ )
        wholeJnt(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId()) += RadConv(Value(i));

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= wholeJnt;
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();
	for(int i=0;i<PlanBase::instance()->arm(LR_flag)->nJoints;i++)
            tmpState.pathPlanDOF.push_back(PlanBase::instance()->arm(LR_flag)->arm_path->joint(i)->jointId());

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- ArmJntRel()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RHAND_JNT_ABS:右のハンドの関節角の絶対値
// LHAND_JNT_ABS:左のハンドの関節角の絶対値
// --------------------------------------------------------------------------------------
bool MotionFileControl::HandJntAbs( int LR_flag, double Ts, double Te, VectorXd Value, int state )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> HandJntAbs(" << LR_flag << ")" << endl;
#endif

	// 各指の関節数を取得して全指の関節数を求める
	int num = 0;
	for ( int i = 0; i < PlanBase::instance()->nFing(LR_flag); i++ ) {
		num += PlanBase::instance()->fingers(LR_flag,i)->fing_path->numJoints();
		//cout << "指" << i << "の関節数 = " << PlanBase::instance()->fingers(LR_flag,i)->fing_path->numJoints() << endl;
	}
	//cout << "全指の関節数 = " << num << endl;

	/* ここで、fingers変数の引数は右の場合は0で左の場合は1 */

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();

	VectorXd wholeJnt(num2);
	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		wholeJnt = PlanBase::instance()->graspMotionSeq.back().jointSeq;
	else {
		for ( int i = 0; i < num2; i++ )
            wholeJnt(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
	}

	int k = 0;
	for ( int i=0; i < PlanBase::instance()->nFing(LR_flag); i++ ) {
		for ( int j = 0; j < PlanBase::instance()->fingers(LR_flag,i)->fing_path->numJoints(); j++ ) {
            cout << "関節タイプ = " << PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointType() << endl;
			if ( PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointType() == Link::ROTATIONAL_JOINT ) {
                wholeJnt(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointId()) = RadConv(Value(k++));
			}
			else {
                wholeJnt(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointId()) = Value(k++);
				//MessageView::mainInstance()->cout() << "illigal command! finger joint type not ROTATIONAL_JOINT! line:" << MotionCnt << endl;
			}
		}
	}

	if(LR_flag == 0) {
		tmpState.graspingState = state;
		if(state == PlanBase::GRASPING)
			tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	}
	else{
		tmpState.graspingState2 = state;
		if(state == PlanBase::GRASPING)
			tmpState.graspingState = PlanBase::NOT_GRASPING;
	}

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= wholeJnt;
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- HandJntAbs()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RHAND_JNT_REL:右のハンドの関節角の相対値
// LHAND_JNT_REL:左のハンドの関節角の相対値
// --------------------------------------------------------------------------------------
bool MotionFileControl::HandJntRel( int LR_flag, double Ts, double Te, VectorXd Value, int state )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> HandJntRel(" << LR_flag << ")" << endl;
#endif

	// 各指の関節数を取得して全指の関節数を求める
	int num = 0;
	for ( int i = 0; i < PlanBase::instance()->nFing(LR_flag); i++ ) {
		num += PlanBase::instance()->fingers(LR_flag,i)->fing_path->numJoints();
		//cout << "指" << i << "の関節数 = " << PlanBase::instance()->fingers(LR_flag,i)->fing_path->numJoints() << endl;
	}
	//cout << "全指の関節数 = " << num << endl;

	/* ここで、fingers変数の引数は右の場合は0で左の場合は1 */

	int num2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();

	VectorXd wholeJnt(num2);
	if ( PlanBase::instance()->graspMotionSeq.size() > 0 )
		wholeJnt = PlanBase::instance()->graspMotionSeq.back().jointSeq;
	else {
		for ( int i = 0; i < num2; i++ )
            wholeJnt(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q();
	}

	int k = 0;
	for ( int i=0; i < PlanBase::instance()->nFing(LR_flag); i++ ) {
		for ( int j = 0; j < PlanBase::instance()->fingers(LR_flag,i)->fing_path->numJoints(); j++ ) {
            cout << "関節タイプ = " << PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointType() << endl;
			if ( PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointType() == Link::ROTATIONAL_JOINT ) {
                wholeJnt(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointId()) += RadConv(Value(k++));
			}
			else {
                wholeJnt(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->jointId()) += Value(k++);
				//MessageView::mainInstance()->cout() << "illigal command! finger joint type not ROTATIONAL_JOINT! line:" << MotionCnt << endl;
			}
		}
	}

	if(LR_flag == 0) {
		tmpState.graspingState = state;
		if(state == PlanBase::GRASPING)
			tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	}
	else {
		tmpState.graspingState2 = state;
		if(state == PlanBase::GRASPING)
			tmpState.graspingState = PlanBase::NOT_GRASPING;
	}

	tmpState.pos		= InitBasePos;
	tmpState.rpy		= InitBaseRPY;
	tmpState.jointSeq	= wholeJnt;
	tmpState.motionTime = Te - Ts;
	tmpState.startTime = Ts;
	tmpState.endTime = Te;

	tmpState.pathPlanDOF.clear();

	PlanBase::instance()->graspMotionSeq.push_back(tmpState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- HandJntRel()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RARM_OBJECT : 右ハンドの対象物からの相対位置への移動
// LARM_OBJECT : 左ハンドの対象物からの相対位置への移動
// --------------------------------------------------------------------------------------
bool MotionFileControl::ArmObject( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> ArmObject(" << LR_flag << ")" << endl;
#endif

	// 対象物が設定されているか？
	if ( PlanBase::instance()->targetObject == NULL ) {
		MessageView::mainInstance()->cout() << "No data of the object. (SetObject)" << endl;
		cout << "targetObject=" << PlanBase::instance()->targetObject << endl;
		return false;
	}
	cout << "Target Object [" << PlanBase::instance()->targetObject->name() << "]" << endl;

	// 対象物位置
	Vector3	objPos = PlanBase::instance()->objVisPos();
	// 対処物姿勢
	Matrix3	objRot = PlanBase::instance()->objVisRot();

	// 手先の目標位置
	Vector3 pos = objPos + objRot*Vector3( Value(0), Value(1), Value(2) );
	// 手先の目標姿勢
	Vector3 rpy = rpyFromRot( Matrix3( objRot*rotFromRpy( Vector3( RadConv(Value(3)), RadConv(Value(4)), RadConv(Value(5)) ) ) ) );

	VectorXd Val(Value.size());
	Val(0) = pos(0);
	Val(1) = pos(1);
	Val(2) = pos(2);
	Val(3) = DegConv(rpy(0));
	Val(4) = DegConv(rpy(1));
	Val(5) = DegConv(rpy(2));

	ArmXYZAbs( LR_flag, Ts, Te, Val );

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- ArmObject()" << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RHAND_JNT_OPEN : 右ハンドの対象物を離す
// LHAND_JNT_OPEN : 左ハンドの対象物を離す
// --------------------------------------------------------------------------------------
bool MotionFileControl::HandJntOpen( int LR_flag, double Ts, double Te )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> HandJntOpen()" << endl;
#endif

	if ( LR_flag == 0 ) {
		if(PlanBase::instance()->graspMotionSeq.size()>0)
			PlanBase::instance()->graspMotionSeq.back().graspingState = PlanBase::UNDER_GRASPING;
	}
	else {
		if(PlanBase::instance()->graspMotionSeq.size()>0)
			PlanBase::instance()->graspMotionSeq.back().graspingState2 = PlanBase::UNDER_GRASPING;
	}

	int numJoints=0;
	for(int i=0; i<PlanBase::instance()->nFing(LR_flag); i++)
			numJoints += PlanBase::instance()->fingers(LR_flag, i)->fing_path->numJoints();

	VectorXd Value(numJoints);
	int k=0;
	for(int i=0; i<PlanBase::instance()->nFing(LR_flag); i++)
		for(int j=0; j<PlanBase::instance()->fingers(LR_flag, i)->fing_path->numJoints(); j++){
			Value(k) = PlanBase::instance()->fingers(LR_flag,i)->fingerOpenPose[j];
			k++;
		}

	HandJntAbs( LR_flag, Ts, Te, Value , PlanBase::NOT_GRASPING);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- HandJntOpen() " << Value.transpose() << endl;
#endif

	return true;
}

// --------------------------------------------------------------------------------------
// RHAND_JNT_CLOSE : 右ハンドの対象物を掴む
// LHAND_JNT_CLOSE : 左ハンドの対象物を掴む
// --------------------------------------------------------------------------------------
bool MotionFileControl::HandJntClose( int LR_flag, double Ts, double Te, VectorXd Value )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> HandJntClose()" << endl;
#endif

	if ( LR_flag == 0 ) {
		if(PlanBase::instance()->graspMotionSeq.size()>0)
			PlanBase::instance()->graspMotionSeq.back().graspingState = PlanBase::UNDER_GRASPING;
	}
	else {
		if(PlanBase::instance()->graspMotionSeq.size()>0)
			PlanBase::instance()->graspMotionSeq.back().graspingState2 = PlanBase::UNDER_GRASPING;
	}

	HandJntAbs( LR_flag, Ts, Te, Value, PlanBase::GRASPING );

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- HandJntClose()" << endl;
#endif

	return true;
}

bool MotionFileControl::HandGrasp( int LR_flag, double Ts, double Te )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> HandJntClose()" << endl;
#endif

	PlanBase::instance()->setMotionState(PlanBase::instance()->graspMotionSeq.back());

	GraspController::instance()->tc = PlanBase::instance();
	GraspController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
	GraspController::instance()->doGraspPlanning();

	PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();

	if ( LR_flag == 0 ) {
		if(PlanBase::instance()->graspMotionSeq.size()>0)
			PlanBase::instance()->graspMotionSeq.back().graspingState = PlanBase::UNDER_GRASPING;
		PlanBase::instance()->graspMotionState.graspingState = PlanBase::GRASPING;
	}
	else {
		if(PlanBase::instance()->graspMotionSeq.size()>0)
			PlanBase::instance()->graspMotionSeq.back().graspingState2 = PlanBase::UNDER_GRASPING;
		PlanBase::instance()->graspMotionState.graspingState2 = PlanBase::GRASPING;
	}

	PlanBase::instance()->graspMotionSeq.push_back(PlanBase::instance()->graspMotionState);

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- HandGrasp()" << endl;
#endif

	return true;
}

void MotionFileControl::TrackObjectPose()
{
	PlanBase *tc = PlanBase::instance();

	for(size_t i=0; i<tc->graspMotionSeq.size(); i++){
		if(i>0)
			setMotionState(tc->graspMotionSeq[i], tc->graspMotionSeq[i-1]);
		else
			tc->setMotionState(tc->graspMotionSeq[i]);

//		tc->flush();
//		usleep(3000000);
	}

    StartObjPos = tc->object()->p();
	StartObjRPY = rpyFromRot(tc->object()->attitude());
}

void MotionFileControl::setMotionState(MotionState gm, MotionState gm_prev){

	PlanBase *tc = PlanBase::instance();

	for(int i=0;i<tc->body()->numJoints();i++)
        tc->body()->joint(i)->q() = gm.jointSeq[i];

    tc->body()->link(0)->p() = gm.pos;
    tc->body()->link(0)->R() = rotFromRpy(gm.rpy);

	tc->setGraspingState(gm_prev.graspingState);
	tc->setGraspingState2(gm_prev.graspingState2);
	// if( tc->getGraspingStateMainArm()==PlanBase::GRASPING ){
	// 	tc->targetArmFinger->objectPalmPos = gm.objectPalmPos;
	// 	tc->targetArmFinger->objectPalmRot = gm.objectPalmRot;
	// }
	tc->setObjectContactState(gm.objectContactState);
	tc->pathPlanDOF = gm.pathPlanDOF;
	tc->setTolerance(gm.tolerance);
	tc->calcForwardKinematics();
//	tc->motionId = gm.id;

	tc->setGraspingState(gm.graspingState);
	tc->setGraspingState2(gm.graspingState2);
}

bool MotionFileControl::HeadForwardFunc()
{
	//Only Applicable function for Hiro (to be modified)
#if 1
	MotionState ms = PlanBase::instance()->getMotionState();


	for(size_t i=0; i<PlanBase::instance()->graspMotionSeq.size(); i++){

		PlanBase::instance()->graspMotionSeq[i].jointSeq(1) = - PlanBase::instance()->graspMotionSeq[i].jointSeq(0);

		PlanBase::instance()->setMotionState(PlanBase::instance()->graspMotionSeq[i]);

		PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

        Vector3 p = PlanBase::instance()->bodyItemRobot()->body()->link("RARM_JOINT5")->p();
		Matrix3 R = PlanBase::instance()->bodyItemRobot()->body()->link("RARM_JOINT5")->attitude();
		Vector3 m = p + R*Vector3(0.014, 0, 0.095);
		cout << "Marker Position " << m.transpose() << endl;

		PlanBase::instance()->graspMotionSeq[i].jointSeq(0) -= 3.1415/2.0;
		//PlanBase::instance()->graspMotionSeq[i].jointSeq(1) -= 3.1415/2.0;
	}

	PlanBase::instance()->setMotionState(ms);
#endif
	return true;
}


// --------------------------------------------------------------------------------------
// 20140508機能追加
// Save Posiotionボタンの処理
//   ARM_JOINT5の位置姿勢とLRHAND_JOINTの指関節角度の把持DB出力
// --------------------------------------------------------------------------------------
void MotionFileControl::SavePositionToDB( void )
{
#if DEBUG_MOTIONFILE_INPUT
	cout << "start ---> SavePositionToDB()" << endl;
#endif

	// SetRobotで指定されたターゲット・アームのチェック
/*
	if ( PlanBase::instance()->targetArmFinger == PlanBase::instance()->armsList[RIGHT_FLAG] ) {
		cout << "Target Arm Right!" << endl;
        Vector3 P = PlanBase::instance()->bodyItemRobot()->body()->link("RARM_JOINT5")->p();
		Matrix3 R = PlanBase::instance()->bodyItemRobot()->body()->link("RARM_JOINT5")->attitude();
		cout << "位置(XYZ):" << endl;
		cout << P << endl;
		cout << "姿勢(RPY):" << endl;
		cout << DegConv(rpyFromRot(R)) << endl;
		cout << "姿勢(行列):" << endl;
		cout << R << endl;
	}
	else if ( PlanBase::instance()->targetArmFinger == PlanBase::instance()->armsList[LEFT_FLAG] ) {
		cout << "Target Arm Left!" << endl;
	}
	else {
		cout << "Non Target Arm!" << endl;
		return;
	}
*/
	// 対象物が設定されているか？
	if ( PlanBase::instance()->targetObject == NULL ) {
		MessageView::mainInstance()->cout() << "No data of the object. (SetObject)" << endl;
		cout << "targetObject=" << PlanBase::instance()->targetObject << endl;
		return;
	}
#if DEBUG_MOTIONFILE_INPUT
	cout << "ターゲット・オブジェクト [" << PlanBase::instance()->targetObject->name() << "]" << endl;
#endif

	Vector3 P = PlanBase::instance()->palm()->p();
	Matrix3 R = PlanBase::instance()->palm()->attitude();
#if DEBUG_MOTIONFILE_INPUT
	cout << "ターゲット・アームの" << endl;
	cout << "位置(XYZ):" << endl;
	cout << P << endl;
	cout << "姿勢(RPY):" << endl;
	cout << DegConv(rpyFromRot(R)) << endl;
	cout << "姿勢(行列):" << endl;
	cout << R << endl;

	cout << "指は何本?:" << PlanBase::instance()->nFing() << endl;
#endif

	// 出力先フォルダと把持DBファイル名
	string DBname;
	DBname = PlanBase::instance()->dataFilePath() + "preplanning_" + PlanBase::instance()->targetObject->name() + ".txt";
#if DEBUG_MOTIONFILE_INPUT
	cout << "把持ＤＢ名:" << DBname << endl;
#endif

	MessageView::mainInstance()->cout() << "Save position to " << DBname << endl;

	// 把持DBへ出力
	ofstream DB( DBname.c_str(), ios::app );
	DB << "1";
	for ( int i=0; i<3; i++ )
		for ( int j=0; j<3; j++ ) DB << " " << R(i,j);
	for ( int i=0; i<3; i++ ) DB << " " << P(i);

	if ( PlanBase::instance()->nFing() ) {
		for ( int i=0; i<PlanBase::instance()->nFing(); i++ ) {
			for ( int j=0; j<PlanBase::instance()->fingers(i)->fing_path->numJoints(); j++ )
				DB << " " << PlanBase::instance()->body()->joint(PlanBase::instance()->fingers(i)->fing_path->joint(j)->jointId())->q();
		}
	}

	DB << endl;

#if DEBUG_MOTIONFILE_INPUT
	cout << "end <--- SavePositionToDB()" << endl;
#endif
}

/* end */


