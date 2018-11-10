#include "RobotInMotionEdit.h"
#include "MotionDirectiveInfo.h"
#include "MotionDirectiveTypeConverter.h"
#include "UsingHandTypeConverter.h"

using namespace std;
using namespace motionedit;

DefaultRobotInMotionEdit::DefaultRobotInMotionEdit(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter, 
boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter)
#else
cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter, 
cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter)
#endif
{
	 m_motionDirectiveTypeConverter = motionDirectiveTypeConverter;
	 m_usingHandTypeConverter = usingHandTypeConverter;
}

vector<UsingHandType> DefaultRobotInMotionEdit::GetHands()
{
	map<UsingHandType, string> usingHandTypeAndStringMap = m_usingHandTypeConverter->GetMap();

	if(m_handList.empty())
	{
		for(map<UsingHandType, string >::iterator iter = usingHandTypeAndStringMap.begin();
		    iter != usingHandTypeAndStringMap.end();iter++)
		{
			m_handList.push_back(iter->first);
		}
	}

	return m_handList;
}

string DefaultRobotInMotionEdit::GetName()
{
	return "DEFAULT_ROBOT_IN_MOTIONEDIT" ;
}

map<std::string, vector<double> > DefaultRobotInMotionEdit::GetMotionAndDefaultValuesMap()
{
	if(m_motionAndDefaultValueMap.empty())
	{
		string pauseName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_None) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Pause);
		m_motionAndDefaultValueMap[pauseName] = vector<double>(0, 0);

		string headforwardName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_None) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_HeadForward);
		m_motionAndDefaultValueMap[headforwardName] = vector<double>(0, 0);

		string noneName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_None) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_None);
		m_motionAndDefaultValueMap[noneName] = vector<double>(0, 0);

		string jntName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_None) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Joint);
		 m_motionAndDefaultValueMap[jntName] = vector<double>(m_jointCount, 0);

		string rArmXyzName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Xyz);
		m_motionAndDefaultValueMap[rArmXyzName] = vector<double>(m_rightArmJointCount, 0);

		string rArmLinearName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Linear);
		 m_motionAndDefaultValueMap[rArmLinearName] = vector<double>(m_rightArmJointCount, 0);

		string rArmJntName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Jnt);
		 m_motionAndDefaultValueMap[rArmJntName] = vector<double>(m_rightArmJointCount, 0);

		string rArmObjName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Object);
		 m_motionAndDefaultValueMap[rArmObjName] = vector<double>(m_rightArmJointCount, 0);	

		string rHandJntCloseName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Hand_Jnt_Close);
		 m_motionAndDefaultValueMap[rHandJntCloseName] = vector<double>(m_rightArmJointCount, 0);	

		string rHandJntOpenName  =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Hand_Jnt_Open);
		 m_motionAndDefaultValueMap[rHandJntOpenName] = vector<double>(m_rightArmJointCount, 0);	

		string rHandGraspName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Right) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Hand_Grasp);
		 m_motionAndDefaultValueMap[rHandGraspName] = vector<double>(m_rightArmJointCount, 0);	


		string lArmXyzName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Xyz);
		m_motionAndDefaultValueMap[lArmXyzName] = vector<double>(m_rightArmJointCount, 0);

		string lArmLinearName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Linear);
		 m_motionAndDefaultValueMap[lArmLinearName] = vector<double>(m_leftArmJointCount, 0);

		string lArmJntName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Jnt);
		 m_motionAndDefaultValueMap[lArmJntName] = vector<double>(m_leftArmJointCount, 0);

		string lArmObjName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Arm_Object);
		 m_motionAndDefaultValueMap[lArmObjName] = vector<double>(m_leftArmJointCount, 0);	

		string lHandJntCloseName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Hand_Jnt_Close);
		 m_motionAndDefaultValueMap[lHandJntCloseName] = vector<double>(m_rightArmJointCount, 0);	

		string lHandJntOpenName  =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Hand_Jnt_Open);
		 m_motionAndDefaultValueMap[lHandJntOpenName] = vector<double>(m_leftArmJointCount, 0);	

		string lHandGraspName =
		 m_usingHandTypeConverter->ConvertToString(UsingHandType_Left) + m_motionDirectiveTypeConverter->ConvertToString(MotionType_Hand_Grasp);
		 m_motionAndDefaultValueMap[lHandGraspName] = vector<double>(m_leftArmJointCount, 0);
	}

	return m_motionAndDefaultValueMap;
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void DefaultRobotInMotionEdit::SetMotionDirectiveTypeConverter(boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter)
#else
void DefaultRobotInMotionEdit::SetMotionDirectiveTypeConverter(cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter)
#endif
{
	m_motionDirectiveTypeConverter = motionDirectiveTypeConverter;
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void DefaultRobotInMotionEdit::SetUsingHandTypeConverter(boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter)
#else
void DefaultRobotInMotionEdit::SetUsingHandTypeConverter(cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter)
#endif
{
	m_usingHandTypeConverter = usingHandTypeConverter;
}

	
