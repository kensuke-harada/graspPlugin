#include "MotionDirectiveTypeConverter.h"
#include <vector>

using namespace std;
using namespace motionedit;

map<MotionDirectiveType, string>MotionDirectiveTypeConverter::m_map;

/**
*@brief コンストラクタの実装です。
* 動作の種類と文字列のマップが作成されていない場合は、新たに作成します。
*/
MotionDirectiveTypeConverter::MotionDirectiveTypeConverter()
{
if(m_map.size() == 0)
	{
		m_map.insert(make_pair(MotionType_Pause, "PAUSE"));
		m_map.insert(make_pair(MotionType_Joint, "JOINT"));
		m_map.insert(make_pair(MotionType_HeadForward, "HEAD_FORWARD"));
		m_map.insert(make_pair(MotionType_Arm_Xyz, "ARM_XYZ"));
		m_map.insert(make_pair(MotionType_Arm_Linear, "ARM_LINEAR"));
		m_map.insert(make_pair(MotionType_Arm_Jnt, "ARM_JNT"));
		m_map.insert(make_pair(MotionType_Arm_Object, "ARM_OBJECT"));
		m_map.insert(make_pair(MotionType_Hand_Jnt_Open, "HAND_JNT_OPEN"));
		m_map.insert(make_pair(MotionType_Hand_Jnt_Close, "HAND_JNT_CLOSE"));
		m_map.insert(make_pair(MotionType_Hand_Grasp, "HAND_GRASP"));
		m_map.insert(make_pair(MotionType_None, "NONE"));
	}
}

/**
* @brief 文字列を動作の種類に変換するメンバ関数の実装でです。
*/
MotionDirectiveType MotionDirectiveTypeConverter::ConvertToMotionDirectiveType(const string& motionDirectiveTypeString)
{
	for (map<MotionDirectiveType, string>::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
	{
		if (iter->second == motionDirectiveTypeString)
		{
			return iter->first;
		}
	}

	return MotionType_None;
}

string MotionDirectiveTypeConverter::ConvertToString(MotionDirectiveType motionDirectiveType)
{

	if (m_map.find(motionDirectiveType) == m_map.end())
	{
		return "NONE";
	}

	return m_map[motionDirectiveType];
}

vector<std::string>  MotionDirectiveTypeConverter::GetConvertibleStringList()
{
	vector<string> convertibleStringList;
	{
		for (map< MotionDirectiveType, string>::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
		{
			convertibleStringList.push_back(iter->second);
		}
	}

	return convertibleStringList;
}

/**
* @brief 動作の種類と文字列のマップを返すメンバ関数の実装でです。
*/
map<MotionDirectiveType, string> MotionDirectiveTypeConverter::GetMap()
{
	return m_map; 
}
