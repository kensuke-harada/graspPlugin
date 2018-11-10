#include"UsingHandTypeConverter.h"
#include <vector>

using namespace std;
using namespace motionedit;

map<UsingHandType, string> UsingHandTypeConverter::m_map;

/**
*@brief コンストラクタの実装です。
* 腕の種類と文字列のマップが作成されていない場合は、新たに作成します。
*/
UsingHandTypeConverter::UsingHandTypeConverter()
{
	if (m_map.size() == 0)
	{
		m_map.insert(make_pair(UsingHandType_Right, "R"));
		m_map.insert(make_pair(UsingHandType_Left, "L"));
	}
}

/**
* @brief 文字列を腕の種類に変換するメンバ関数の実装でです。
*/
UsingHandType UsingHandTypeConverter::ConvertToUsingHandType(const string& usingHandTypeString)
{
	for (map<UsingHandType, string>::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
	{
		if (iter->second == usingHandTypeString)
		{
			return iter->first;
		}
	}

	return UsingHandType_None;
}

/**
* @brief 腕の種類を文字列に変換するメンバ関数の実装です。
*/
string UsingHandTypeConverter::ConvertToString(UsingHandType usingHandType)
{

	if (m_map.find(usingHandType) == m_map.end())
	{
		return "";
	}

	return m_map[usingHandType];
}

/**
* @brief 腕の種類に変換できる文字列のリストを返すメンバ関数の実装でです。
*/
vector<std::string>  UsingHandTypeConverter::GetConvertibleStringList()
{
	vector<string> convertibleStringList;
	{
		for (map<UsingHandType, string>::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
		{
			convertibleStringList.push_back(iter->second);
		}
	}

	return convertibleStringList;
}

/**
* @brief 腕の種類と文字列のマップを返すメンバ関数の実装でです。
*/
std::map<UsingHandType, std::string> UsingHandTypeConverter::GetMap()
{
	return m_map;
}
