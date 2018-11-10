#include "CoordinateTypeConverter.h"

#include <iostream>

using namespace motionedit;
using namespace std;

map<CoordinateType, string> CoordinateTypeConverter::m_map;

/**
*@brief コンストラクタの実装です。
* 座標系の種類と文字列のマップが作成されていない場合は、新たに作成します。
*/
CoordinateTypeConverter::CoordinateTypeConverter()
{
	if (m_map.size() == 0)
	{
		m_map.insert(std::make_pair(CoordinateType_Abs, "ABS"));
		m_map.insert(std::make_pair(CoordinateType_Rel, "REL"));
	}
}

/**
* @brief 文字列を座標系の種類に変換するメンバ関数の実装でです。
*/
CoordinateType CoordinateTypeConverter::ConvertToCoordinateType(const string& coordinateTypeString)
{
	for (map<CoordinateType, string>::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
	{
		if (iter->second == coordinateTypeString)
		{
			return iter->first;
		}
	}

	return CoordinateType_None;
}

/**
* @brief 座標系の種類を文字列に変換するメンバ関数の実装です。
*/
string CoordinateTypeConverter::ConvertToString(CoordinateType coordinateType)
{

	if (m_map.find(coordinateType) == m_map.end())
	{
		return "";
	}

	return m_map[coordinateType];
}

/**
* @brief 座標系の種類に変換できる文字列のリストを返すメンバ関数の実装でです。
*/
vector<std::string> CoordinateTypeConverter::GetConvertibleStringList()
{
	vector<string> convertibleStringList;
	{
		for (map<CoordinateType, string>::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
		{
			convertibleStringList.push_back(iter->second);
		}
	}

	return convertibleStringList;
}

/**
* @brief 座標系の種類と文字列のマップを返すメンバ関数の実装でです。
*/
map<CoordinateType, string> CoordinateTypeConverter::GetMap()
{
	return m_map;
}

