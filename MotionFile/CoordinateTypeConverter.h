#ifndef COORDINATETYPE_CONVERTER_H
#define COORDINATETYPE_CONVERTER_H

#include "MotionDirectiveInfo.h"
#include <map>
#include <vector>

#include <cnoid/Referenced>

namespace motionedit
{
	/**
	* @brief 座標系の種類と対応する文字列を相互に変換するインターフェースです。
	*/
	class ICoordinateTypeConverter : public cnoid::Referenced
	{
	public:

		virtual ~ICoordinateTypeConverter(){}
		
		/**
		* @brief 文字列を座標系の種類に変換する純粋仮想関数です。(例:"ABS" → CoordinateType_Abs)
		*/		
		virtual CoordinateType ConvertToCoordinateType(const std::string&) = 0;

		/**
		* @brief 座標系の種類を文字列に変換する純粋仮想関数です。(例:CoordinateType_Abs → "ABS" )
		*/
		virtual std::string ConvertToString(CoordinateType) = 0;

		/**
		* @brief 座標系の種類に変換できる文字列のリストを返す純粋仮想関数です。
		*/
		virtual std::vector<std::string> GetConvertibleStringList() = 0;

		/**
		* @brief 座標系の種類と文字列のマップを返す純粋仮想関数です。
		*/		
		virtual std::map<CoordinateType, std::string> GetMap() = 0;
	};

	/**
	* @brief 座標系の種類と対応する文字列を相互に変換するインターフェースです。
	*/
	class CoordinateTypeConverter : public ICoordinateTypeConverter
	{
	public:

		CoordinateTypeConverter();

		/**
		* @brief 文字列を座標系の種類に変換するメンバ関数です。
		*/
		CoordinateType ConvertToCoordinateType(const std::string&);

		/**
		* @brief 座標系の種類を文字列に変換するメンバ関数です。
		*/
		std::string ConvertToString(CoordinateType);

		/**
		* @brief 座標系の種類に変換できる文字列のリストを返すメンバ関数です。
		*/
		std::vector<std::string> GetConvertibleStringList();

		/**
		* @brief 座標系の種類と文字列のマップを返すメンバ関数です。
		*/		
		std::map<CoordinateType, std::string> GetMap();

	private:

		/**
		* @brief 座標系の種類と文字列のマップです。
		*/
		static std::map<CoordinateType, std::string> m_map;
	};
}
#endif
