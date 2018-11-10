#ifndef USINGHANDTYPE_CONVERTER_H
#define USINGHANDTYPE_CONVERTER_H

#include <string>
#include <vector>
#include <map>

#include <cnoid/Referenced>

#include "MotionDirectiveInfo.h"

namespace motionedit
{
	/**
	* @brief 動作の際に使用する腕の種類と対応する文字列を相互に変換するインターフェースです。
	*/
	class IUsingHandTypeConverter : public cnoid::Referenced
	{
	public:

		virtual ~IUsingHandTypeConverter(){}

		/**
		* @brief 文字列を腕の種類に変換する純粋仮想関数です。(例:"R" →  UsingHandType_Right)
		*/
		virtual UsingHandType ConvertToUsingHandType(const std::string&) = 0;

		/**
		* @brief 腕の種類を文字列に変換する純粋仮想関数です。(例:UsingHandType_Right → "R" )
		*/
		virtual std::string ConvertToString(UsingHandType) = 0;

		/**
		* @brief 腕の種類型に変換できる文字列のリストを返す純粋仮想関数です。
		*/
		virtual std::vector<std::string> GetConvertibleStringList() = 0;
		
		/**
		* @brief 腕の種類と文字列のマップを返す純粋仮想関数です。
		*/
		virtual std::map<UsingHandType, std::string> GetMap() = 0;		
	};

	/**
	* @brief 動作の際に使用する腕の種類と対応する文字列を相互に変換するインターフェースです。
	*/
	class UsingHandTypeConverter : public IUsingHandTypeConverter
	{
	public:
		UsingHandTypeConverter();

		/**
		* @brief 文字列を腕の種類に変換するメンバ関数です。
		*/
		UsingHandType ConvertToUsingHandType(const std::string&);

		/**
		* @brief 腕の種類を文字列に変換するメンバ関数です。
		*/
		std::string ConvertToString(UsingHandType);

		/**
		* @brief 腕の種類に変換できる文字列のリストを返すメンバ関数です。
		*/
		std::vector<std::string> GetConvertibleStringList();

		/**
		* @brief 腕の種類と文字列のマップを返すメンバ関数です。
		*/
		std::map<UsingHandType, std::string> GetMap();

	private:
		/**
		* @brief 腕の種類と文字列のマップです。
		*/
		static std::map<UsingHandType, std::string> m_map;
	};
}
#endif

