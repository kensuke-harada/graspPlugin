#ifndef MOTINDIRECTIVETYPE_CONVERTER_H
#define MOTINDIRECTIVETYPE_CONVERTER_H

#include <string>
#include <vector>
#include <map>
#include "MotionDirectiveInfo.h"
#include <cnoid/Referenced>

namespace motionedit
{
	/**
	* @brief 動作指示の種類と対応する文字列を相互に変換するインターフェースです。
	*/
	class IMotionDirectiveTypeConverter : public cnoid::Referenced
	{
	public:

		virtual ~IMotionDirectiveTypeConverter(){}

		/**
		* @brief 文字列を動作指示の種類に変換する純粋仮想関数です。(例:"PAUSE" →  MotionType_Pause)
		*/
		virtual  MotionDirectiveType ConvertToMotionDirectiveType(const std::string&) = 0;

		/**
		* @brief 動作指示の種類を文字列に変換する純粋仮想関数です。(例:UsingHandType_Right → "R" )
		*/
		virtual std::string ConvertToString(MotionDirectiveType) = 0;

		/**
		* @brief 動作指示の種類に変換できる文字列のリストを返す純粋仮想関数です。
		*/
		virtual std::vector<std::string> GetConvertibleStringList() = 0;

		/**
		* @brief 動作指示の種類と文字列のマップを返す純粋仮想関数です。
		*/
		virtual std::map<MotionDirectiveType, std::string> GetMap() = 0;
	};

	/**
	* @brief 動作指示の種類と対応する文字列を相互に変換するクラスです。
	*/
	class MotionDirectiveTypeConverter : public IMotionDirectiveTypeConverter
	{
	public:

		MotionDirectiveTypeConverter();

		/**
		* @brief 文字列を動作指示の種類に変換するメンバ関数です。
		*/
		MotionDirectiveType ConvertToMotionDirectiveType(const std::string&);

		/**
		* @brief 動作指示の種類を文字列に変換するメンバ関数です。
		*/
		std::string ConvertToString(MotionDirectiveType);

		/**
		* @brief 動作指示の種類に変換できる文字列のリストを返すメンバ関数です。
		*/
		std::vector<std::string> GetConvertibleStringList();

		/**
		* @brief 動作指示の種類と文字列のマップを返すメンバ関数です。
		*/
		std::map<MotionDirectiveType, std::string> GetMap();

	private:

		static std::map<MotionDirectiveType, std::string> m_map;
	};
}
#endif
