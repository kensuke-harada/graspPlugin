#ifndef MOTIONDIRECTIVE_WORD_ANALYZER_H
#define MOTIONDIRECTIVE_WORD_ANALYZER_H

#include "MotionDirectiveInfo.h"
#include <string>

#include "MotionDirectiveTypeConverter.h"
#include "CoordinateTypeConverter.h"
#include "UsingHandTypeConverter.h"

#include <cnoid/Referenced>

namespace motionedit
{
	/**
	* @breif 動作指示内容を表す単語の解析結果を表す構造体です。
	*/ 
	struct MotionDirectiveAnalyzeResult
	{
		//解析の結果得られた動作指示の種類です。
		MotionDirectiveType motionDirectieType;
		
		//解析の結果得られた使用する腕の種類です。
		UsingHandType usingHandType;
		
		//解析の結果得られ座標系の種類です。
		CoordinateType coordinateType;
	};

	/**
	* @breif 動作指示内容を表す単語の解析を行うインターフェースです。
	*/ 
	class IMotionDirectiveWordAnalyzer : public cnoid::Referenced
	{
	public:

		virtual ~IMotionDirectiveWordAnalyzer(){};

		/**
		* @breif 動作指示内容を表す単語(例："LHAND_JNT_REL")  を解析して
		* 動作の際に使用する腕、動作指示の内容及び座標系の種類を得る純粋仮想関数です。
		*/ 
		virtual MotionDirectiveAnalyzeResult Analyze(const std::string&) = 0;
	};

	/**
	* @breif 動作指示内容を表す単語の解析を行うクラスです。
	*/
	class MotionDirectiveWordAnalyzer : public IMotionDirectiveWordAnalyzer
	{
	public:

		/**
		* @breif コンストラクタです。
		* 解析の際に使用する、
		* 文字列を動作指示情報に変換するコンバータ、使用する腕の種類を文字列に変換するコンバータ、
		* 及び座標系の種類を文字列に変換するコンバータを引数に取ります。
		*/ 
		MotionDirectiveWordAnalyzer(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<IMotionDirectiveTypeConverter>,
		boost::intrusive_ptr<IUsingHandTypeConverter>,
		boost::intrusive_ptr<ICoordinateTypeConverter>
#else
		cnoid::ref_ptr<IMotionDirectiveTypeConverter>,
		cnoid::ref_ptr<IUsingHandTypeConverter>,
		cnoid::ref_ptr<ICoordinateTypeConverter>
#endif
		);

		/**
		* @breif 動作指示内容を表す文字列を解析するメンバ関数です。
		*/
		MotionDirectiveAnalyzeResult Analyze(const std::string&);
	
	private:
		/**
		*@brief 解析クラスの実装クラスです。
		*/
		class Impl;

		/**
		*@brief 実装クラスへのポインタです。
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<Impl> mImpl;
#else
		cnoid::ref_ptr<Impl> mImpl;
#endif
	};
}
#endif
