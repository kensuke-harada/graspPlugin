#ifndef TO_MOTIONDIRECTIVEINFO_CONVERTER_H
#define TO_MOTIONDIRECTIVEINFO_CONVERTER_H

#include "MotionDirectiveWordAnalyzer.h"
#include<vector>

#include <cnoid/Referenced>

namespace motionedit
{
	class MotionDirectiveInfo;

	/**
	* @brief 動作指示の内容を表す文字列から動作指示情報型のデータに変換するインターフェースです。
	*/
	class IToMotionDirectiveInfoConverter : public cnoid::Referenced
	{
	public:
		virtual ~IToMotionDirectiveInfoConverter(){}
		// 文字列から動作指示情報型のデータに変換する純粋仮想関数です。
		virtual MotionDirectiveInfo Convert(std::vector<std::string>&) = 0;
	};

	/**
	* @brief 動作指示の内容を表す文字列から動作指示情報型のデータに変換するクラスです。
	*/
	class ToMotionDirectiveInfoConverter : public IToMotionDirectiveInfoConverter
	{
	public:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ToMotionDirectiveInfoConverter(boost::intrusive_ptr<IMotionDirectiveWordAnalyzer>);
#else
		ToMotionDirectiveInfoConverter(cnoid::ref_ptr<IMotionDirectiveWordAnalyzer>);
#endif
		/**
	　	* @brief 文字列から動作指示情報型のデータに変換するメンバ関数です。
		*/
		MotionDirectiveInfo Convert(std::vector<std::string>&);
	private:
		class Impl : public cnoid::Referenced
		{
		public:
			std::string GetMotionDirectiveName(const std::vector<std::string>&);
			MotionDirectiveInfo GetMotionDirectiveInfo(const std::string&, const std::vector<std::string>&);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			boost::intrusive_ptr<IMotionDirectiveWordAnalyzer> motionDirectiveWordAnalyzer;
#else
			cnoid::ref_ptr<IMotionDirectiveWordAnalyzer> motionDirectiveWordAnalyzer;
#endif
		};

		/**
	　	* @brief 実装クラスへのポインタです。
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<Impl> mImpl;
#else
		cnoid::ref_ptr<Impl> mImpl;
#endif
	};
}
#endif
