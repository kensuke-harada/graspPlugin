#ifndef MOTIONDIRECTIVEINFO_WRITER
#define MOTIONDIRECTIVEINFO_WRITER

#include<string>
#include<vector>
#include<ostream>

#include "MotionDirectiveInfo.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/intrusive_ptr.hpp>
#endif
#include <cnoid/Referenced>

namespace motionedit
{
	class IMotionDirectiveInfoFormatter;
	
	/** 
	* @brief 動作指示型データをストリームに書き込むインターフェースです。
	*/
	class IMotionDirectiveInfoWriter : public cnoid::Referenced
	{
	public:

		virtual ~IMotionDirectiveInfoWriter(){}

		/** 
	 	* @brief 動作指示型データを文字列に変換してストリームに書き込む純粋仮想関数です。
		*/
		virtual void Write(std::ostream*, const std::vector<MotionDirectiveInfo>&) = 0;
	};

	/** 
	* @brief 動作指示型データを文字列に変換してストリームに書き込むクラスです。
	*/
	class MotionDirectiveInfoWriter : public IMotionDirectiveInfoWriter
	{
	public:
		/** 
		* @briefコンストラクタです。
		* 動作指示型データを文字列に変換するインターフェースを引数に取ります。
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		MotionDirectiveInfoWriter(boost::intrusive_ptr<IMotionDirectiveInfoFormatter>);
#else
		MotionDirectiveInfoWriter(cnoid::ref_ptr<IMotionDirectiveInfoFormatter>);
#endif

		/** 
		* @brief 動作指示型データを字列に変換してストリームに書き込むメンバ関数です。
		*/
		void Write(std::ostream*, const std::vector<MotionDirectiveInfo>&);

	private:

		/** 
		* @brief 動作指示型データを文字列に変換する際に使用するメンバです。
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<IMotionDirectiveInfoFormatter> m_formatter;
#else
		cnoid::ref_ptr<IMotionDirectiveInfoFormatter> m_formatter;
#endif
	};
}

#endif
