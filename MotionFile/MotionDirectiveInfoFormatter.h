#ifndef MOTIONDIRECTIVEINFO_FORMATTER_H
#define MOTIONDIRECTIVEINFO_FORMATTER_H

#include <string>

#include "MotionDirectiveTypeConverter.h"
#include "UsingHandTypeConverter.h"
#include "CoordinateTypeConverter.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/intrusive_ptr.hpp>
#else
#include <cnoid/Referenced>
#endif

namespace motionedit
{
	class MotionDirectiveInfo;

	/**
	* @brief　動作指示情報型データを文字列に変換するインターフェースです。 
	*/
	class IMotionDirectiveInfoFormatter : public cnoid::Referenced
	{
	public:

		virtual ~IMotionDirectiveInfoFormatter(){};

		/**
		* @brief 動作指示情報型データを文字列に変換する純粋仮想関数です。
		*/
		virtual std::string Format(const MotionDirectiveInfo&) = 0;
	};

	/**
	* @brief 動作指示情報型データを文字列に変換するクラスです。
	*/
	class MotionDirectiveInfoFormatter : public IMotionDirectiveInfoFormatter
	{
	public:
		/**
	　　	* @brief コンストラクタです。
		* 自身の Foram メソッド内で使用する、 動作指示の種類を文字列に変換するコンバータ、
                * 使用する腕の種類を文字列に変換するコンバータ及びに座標系の種類を文字列に変換するコンバータを引数に取ります。
		*/
		MotionDirectiveInfoFormatter(
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
		* @brief 動作指示情報型データを文字列に変換するメンバ関数です。
		*/
		std::string Format(const MotionDirectiveInfo&);

		/**
		* @brief 変換後の単語間を区切る区切り文字を設定するメンバ関数です。
		*/	
		void SetDelimiter(const std::string&);

	private:
		/**
		* @brief 変換後の単語間を区切る区切り文字を設定するメンバ関数です。
		*/
		std::string m_delimiter;

		/**
		*@brief 動作指示情報を文字列に変換するクラスの実装部を表すクラスです。
		*/
		class Impl : public cnoid::Referenced
		{
		public:
			/**
			*@brief 与えられた座標系の種類、使用する腕の種類及び座標系の種類から、動作指示情報を表す文字列を作成するメンバ関数です。
			*/
			std::string GetMotionDirectiveAndCoordinateString(MotionDirectiveType, UsingHandType, CoordinateType);

			/**
			*@brief動作指示情報と対応する文字列を相互に変換するのコンバータです。
			*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter;
#else
			cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter;
#endif

			/**
			*@brief使用する腕の情報と対応する文字列を相互に変換するコンバータです。
			*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter;
#else
			cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter;
#endif

			/**
			*@brief座標系の種類と文字列を相互に変換するコンバータでです。
			*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			boost::intrusive_ptr<ICoordinateTypeConverter> coordinateTypeConverter;
#else
			cnoid::ref_ptr<ICoordinateTypeConverter> coordinateTypeConverter;
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
