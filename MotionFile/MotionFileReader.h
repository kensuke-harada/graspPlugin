#ifndef MOTIONFILEREADER_H
#define MOTIONFILEREADER_H

#include <istream>
#include <vector>
#include "MotionDirectiveInfo.h"
#include "MotionFileData.h"
#include "TextFieldParser.h"
#include "ToMotionDirectiveInfoConverter.h"

#include <cnoid/Referenced>

namespace motionedit
{
	/**
	 * @brief ファイルから動作指示の情報を読み込むインターフェースです。
	 */
	class IMotionFileReader : public cnoid::Referenced
	{
	public:
		IMotionFileReader() { }

	        // 引数で与えられたファイルを読み込み動作指示データを得る純粋仮想関数です。
		virtual MotionFileData Read(std::string) = 0;
	};

	/**
	* @brief ファイルから動作指示の情報を読み込むインターフェースです。
	*/
	class MotionFileReader : public IMotionFileReader
	{
	public:
		/**
		* @brief コンストラクタです。
		*/
		MotionFileReader() { } 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	        /**
		* @brief ファイル読み込みの際に使用する パーサー と コンバータ を引数に取ります。 
		*/
		MotionFileReader(boost::intrusive_ptr<ITextFieldParser>, boost::intrusive_ptr<IToMotionDirectiveInfoConverter>);
#else
	        /**
		* @brief ファイル読み込みの際に使用する パーサー と コンバータ を引数に取ります。 
		*/
		MotionFileReader(cnoid::ref_ptr<ITextFieldParser>, cnoid::ref_ptr<IToMotionDirectiveInfoConverter>);
#endif
		/**
		* @brief 引数の名前を持つファイルを読み込み動作指示情報を得るインターフェースの実装です。
		*/
		MotionFileData Read(std::string);


	private:
		/**
		* @brief ファイル読み込みクラスの実装クラスです。
		*/
		class Impl : public cnoid::Referenced
		{
		public:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			// パーサ インターフェースです。
			boost::intrusive_ptr<ITextFieldParser> textFieldParser;
			// パースされた文字列から動作指示情報を取り出すインターフェースです。
			boost::intrusive_ptr<IToMotionDirectiveInfoConverter> toMotionDirectiveConverter;
#else
			// パーサ インターフェースです。
			cnoid::ref_ptr<ITextFieldParser> textFieldParser;
			// パースされた文字列から動作指示情報を取り出すインターフェースです。
			cnoid::ref_ptr<IToMotionDirectiveInfoConverter> toMotionDirectiveConverter;
#endif

			// ファイル中に含まれる他の動作指示ファイルの名前を得るメンバ関数です。 
			std::string GetMotionFileName(std::vector<std::vector<std::string> >&);
		    
			// 文字で表記された動作指示情報のリストを得るメンバ関数です。
			std::vector<MotionDirectiveInfo> 
				GetMotionDirectiveInfoList(std::vector<std::vector<std::string> >&);

			// ファイルには他の動作指示ファイルの名前が記載されている場合があります。その位置のインデックスです。
			static const int motionFileNamePos = 0;
		};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<Impl> mImpl;
#else
		cnoid::ref_ptr<Impl> mImpl;
#endif
	};
}

#endif
