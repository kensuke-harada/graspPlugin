#include <iostream>
#include <exception>

#include "MotionDirectiveInfoFormatter.h"
#include "MotionDirectiveInfo.h"

#include "MotionDirectiveTypeConverter.h"
#include "UsingHandTypeConverter.h"
#include "CoordinateTypeConverter.h"
#include "MotionUtil.h"

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace motionedit;

/**
*@brief 与えられた座標系の種類、使用する腕の種類及び座標系の種類から、動作指示情報を表す文字列を作成するメンバ関数の実装です。
*/
string MotionDirectiveInfoFormatter::Impl::GetMotionDirectiveAndCoordinateString(
	MotionDirectiveType motionDirectiveType,
	UsingHandType usingHandType,
	CoordinateType coordinateType
	)
{
	// 使用する腕の種類に対応する文字列を作成します。
	string usingHandTypeString;
	{
		// 使用する腕の種類が定まらない場合は空の文字列を返します。
		if (usingHandType == UsingHandType_None)
		{
			usingHandTypeString = "";
		}
		// 使用する腕の種類をコンバータを用いて文字列に変換します。
		else
		{
			usingHandTypeString = usingHandTypeConverter->ConvertToString(usingHandType);
		}
	}

	// 動作指示の種類をコンバータを用いて文字列に変換します。
	string motionDirectiveTypeString = motionDirectiveTypeConverter->ConvertToString(motionDirectiveType);

	// 座標系の種類をコンバータを用いて文字列に変換します。
	string coordinateTypeString = coordinateTypeConverter->ConvertToString(coordinateType);

	// 得られた 3 つの文字列を合わせて 1  つの単語を作成します。
        // 座標系を表す文字列の前には "_"  を追加します。
	string resultString = usingHandTypeString+ motionDirectiveTypeString;
	if(!coordinateTypeString.empty())
	{
		resultString += "_";
	}

	return resultString + coordinateTypeString;
}

/**
*@brief 動作指示情報を文字列に変換するクラスのコンストラクタの実装です。
* 内部で使用する、動作指示の種類の変換コンバータ、腕の種類の文字列への変換コンバータ、及び座標系の文字列への変換コンバータを引数に取ります。
*/
MotionDirectiveInfoFormatter::MotionDirectiveInfoFormatter(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter,
	boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter,
	boost::intrusive_ptr<ICoordinateTypeConverter> coordinateTypeConverter
#else
	cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter,
	cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter,
	cnoid::ref_ptr<ICoordinateTypeConverter> coordinateTypeConverter
#endif
	) :mImpl(new Impl())
{

	mImpl->motionDirectiveTypeConverter = motionDirectiveTypeConverter;
	mImpl->usingHandTypeConverter = usingHandTypeConverter;
	mImpl->coordinateTypeConverter = coordinateTypeConverter;
}

/**
*@brief 動作指示情報型のデータを文字列に変換数するメンバ関数の実装です。
*/
string MotionDirectiveInfoFormatter::Format(const MotionDirectiveInfo& motionDirectiveInfo)
{
	// 変換後の文字列を格納する変数を作成します。
	string formattedMotionDirectiveInfo;
		
	// 動作開始時間を表す文字列を得ます。開始時間情報を文字列に変換する際に例外が発生した場合は 
        // "0" とします。 
	string startTimeString;
	try{

		startTimeString = lexical_cast<string>(motionDirectiveInfo.GetStartTime());
	        //startTimeString = util::GetFormattedDoubleString(motionDirectiveInfo.GetStartTime());	

	}catch(std::exception e)
	{
		startTimeString = "0.0000";
	}

	formattedMotionDirectiveInfo += startTimeString;
	formattedMotionDirectiveInfo += m_delimiter;

	// 開始時間と終了時間が異なる場合のみ終了時間を文字列に変換します。
	if (motionDirectiveInfo.GetStartTime() != motionDirectiveInfo.GetEndTime())
	{
		// 動作終了時間を表す文字列を得ます。終了時間情報を文字列に変換する際に例外が発生した場合は 
       		// "0" とします。  
		string endTimeString;
		try{
			endTimeString = lexical_cast<string>(motionDirectiveInfo.GetEndTime());
			//endTimeString = util::GetFormattedDoubleString(motionDirectiveInfo.GetEndTime());
		
		}catch(std::exception e)
		{
			endTimeString = "0.0000";
		}
		formattedMotionDirectiveInfo += endTimeString;
		formattedMotionDirectiveInfo += m_delimiter;
	}

	//与えられた動作指示情報型データ内の、動作指示の種類、使用する腕の種類及び座標系の種類から、
	// 動作の内容と座標系をらわす 1 つの単語を作成し追加します。
	string motionDirectiveName = mImpl->GetMotionDirectiveAndCoordinateString(
		motionDirectiveInfo.GetMotionDirectiveType(),
		motionDirectiveInfo.GetUsingHandType(),
		motionDirectiveInfo.GetCoordinateType()
		);

	formattedMotionDirectiveInfo += motionDirectiveName;

        // 数値文字列を作成します。

	for (int i = 0; i < motionDirectiveInfo.GetDirectiveValueCount(); i++)
	{
		formattedMotionDirectiveInfo += m_delimiter;
		formattedMotionDirectiveInfo += util::GetFormattedDoubleString(motionDirectiveInfo.GetDirectiveValue(i));
	}

	return formattedMotionDirectiveInfo;
}

/**
*@brief 動作指示情報の各要素を区切る区切り文字を設定するメンバ関数の実装です。
*/
void MotionDirectiveInfoFormatter::SetDelimiter(const string& delimiter)
{
	m_delimiter = delimiter;

	return;
}
