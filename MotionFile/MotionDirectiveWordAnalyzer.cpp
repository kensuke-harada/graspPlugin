#include<vector>
#include<iostream>

#include "MotionDirectiveWordAnalyzer.h"
#include "MotionUtil.h"

using namespace std;
using namespace motionedit;

/**
*@brief 動作指示内容を表す単語の解析クラスの実装クラスです。
*/
class MotionDirectiveWordAnalyzer::Impl : public cnoid::Referenced
{
public:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	/**
	*@brief 文字列から動作指示の種類を得るためのコンバータです。
	*/
	boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter;
	/**
	*@brief 文字列から使用する腕の種類を得るためのコンバータです。
	*/
	boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter;
	/**
	*@brief 文字列から座標系の種類を得るためのコンバータです。
	*/
	boost::intrusive_ptr<ICoordinateTypeConverter> coordinateTypeConverter;
#else
	/**
	*@brief 文字列から動作指示の種類を得るためのコンバータです。
	*/
	cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter;
	/**
	*@brief 文字列から使用する腕の種類を得るためのコンバータです。
	*/
	cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter;
	/**
	*@brief 文字列から座標系の種類を得るためのコンバータです。
	*/
	cnoid::ref_ptr<ICoordinateTypeConverter> coordinateTypeConverter;
#endif
	/**
	*@brief 文字列を使用する腕、動作指示内容及び座標系を表す部分の分解するメンバ関数です。
	*/
	void Decompose(const string&, string&, string&, string&);
};

/**
*@brief 文字列を使用する腕、動作指示内容及び座標系を表す部分の分解するメンバ関数の実装です。
*/
void MotionDirectiveWordAnalyzer::Impl::Decompose(
	const string& motionDirectiveWord,
	string& motionDirectiveTypeString,
	string& usingHandTypeString,
	string& coordinateTypepString
	)
{
	// 分解結果を格納する文字列を作成します。
	{
		motionDirectiveTypeString = "";
		usingHandTypeString = "";
		coordinateTypepString = "";
	}

        // 引数として与えられた文字列のうち動作指示内容を表す文字列を得ます。
	size_t motionDirectivePos;
	{
		vector<string> motionDirectiveStingList = motionDirectiveTypeConverter->GetConvertibleStringList();

		motionDirectivePos = util::Find(
			motionDirectiveWord,
			motionDirectiveStingList,
			motionDirectiveTypeString
			);
		
		if (motionDirectivePos == string::npos)
		{
			return;
		}
	}

	// 引数として与えられた文字列のうち使用する腕の種類の内容を表す文字列を得ます。 
	size_t usingHandTypePos;
	{
		vector<string> usingHandTypeStringList = usingHandTypeConverter->GetConvertibleStringList();

		usingHandTypePos = util::Find(
			motionDirectiveWord.substr(0, motionDirectivePos),
			usingHandTypeStringList,
			usingHandTypeString
			);
	}

	// 引数として与えられた文字列のうち座標系の種類の内容を表す文字列を得ます。
	size_t coordinateTypePos;
	{
		vector<string> coordinateTypeStringList = coordinateTypeConverter->GetConvertibleStringList();

		usingHandTypePos = util::Find(
			motionDirectiveWord.substr(motionDirectivePos + 1),
			coordinateTypeStringList,
			coordinateTypepString
			);
	}


	return;
}

/**
* @breif コンストラクタの実装です。
* 解析の際に使用する、
* 文字列を動作指示情報に変換するコンバータ、使用する腕の種類を文字列に変換するコンバータ、
* 及び座標系の種類を文字列に変換するコンバータを引数に取ります。
*/ 
MotionDirectiveWordAnalyzer::MotionDirectiveWordAnalyzer(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter,
	boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter,
	boost::intrusive_ptr<ICoordinateTypeConverter> coordinateTypeConverter)
#else
	cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter,
	cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter,
	cnoid::ref_ptr<ICoordinateTypeConverter> coordinateTypeConverter)
#endif
	:mImpl(new Impl())
{
	mImpl->motionDirectiveTypeConverter = motionDirectiveTypeConverter;
	mImpl->usingHandTypeConverter = usingHandTypeConverter;
	mImpl->coordinateTypeConverter = coordinateTypeConverter;
}

/**
* @breif 動作指示内容を表す文字列を解析するメンバ関数の実装です。
* 与えられた文字列を 3 つの部分に分解し、使用する腕の種類、動作指示の種類、座標系の種類を得ます。
*/
MotionDirectiveAnalyzeResult MotionDirectiveWordAnalyzer::Analyze(
	const string& motionDirectiveWord
	)
{
	// 与えられた文字列のうち、動作指示の内容を表す部分を格納する変数を作成します。
	string motionDirectiveTypeString;
	
	// 与えられた文字列のうち、使用する腕の内容を表す部分を格納する変数を作成します。	
	string usingHandTypeString;

	// 与えられた文字列のうち、座標系の内容を表す部分を格納する変数を作成します。
	string coordinateTypepString;

	// 与えられた文字列を分解します。
	mImpl->Decompose(
		motionDirectiveWord, 
		motionDirectiveTypeString, 
		usingHandTypeString,
		coordinateTypepString
		);

	// 分解後の結果得られた文字列をそれぞれ、メンバとして持っているコンバータを使用して列挙型データに変換します。
	MotionDirectiveAnalyzeResult result;
	{
		result.motionDirectieType = 
			mImpl->motionDirectiveTypeConverter->ConvertToMotionDirectiveType(motionDirectiveTypeString);
		
		result.usingHandType = 
			mImpl->usingHandTypeConverter->ConvertToUsingHandType(usingHandTypeString);

		result.coordinateType =
			mImpl->coordinateTypeConverter->ConvertToCoordinateType(coordinateTypepString);
	}

	return result;
}
