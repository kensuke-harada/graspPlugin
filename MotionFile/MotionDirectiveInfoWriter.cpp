#include <iostream>

#include "MotionDirectiveInfoWriter.h"
#include "MotionDirectiveInfoFormatter.h"

using namespace std;
using namespace motionedit;

MotionDirectiveInfoWriter::MotionDirectiveInfoWriter(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<IMotionDirectiveInfoFormatter> formatter
#else
	cnoid::ref_ptr<IMotionDirectiveInfoFormatter> formatter
#endif
	) :m_formatter(formatter)
{}

/** 
* @brief 動作指示型データをストリームに書き込むメンバ関数の実装です。
*/
void MotionDirectiveInfoWriter::Write(
	ostream* stream, 
	const vector<MotionDirectiveInfo>& motionDirectiveInfoList
	)
{
	// 引数として与えられた動作指示情報型データのリストをひとつずつ変換し、ストリームに 1 行ずつ書き込んでいきます。
	for (vector<MotionDirectiveInfo>::const_iterator iter = motionDirectiveInfoList.begin();
		iter != motionDirectiveInfoList.end();
		iter++)
	{
		if ( iter->GetMotionDirectiveType() == MotionType_None ) continue;
		*stream << m_formatter->Format(*iter) + "\n";
	}

	return;
}
