#include "ToMotionDirectiveInfoConverter.h"

#include <iostream>
#include <exception>

#include <Eigen/Dense>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "MotionDirectiveInfo.h"
#include "MotionDirectiveTypeConverter.h"
#include "UsingHandTypeConverter.h"
#include "CoordinateTypeConverter.h"
#include "MotionDirectiveWordAnalyzer.h"


using namespace std;
using namespace boost;
using namespace motionedit;
using namespace Eigen;


string ToMotionDirectiveInfoConverter::Impl::GetMotionDirectiveName(const vector<string>& motionDirectiveInfoInfoStringList)
{
	for (vector<string>::const_iterator iter = motionDirectiveInfoInfoStringList.begin();
		iter != motionDirectiveInfoInfoStringList.end();
		iter++)
	{
		if (isalpha((*iter)[0]))
		{
			return *iter;
		}
	}

	throw std::exception();
}

MotionDirectiveInfo ToMotionDirectiveInfoConverter::Impl::GetMotionDirectiveInfo(
	const string& motionDirectiveWord,
	const vector<string>& motionDirectiveInfoInfoStringList
	)
{
	vector<string>::const_iterator iter = motionDirectiveInfoInfoStringList.begin();

	int fieldCount = motionDirectiveInfoInfoStringList.size();

	MotionDirectiveInfo motionDirectiveInfo;

	MotionDirectiveAnalyzeResult analyzeResult = motionDirectiveWordAnalyzer->Analyze(motionDirectiveWord);

	{
		motionDirectiveInfo.SetMotionDirectiveType(analyzeResult.motionDirectieType);
		motionDirectiveInfo.SetUsingHandType(analyzeResult.usingHandType);
		motionDirectiveInfo.SetCoordinateType(analyzeResult.coordinateType);
	}

	// Pause の場合
	if ((analyzeResult.motionDirectieType == MotionType_Pause))
	{
		const int pauseOrHeadForwardSize = 2;
	
		if (fieldCount != pauseOrHeadForwardSize)
		{
			throw std::exception();
		}
	
		double startAndEndTime = lexical_cast<double>(*iter);

		motionDirectiveInfo.SetStartTime(startAndEndTime);
		motionDirectiveInfo.SetEndTime(startAndEndTime);
	
		return motionDirectiveInfo;
	}
	// Pause 以外の場合
	else
	{
		const int minimumSize = 3;

		if (fieldCount < minimumSize)
		{
			throw std::exception();
		}

		motionDirectiveInfo.SetStartTime(lexical_cast<double>(*iter++));
		motionDirectiveInfo.SetEndTime(lexical_cast<double>(*iter++));
		
		// 動作指示単語を表す文字列を飛ばします。
		iter++;

		// JNT_OPEN の場合
		if ((motionDirectiveWord == "LHAND_JNT_OPEN") || (motionDirectiveWord == "RHAND_JNT_OPEN"))
		{
			return motionDirectiveInfo;
		}

		vector<double> valueList;

		while (iter != motionDirectiveInfoInfoStringList.end())
		{
			double value;
			try {
				value = lexical_cast<double>(*iter++);
			} catch (std::exception e) {
				value = 0;
			}
			valueList.push_back(value);		
		}
		
		VectorXd valueVector(valueList.size());
		
		for (int i = 0; i < valueList.size(); i++)
		{
			valueVector[i] = valueList[i];
		}

		motionDirectiveInfo.SetDirectiveValueVector(valueVector);

		return motionDirectiveInfo;	
	}
}

ToMotionDirectiveInfoConverter::ToMotionDirectiveInfoConverter(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<IMotionDirectiveWordAnalyzer> motionDirectiveWordAnalyzer
#else
	cnoid::ref_ptr<IMotionDirectiveWordAnalyzer> motionDirectiveWordAnalyzer
#endif
	) :mImpl(new Impl())
{
	mImpl->motionDirectiveWordAnalyzer = motionDirectiveWordAnalyzer;
}

MotionDirectiveInfo ToMotionDirectiveInfoConverter::Convert(vector<string>& motionDirectiveInfoInfoStringList)
{
	const int minimumListCount = 2;

	if (motionDirectiveInfoInfoStringList.size() < minimumListCount)
	{
		throw std::exception();
	}

	string motionDirectiveWord = mImpl->GetMotionDirectiveName(motionDirectiveInfoInfoStringList);

	return mImpl->GetMotionDirectiveInfo(motionDirectiveWord, motionDirectiveInfoInfoStringList);
}
