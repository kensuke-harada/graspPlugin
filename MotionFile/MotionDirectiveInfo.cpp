#include "MotionDirectiveInfo.h"

#include "MotionDirectiveTypeConverter.h"
#include "CoordinateTypeConverter.h"
#include "UsingHandTypeConverter.h"

using namespace std;
using namespace Eigen;
using namespace motionedit;

double MotionDirectiveInfo::GetStartTime() const
{
	return m_startTime;
}

double MotionDirectiveInfo::GetEndTime() const
{
	return m_endTime;
}

UsingHandType MotionDirectiveInfo::GetUsingHandType() const
{
	return m_usingHandType;
}

MotionDirectiveType MotionDirectiveInfo::GetMotionDirectiveType() const
{
	return m_motionDirectiveType;
}

CoordinateType MotionDirectiveInfo::GetCoordinateType() const
{
	return m_coordinateType;
}

int MotionDirectiveInfo::GetDirectiveValueCount() const
{
	return m_directiveValueVector.size();
}

double MotionDirectiveInfo::GetDirectiveValue(int index) const
{
	return m_directiveValueVector[index];
}

VectorXd MotionDirectiveInfo::GetDirectiveValueVector() const
{
	return m_directiveValueVector;
}

void MotionDirectiveInfo::SetStartTime(double startTime)
{
	m_startTime = startTime;
}

void MotionDirectiveInfo::SetEndTime(double endTime)
{
	m_endTime = endTime;
}

void MotionDirectiveInfo::SetUsingHandType(UsingHandType usingHandType)
{
	m_usingHandType = usingHandType;
}

void MotionDirectiveInfo::SetMotionDirectiveType(MotionDirectiveType motionDirectiveType)
{
	m_motionDirectiveType = motionDirectiveType;
}

void MotionDirectiveInfo::SetCoordinateType(CoordinateType coordinateType)
{
	m_coordinateType = coordinateType;
}

void MotionDirectiveInfo::SetMotionDirectiveValue(int i, double value)
{
	m_directiveValueVector[i] = value;
}

void MotionDirectiveInfo::SetDirectiveValueVector(VectorXd & directiveValueVector)
{
	m_directiveValueVector = directiveValueVector;
}


