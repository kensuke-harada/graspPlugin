#ifndef MOTIONFILEDATA_H
#define MOTIONFILEDATA_H


#include <string>
#include <vector>
#include "MotionDirectiveInfo.h"

namespace motionedit
{
	class MotionFileData
	{
	public:
		std::string motionFileName;

		std::vector<MotionDirectiveInfo> motionDirectiveInfoList;	

		MotionFileData(){};
	};
}

#endif
