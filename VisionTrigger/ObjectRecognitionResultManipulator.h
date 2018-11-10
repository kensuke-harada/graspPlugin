#ifndef _VISION_TRIGGER_OBJECTRECOGNITIONRESULTMANIPULATOR_H_
#define _VISION_TRIGGER_OBJECTRECOGNITIONRESULTMANIPULATOR_H_

#include <vector>
#include <queue>

#include <boost/thread.hpp>

#include <cnoid/EigenTypes>

#include "exportdef.h"
#include "rtc/ConvertData.h"

namespace grasp {
	class EXCADE_API ObjectRecognitionResultManipulator {
	public:
		static ObjectRecognitionResultManipulator* instance();

		void startRecognition();
		void changeStartFlag(bool val);
		bool getStartFlag();

		void setObjectID(int id);
		int getObjectID() const;

		bool isFinish() const;
		void pushResult(double* res, int len);
		bool popResult(ConvertData::RecognitionResultExt& res);

		void setRecogParams(const ConvertData::RecognitionParam& param);
		void getRecogParamsDblVec(std::vector<double>& param_vec) const;
		void setEnvResult(double* res, int len);
		void getEnvResult(ConvertData::RecognitionEnvResult& res) const;

	private:
		ObjectRecognitionResultManipulator();

		bool recog_start_flag_;
		int obj_id_;

		bool finish_flag_;
		std::queue<ConvertData::RecognitionResultExt> results_;
		ConvertData::RecognitionParam params_;
		ConvertData::RecognitionEnvResult env_res_;

		boost::mutex mtx_;
	};
}

#endif
