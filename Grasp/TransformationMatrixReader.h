#ifndef _GRASP_TRANSFORMATIONMATRIXREADER_H_
#define _GRASP_TRANSFORMATIONMATRIXREADER_H_

#include <string>

#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp {
	class EXCADE_API TransMatReader {
	public:
		/**
		 * @brief load a transfomation matrix from a calibmat file.
		 * @param[in] filepath filepath (absolute path)
		 * @param[out] R rotation matrix
		 * @param[out] p translation vector
		 * @return true : succeeded / false : failed
		 */
		static bool getTransMatrix(const std::string& filepath, cnoid::Matrix3& R, cnoid::Vector3& p);

		static bool writeTransMatrix(const std::string& filepath, const cnoid::Matrix4f& T);
	};
} // namespace grasp

#endif /* _GRASP_TRANSFORMATIONMATRIXREADER_H_ */
