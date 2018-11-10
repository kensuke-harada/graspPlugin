#ifndef _GRASP_POINTCLOUDENV_
#define _GRASP_POINTCLOUDENV_

#include <vector>
#include <limits>

#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp {

	class EXCADE_API PointCloudEnv {
	public:
		PointCloudEnv() :
		max_x(-std::numeric_limits<double>::max()), max_y(-std::numeric_limits<double>::max()), max_z(-std::numeric_limits<double>::max()),
			min_x(std::numeric_limits<double>::max()), min_y(std::numeric_limits<double>::max()), min_z(std::numeric_limits<double>::max()) {;}

		std::vector<cnoid::Vector3>& p() {return points;}
		const std::vector<cnoid::Vector3>& p() const {return points;}
		std::vector<cnoid::Vector3>& n() {return normals;}
		const std::vector<cnoid::Vector3>& n() const {return normals;}
		void addPoints(const std::vector<cnoid::Vector3>& _p) {
			points = _p;
			calcAABB();
		}

		void addNormals(const std::vector<cnoid::Vector3>& _n) {
			normals = _n;
		}

		void calcAABB() {
			for(unsigned int i = 0; i < points.size(); i++) {
				max_x = (max_x > points[i](0)) ? max_x : points[i](0);
				min_x = (min_x < points[i](0)) ? min_x : points[i](0);
				max_y = (max_y > points[i](1)) ? max_y : points[i](1);
				min_y = (min_y < points[i](1)) ? min_y : points[i](1);
				max_z = (max_z > points[i](2)) ? max_z : points[i](2);
				min_z = (min_z < points[i](2)) ? min_z : points[i](2);
			}
		}

		double max_x, max_y, max_z, min_x, min_y, min_z;

	private:
		std::vector<cnoid::Vector3> points;
		std::vector<cnoid::Vector3> normals;
	};

}

#endif
