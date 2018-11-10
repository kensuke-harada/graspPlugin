#ifndef _PCL_POINTCLOUDHANDLERINTERFACE_H_
#define _PCL_POINTCLOUDHANDLERINTERFACE_H_

#include <vector>
#include <cnoid/ColdetModel>
#include "exportdef.h"

class EXCADE_API PointCloudHandlerInterface {
public:
	PointCloudHandlerInterface(){;}
	virtual ~PointCloudHandlerInterface(){;}

	class Cylinder{
	public:
		cnoid::Vector3 pos, dir;
		double radius, len;
	};

	void cylinderSegmentation(const cnoid::ColdetModelPtr object, std::vector<Cylinder>& cylinders, double dist_th = 0.04, int max_num = 1);
private:
};

#endif /* _PCL_POINTCLOUDHANDLERINTERFACE_H_ */
