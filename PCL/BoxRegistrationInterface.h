#ifndef _PCL_BOXREGISTRATOIONINTERFACE_H_
#define _PCL_BOXREGISTRATOIONINTERFACE_H_

#include <vector>

#include <cnoid/EigenTypes>
#include <cnoid/Body>

#include "exportdef.h"

class EXCADE_API BoxRegistrationInterface {
 public:
	BoxRegistrationInterface();
	virtual ~BoxRegistrationInterface();

	bool registration(cnoid::BodyPtr body,
										const std::vector<cnoid::Vector3>& input,
										const cnoid::Vector3& viewpoint) const;
};

#endif /* _PCL_BOXREGISTRATOIONINTERFACE_H_ */
