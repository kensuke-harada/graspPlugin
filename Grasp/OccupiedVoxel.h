#ifndef _GRASP_OCCUPIEDVOXEL_H_
#define _GRASP_OCCUPIEDVOXEL_H_

#include <vector>
#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp {
	class EXCADE_API OccupiedVoxel {
	public:
		OccupiedVoxel() {;}
		virtual ~OccupiedVoxel() {;}

		enum State {
			UNVISITED,
			FREE,
			OCCUPIED,
			UNKNOWN
		};

		cnoid::Vector3 center;
		cnoid::Vector3 len;
		cnoid::Matrix3 R;
		State data;
		double weight;
	};

	typedef std::vector<OccupiedVoxel> OccupiedVoxelVec;
}

#endif /* _GRASP_OCCUPIEDVOXEL_H_ */
