#ifndef _GRASPDATAGEN_GRASPABLEREGION_H_
#define _GRASPDATAGEN_GRASPABLEREGION_H_

#include <map>

#include <boost/dynamic_bitset.hpp>

#include <cnoid/EigenTypes>

#include "BoxInfo.h"

namespace grasp {
	class GraspableRegion {
	public:
		static double grid_interval;

		GraspableRegion();
		explicit GraspableRegion(int _arm_size);

		void addGraspablePoint(int x_idx, int y_idx, BoxInfoPtr box, int arm_id);
		
		cnoid::Vector3 getPoint() const;
		cnoid::Vector3 getCoord(int i) const;
		cnoid::Vector3 getCoord(int x, int y) const;
		Eigen::Vector2i getCoordID(int i) const;
		BoxInfoArray getBoxes() const;
		BoxInfoArray getBoxes(int x, int y) const;
		BoxInfoArray getAllBoxes() const;
		std::vector<std::vector<int> > getArmIDs() const;
		std::vector<std::vector<int> > getArmIDs(int x, int y) const;

		bool empty() const {return points.empty();}
		int size() const;
		cnoid::Vector3 getCenter() const;
		cnoid::Vector3 getNearestPoint(const cnoid::Vector3 p) const;
		

		void displayPoints() const;
		void displayPoints(const BoxInfoArray& boxes, const std::vector<bool>& right, const std::vector<bool>& left, bool do_clear = true, bool do_draw = true) const;
		GraspableRegion operator&(const GraspableRegion& r) const;
		GraspableRegion operator|(const GraspableRegion& r) const;
		GraspableRegion& operator&=(const GraspableRegion& r);
		GraspableRegion& operator|=(const GraspableRegion& r);
	private:
		class PointProperty {
		public:
			std::vector<BoxInfoPtr> box_infos;
			std::vector<boost::dynamic_bitset<> > arm_flags;
		};
		typedef std::map<int, PointProperty> PointArray;
		typedef std::map<int, PointArray>    Points;
		typedef Points::const_iterator       XIte;
		typedef PointArray::const_iterator   YIte;

		Points points;
		int arm_size;
		
		bool hasPoint(int x, int y) const;
		PointProperty getPoint(int x, int y) const;
		PointProperty getPoint(int i) const;

		void addGraspablePoint(int x_idx, int y_idx, const PointProperty& pp);
	};
}

#endif
