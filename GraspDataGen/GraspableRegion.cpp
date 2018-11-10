#include "GraspableRegion.h"

#include <iostream>
#include <limits>

#include "../Grasp/DrawUtility.h"
#include "../Grasp/VectorMath.h"

using namespace grasp;

double GraspableRegion::grid_interval = 0.05;

GraspableRegion::GraspableRegion() : arm_size(2) {

}

GraspableRegion::GraspableRegion(int _arm_size) {
	arm_size = _arm_size;
}

void GraspableRegion::addGraspablePoint(int x_idx, int y_idx, BoxInfoPtr box, int arm_id) {
	if (hasPoint(x_idx, y_idx)) {
		bool has_box = false;
		for (size_t i = 0; i < points[x_idx][y_idx].box_infos.size(); i++) {
			if (points[x_idx][y_idx].box_infos[i] == box) {
				points[x_idx][y_idx].arm_flags[i].set(arm_id);
				has_box = true;
				break;
			}
		}
		if (!has_box) {
			points[x_idx][y_idx].box_infos.push_back(box);
			boost::dynamic_bitset<> b(arm_size);
			b.set(arm_id);
			points[x_idx][y_idx].arm_flags.push_back(b);
		}
	} else {
		PointProperty pp;
		pp.box_infos.push_back(box);
		boost::dynamic_bitset<> b(arm_size);
		b.set(arm_id);
		pp.arm_flags.push_back(b);
		points[x_idx][y_idx] = pp;
	}
}

cnoid::Vector3 GraspableRegion::getPoint() const {
	int x = points.begin()->first;
	int y = points.begin()->second.begin()->first;
	
	return grid_interval * cnoid::Vector3(x, y, 0.0);
}

cnoid::Vector3 GraspableRegion::getCoord(int i) const {
	int count = 0;
	int x = 0;
	int y = 0;
	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			if (count++ == i) {
				x = xi->first;
				y = yi->first;
				return grid_interval * cnoid::Vector3(x, y, 0.0);
			}
		}
	}
	
	return grid_interval * cnoid::Vector3(x, y, 0.0);
}

Eigen::Vector2i GraspableRegion::getCoordID(int i) const {
	int count = 0;
	int x = 0;
	int y = 0;
	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			if (count++ == i) {
				x = xi->first;
				y = yi->first;
				return Eigen::Vector2i(x, y);
			}
		}
	}
	return Eigen::Vector2i(x, y);
}

cnoid::Vector3 GraspableRegion::getCoord(int x, int y) const {
	return grid_interval * cnoid::Vector3(x, y, 0.0);
}

BoxInfoArray GraspableRegion::getBoxes() const {
	PointProperty pp = getPoint(0);
	return pp.box_infos;
}

BoxInfoArray GraspableRegion::getBoxes(int x, int y) const {
	PointProperty pp = getPoint(x, y);
	return pp.box_infos;
}

BoxInfoArray GraspableRegion::getAllBoxes() const {
	BoxInfoArray ret;
	std::map<BoxInfoPtr, int> boxid;

	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			BoxInfoArray boxes = yi->second.box_infos;
			for(size_t i = 0; i < boxes.size(); i++) {
				if (boxid.count(boxes[i]) == 0) {
					boxid[boxes[i]] = 1;
					ret.push_back(boxes[i]);
				}
			}
		}
	}
	return ret;
}

std::vector<std::vector<int> > GraspableRegion::getArmIDs() const {
	std::vector<std::vector<int> > ret;
	PointProperty pp = getPoint(0);
	for (size_t i = 0; i < pp.arm_flags.size(); i++) {
		std::vector<int> armids;
		for (size_t j = 0; j < pp.arm_flags[i].size(); j++) {
			if (pp.arm_flags[i][j]) {
				armids.push_back(j);
			}
		}
		ret.push_back(armids);
	}
	return ret;
}

std::vector<std::vector<int> > GraspableRegion::getArmIDs(int x, int y) const {
	std::vector<std::vector<int> > ret;
	PointProperty pp = getPoint(x, y);
	for (size_t i = 0; i < pp.arm_flags.size(); i++) {
		std::vector<int> armids;
		for (size_t j = 0; j < pp.arm_flags[i].size(); j++) {
			if (pp.arm_flags[i][j]) {
				armids.push_back(j);
			}
		}
		ret.push_back(armids);
	}
	return ret;
}

int GraspableRegion::size() const {
	int ret = 0;
	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		ret += xi->second.size();
	}
	return ret;
}

Vector3 GraspableRegion::getCenter() const {
	Vector3 ret = Vector3::Zero();
	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			ret += Vector3(xi->first, yi->first, 0);
		}
	}
	ret *= grid_interval;
	ret /= size();

	return ret;
}

Vector3 GraspableRegion::getNearestPoint(const cnoid::Vector3 p) const {
	Vector3 nearest_p;
	double min_dist = std::numeric_limits<double>::max();
	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			Vector3 target_p = grid_interval * Vector3(xi->first, yi->first, 0);
			std::cout << target_p.transpose() << std::endl;
			double dist = (target_p-p).norm() ;
			if ( dist < min_dist) {
				min_dist = dist;
				nearest_p = target_p;
			}
		}
	}
	return nearest_p;
}

void GraspableRegion::displayPoints() const {
	BoxInfoArray boxes = getAllBoxes();
	std::vector<bool> right(boxes.size(), true);
	std::vector<bool> left(boxes.size(), true);
	displayPoints(boxes, right, left);
}

void GraspableRegion::displayPoints(const BoxInfoArray& target_boxes, const std::vector<bool>& right, const std::vector<bool>& left, bool do_clear, bool do_draw) const {
	std::vector<cnoid::Vector3> color;
	color.push_back(Vector3(1.0,0.0,0.0));
	color.push_back(Vector3(0.0,1.0,0.0));
	color.push_back(Vector3(1.0,1.0,0.0));
	color.push_back(Vector3(0.0,1.0,1.0));
	color.push_back(Vector3(1.0,0.0,1.0));
	color.push_back(Vector3(1.0,0.5,0.5));
	color.push_back(Vector3(0.5,1.0,0.5));
	color.push_back(Vector3(1.0,1.0,1.0));

	std::vector<cnoid::Vector3> offset;
	Vector3 t(0.001, 0.0, 0.0);
	offset.push_back(t);
	offset.push_back(rotFromRpy(Vector3(0,0,M_PI)) * t);
	offset.push_back(rotFromRpy(Vector3(0,0,0.5*M_PI)) * t);
	offset.push_back(rotFromRpy(Vector3(0,0,-0.5*M_PI)) * t);
	offset.push_back(rotFromRpy(Vector3(0,0,-0.25*M_PI)) * t);
	offset.push_back(rotFromRpy(Vector3(0,0,0.75*M_PI)) * t);
	offset.push_back(rotFromRpy(Vector3(0,0,0.25*M_PI)) * t);
	offset.push_back(rotFromRpy(Vector3(0,0,-0.75*M_PI)) * t);
	
	DrawUtility* draw = DrawUtility::instance();
	if (do_clear) {
		draw->clear();
	}

	std::map<BoxInfoPtr, int> boxid;

	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			BoxInfoArray boxes = yi->second.box_infos;
			for (size_t i = 0; i < boxes.size(); i++) {
				for (size_t j = 0; j < target_boxes.size(); j++) {
					if (boxes[i] == target_boxes[j]) {
						if ((yi->second.arm_flags[i][0] && right[j]) || (yi->second.arm_flags[i][1] && left[j])) {
							draw->spheres.push_back(Spheres(Vector3(xi->first, yi->first, 0.0) * grid_interval + offset[j%8], 0.01, color[j%8], 1.0));
							break;
						}
					}
				}
			}
		}
	}

	if (do_draw) {
		draw->displayShapes();
	}
}

GraspableRegion GraspableRegion::operator&(const GraspableRegion& r) const {
	GraspableRegion ret;

	for (XIte xi = this->points.begin(); xi != this->points.end(); ++xi) {
		PointArray y_array = xi->second;
		int x_idx = xi->first;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			int y_idx = yi->first;
			if (r.hasPoint(x_idx, y_idx)) {
				ret.addGraspablePoint(x_idx, y_idx, yi->second);
				ret.addGraspablePoint(x_idx, y_idx, r.getPoint(x_idx, y_idx));
			}
		}
	}

	return ret;
}

GraspableRegion GraspableRegion::operator|(const GraspableRegion& r) const{
	GraspableRegion ret;

	ret.points = this->points;

	for (XIte xi = r.points.begin(); xi != r.points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			ret.addGraspablePoint(xi->first, yi->first, yi->second);
		}
	}

	return ret;
}

GraspableRegion& GraspableRegion::operator&=(const GraspableRegion& r){
	GraspableRegion tmp;
	tmp = *this & r;
	*this = tmp;
	return *this;
}

GraspableRegion& GraspableRegion::operator|=(const GraspableRegion& r){
	GraspableRegion tmp;
	tmp = *this | r;
	*this = tmp;
	return *this;
}

bool GraspableRegion::hasPoint(int x, int y) const {
	if (points.count(x) == 0) return false;
	return (points.find(x)->second.count(y) != 0);
}

GraspableRegion::PointProperty GraspableRegion::getPoint(int x, int y) const {
	XIte xi = points.find(x);
	if (xi == points.end()) {std::cerr << "err in GraspableRegion class" << std::endl;}
	YIte yi = xi->second.find(y);
	if (yi == xi->second.end()) {std::cerr << "err in GraspableRegion class" << std::endl;}
	return yi->second;
}

GraspableRegion::PointProperty GraspableRegion::getPoint(int i) const {
	int count = 0;
	for (XIte xi = points.begin(); xi != points.end(); ++xi) {
		PointArray y_array = xi->second;
		for (YIte yi = y_array.begin(); yi != y_array.end(); ++yi) {
			if(count++ == i) return yi->second;
		}
	}
	return PointProperty();
}

void GraspableRegion::addGraspablePoint(int x_idx, int y_idx, const PointProperty& pp) {
	if (hasPoint(x_idx, y_idx)) {
		bool has_box = false;
		for (size_t i = 0; i < pp.box_infos.size(); i++) {
			for (size_t j = 0; j < points[x_idx][y_idx].box_infos.size(); j++) {
				if (points[x_idx][y_idx].box_infos[j] == pp.box_infos[i]) {
					points[x_idx][y_idx].arm_flags[j] |= pp.arm_flags[i];
					has_box = true;
					break;
				}
			}
			if (!has_box) {
				points[x_idx][y_idx].box_infos.push_back(pp.box_infos[i]);
				points[x_idx][y_idx].arm_flags.push_back(pp.arm_flags[i]);
			}
		}
	} else {
		points[x_idx][y_idx] = pp;
	}
}
