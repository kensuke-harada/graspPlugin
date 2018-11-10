/**
 * @file   PrimitiveShapeParameter.h
 * @author Akira Ohchi
*/

#ifndef _PCL_PRIMITIVE_SHAPE_PARAMETER_H_
#define _PCL_PRIMITIVE_SHAPE_PARAMETER_H_

#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include <cnoid/EigenTypes>
#include <cnoid/YAMLWriter>
#include <cnoid/YAMLReader>

#include "../Grasp/VectorMath.h"

class CylinderParam {
	public:
	CylinderParam()
		: pos(cnoid::Vector3::Zero())
		, dir(cnoid::Vector3::UnitZ())
		, radius(0.1)
		, length(0.1) {
	}

 CylinderParam(const cnoid::Vector3& pos_, const cnoid::Vector3& dir_, double radius_, double length_)
	 : pos(pos_)
		, dir(dir_)
		, radius(radius_)
		, length(length_) {
	}
	
	virtual ~CylinderParam() {;}

	cnoid::Vector3 pos;
	cnoid::Vector3 dir;
	double radius;
	double length;
};

class BoxParam {
	public:
	BoxParam()
		: pos(cnoid::Vector3::Zero())
		, length(cnoid::Vector3(0.1, 0.1, 0.1))
		, R(cnoid::Matrix3::Identity()) {
	}

	BoxParam(const cnoid::Vector3& pos_, const cnoid::Vector3& length_, const cnoid::Matrix3& R_)
		: pos(pos_)
		, length(length_)
		, R(R_) {
	}
	
	virtual ~BoxParam() {;}

	cnoid::Vector3 pos;
	cnoid::Vector3 length;
	cnoid::Matrix3 R;
};

class PrimitiveShapeHandler {
 public:
	PrimitiveShapeHandler() {;}
	virtual ~PrimitiveShapeHandler() {;}

	void addCylinder(const cnoid::Vector3& center, const cnoid::Vector3& dir, double radius, double len) {
		cylinders_.push_back(CylinderParam(center, dir, radius, len));
	}

	void addBox(const cnoid::Matrix3& r, const cnoid::Vector3& center, const cnoid::Vector3& edge) {
		boxes_.push_back(BoxParam(center, edge, r));
	}

	int cylinderSize() const {return cylinders_.size();}
	int boxSize() const {return boxes_.size();}
	CylinderParam& cylinder(int i) {return cylinders_[i];}
	BoxParam& box(int i) {return boxes_[i];}

	bool loadYaml(const std::string& yaml_path) {
		cylinders_.clear();
		boxes_.clear();
		boost::filesystem::path path(yaml_path);
		if (!boost::filesystem::exists(path)) return false;

		cnoid::YAMLReader yaml_reader;
		cnoid::ValueNode* doc = yaml_reader.loadDocument(yaml_path);
		if (!doc->isMapping()) return false;

		const cnoid::Mapping& root = *(doc->toMapping());
		const cnoid::Listing& shapes_list = *root.findListing("PrimitiveShapes");
		if (!shapes_list.isValid()) return false;

		for (int i = 0; i < shapes_list.size(); i++) {
			std::string type;
			const cnoid::Mapping& shape = *(shapes_list[i].toMapping());
			shape.read("type", type);

			if (type == "cylinder") {
				cnoid::Vector3 center(0, 0, 0);
				cnoid::Vector3 dir(0, 0, 1);
				double radius = 0.1;
				double length = 0.1;

				const cnoid::Listing& center_list = *(shape.findListing("center"));
				if (center_list.isValid() && center_list.size() == 3) {
					center = cnoid::Vector3(center_list[0].toDouble(), center_list[1].toDouble(), center_list[2].toDouble());
				}

				const cnoid::Listing& dir_list = *(shape.findListing("dir"));
				if (dir_list.isValid() && dir_list.size() == 3) {
					dir = cnoid::Vector3(dir_list[0].toDouble(), dir_list[1].toDouble(), dir_list[2].toDouble());
				}
				const cnoid::ValueNode& radius_node = *(shape.find("radius"));
				if (radius_node.isString()) {
					radius = radius_node.toDouble();
				}
				const cnoid::ValueNode& length_node = *(shape.find("length"));
				if (length_node.isString()) {
					length = length_node.toDouble();
				}
				addCylinder(center, dir, radius, length);
			} else if (type == "box") {
				cnoid::Vector3 center(0, 0, 0);
				cnoid::Vector3 rpy(0, 0, 0);
				cnoid::Vector3 length(0.1, 0.1, 0.1);

				const cnoid::Listing& center_list = *(shape.findListing("center"));
				if (center_list.isValid() && center_list.size() == 3) {
					center = cnoid::Vector3(center_list[0].toDouble(), center_list[1].toDouble(), center_list[2].toDouble());
				}

				const cnoid::Listing& length_list = *(shape.findListing("length"));
				if (length_list.isValid() && length_list.size() == 3) {
					length = cnoid::Vector3(length_list[0].toDouble(), length_list[1].toDouble(), length_list[2].toDouble());
				}

				const cnoid::Listing& rpy_list = *(shape.findListing("rpy"));
				if (rpy_list.isValid() && rpy_list.size() == 3) {
					rpy = cnoid::Vector3(rpy_list[0].toDouble(), rpy_list[1].toDouble(), rpy_list[2].toDouble());
				}

				addBox(grasp::rotFromRpy(rpy/180.0*M_PI), center, length);
			} else {
			}
		}
		return true;
	}

	void saveYaml(const std::string& yaml_path) const {
		boost::filesystem::path path(yaml_path);
		cnoid::Mapping* root;
		cnoid::Listing* shapes_list;
		cnoid::YAMLReader yaml_reader;
		if (boost::filesystem::exists(path)) {
			cnoid::ValueNode* doc = yaml_reader.loadDocument(yaml_path);
			if (doc->isMapping()) {
				root = doc->toMapping();
				shapes_list = root->findListing("PrimitiveShapes");
				if (shapes_list->isValid()) {
					shapes_list->clear();
				} else {
					shapes_list = root->createListing("PrimitiveShapes");
				}
			} else {
				root = new cnoid::Mapping();
				shapes_list = root->createListing("PrimitiveShapes");
			}
		} else {
			root = new cnoid::Mapping();
			shapes_list = root->createListing("PrimitiveShapes");
		}
		for (int i = 0; i < cylinders_.size(); i++) {
			cnoid::Mapping* shape_map = new cnoid::Mapping();
			shape_map->write("type", "cylinder");

			cnoid::Listing* center_list = shape_map->createFlowStyleListing("center");
			center_list->append(cylinders_[i].pos(0));
			center_list->append(cylinders_[i].pos(1));
			center_list->append(cylinders_[i].pos(2));

			cnoid::Listing* dir_list = shape_map->createFlowStyleListing("dir");
			dir_list->append(cylinders_[i].dir(0));
			dir_list->append(cylinders_[i].dir(1));
			dir_list->append(cylinders_[i].dir(2));

			shape_map->write("radius", cylinders_[i].radius);

			shape_map->write("length", cylinders_[i].length);

			shapes_list->append(shape_map);
		}
		for (int i = 0; i < boxes_.size(); i++) {
			cnoid::Mapping* shape_map = new cnoid::Mapping();
			shape_map->write("type", "box");

			cnoid::Listing* center_list = shape_map->createFlowStyleListing("center");
			center_list->append(boxes_[i].pos(0));
			center_list->append(boxes_[i].pos(1));
			center_list->append(boxes_[i].pos(2));

			cnoid::Listing* length_list = shape_map->createFlowStyleListing("length");
			length_list->append(boxes_[i].length(0));
			length_list->append(boxes_[i].length(1));
			length_list->append(boxes_[i].length(2));

			cnoid::Vector3 rpy = grasp::rpyFromRot(boxes_[i].R);
			cnoid::Listing* rpy_list = shape_map->createFlowStyleListing("rpy");
			rpy_list->append(rpy(0));
			rpy_list->append(rpy(1));
			rpy_list->append(rpy(2));

			shapes_list->append(shape_map);
		}
		cnoid::YAMLWriter yaml_writer(yaml_path);
		yaml_writer.putNode(root);
	}

 private:
	std::vector<CylinderParam> cylinders_;
	std::vector<BoxParam> boxes_;
};

#endif /* _PCL_PRIMITIVE_SHAPE_PARAMETER_H_ */

















