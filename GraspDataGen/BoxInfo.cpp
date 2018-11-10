#include "BoxInfo.h"

#include <boost/filesystem.hpp>

#include <cnoid/FileUtil>

#include "../Grasp/VectorMath.h"

using namespace grasp;
using namespace cnoid;
using std::string;

BoxInfo::BoxInfo(const BodyItemPtr box) : target_obj(NULL) {
	target_box = box;
	readBoxInfo();
}

BoxInfo::~BoxInfo() {
	for (size_t i = 0; i < obj_infos.size(); i++) {
		delete obj_infos[i];
	}
	obj_infos.clear();
}

Vector3 BoxInfo::objRelPos() const {
	return target_obj->rel_obj_p;
}

Matrix3 BoxInfo::objRelR() const {
	return target_obj->rel_obj_R;
}

const Length* BoxInfo::marginBoxObj() const {
	return &(target_obj->margin_box_obj);
}

const Length* BoxInfo::marginObjObj() const {
	return &(target_obj->margin_obj_obj);
}

void BoxInfo::readBoxInfo() {
	const Mapping& info = *(body()->info()->toMapping());
	
	const Listing& size_list = *info.findListing("size");
	if (size_list.isValid() && size_list.size() == 2) {
		box_length.x = size_list[0].toDouble();
		box_length.y = size_list[1].toDouble();
	}

	const Listing& initp_list = *info.findListing("initSearchPos");
	if (initp_list.isValid() && initp_list.size() == 2) {
		init_search_rel_p = Vector3(initp_list[0].toDouble(), initp_list[1].toDouble(), 0.0);
	}

	const Listing& olist = *info.findListing("object");

	for (int i = 0; i < olist.size(); i++) {
		const Mapping& o_settings = *olist[i].toMapping();
		if (!o_settings.isValid() || o_settings.empty()) {
			continue;
		}

		ObjectInfo* obj = new ObjectInfo;

		o_settings.read("name", obj->object_name);

		string path;
		o_settings.read("modelFile", path);
		boost::filesystem::path model_path(path);
		if (model_path.has_root_path()) {
			obj->object_model_path = getNativePathString(model_path);
		} else {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			boost::filesystem::path orgpath(target_box->lastAccessedFilePath());
#else
			boost::filesystem::path orgpath(target_box->filePath());
#endif
			obj->object_model_path = getNativePathString(orgpath.parent_path() / model_path);
		}

		const Listing& rel_p_list = *o_settings.findListing("relPos");
		if (rel_p_list.isValid() && rel_p_list.size() == 3) {
			obj->rel_obj_p = Vector3(rel_p_list[0].toDouble(), rel_p_list[1].toDouble(), rel_p_list[2].toDouble());
		}

		const Listing& rel_rpy_list = *o_settings.findListing("relRpy");
		if (rel_rpy_list.isValid() && rel_rpy_list.size() == 3) {
			obj->rel_obj_R = rotFromRpy(rel_rpy_list[0].toDouble(), rel_rpy_list[1].toDouble(), rel_rpy_list[2].toDouble());
		}

		const Listing& boxmargin_list = *o_settings.findListing("marginBoxObj");
		if (boxmargin_list.isValid() && boxmargin_list.size() == 2) {
			obj->margin_box_obj.x = boxmargin_list[0].toDouble();
			obj->margin_box_obj.y = boxmargin_list[1].toDouble();
		}

		const Listing& objmargin_list = *o_settings.findListing("marginObjObj");
		if (objmargin_list.isValid() && objmargin_list.size() == 2) {
			obj->margin_obj_obj.x = objmargin_list[0].toDouble();
			obj->margin_obj_obj.y = objmargin_list[1].toDouble();
		}

		obj_infos.push_back(obj);
	}
}
