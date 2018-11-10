#ifndef _GRASPDATAGEN_BOXINFO_H_
#define _GRASPDATAGEN_BOXINFO_H_
#include <iostream>
#include <string>
#include <vector>

#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>

#include <cnoid/ValueTree>
#include <cnoid/BodyItem>

namespace grasp {
	class BoxInfo;
	typedef boost::shared_ptr<BoxInfo> BoxInfoPtr;
	typedef std::vector<BoxInfoPtr>    BoxInfoArray;

	class Length {
	public:
		double x;
		double y;
	};

	class ObjectInfo {
	public:
		std::string object_name;
		std::string object_model_path;
		cnoid::Vector3 rel_obj_p;
		cnoid::Matrix3 rel_obj_R;
		Length margin_box_obj;
		Length margin_obj_obj;
		cnoid::BodyItemPtr body_item;
	};

	class BoxInfo
		: private boost::noncopyable{
	public:
		explicit BoxInfo(const cnoid::BodyItemPtr box);
		virtual ~BoxInfo();

		cnoid::BodyPtr body() const {return target_box->body();}
		cnoid::BodyItemPtr bodyItem() const {return target_box;}
		const Length* size() const {return &box_length;}
		cnoid::Vector3 initSearchPos() const {return init_search_rel_p;}
		cnoid::Vector3 objRelPos() const;
		cnoid::Matrix3 objRelR() const;
		const Length* marginBoxObj() const;
		const Length* marginObjObj() const;
		const ObjectInfo* getObjInfo() const {return target_obj;}
		void setObjectBodyItem(cnoid::BodyItemPtr bodyitem) {target_obj->body_item = bodyitem;}

		int getObjectSize() const {return obj_infos.size();}
		const ObjectInfo* getObjInfo(int i) {return obj_infos[i];}
		void setTargetObj(int i) {target_obj = obj_infos[i];}
	private:
		BoxInfo();

		cnoid::BodyItemPtr target_box;
		Length box_length;
		cnoid::Vector3 init_search_rel_p;
		ObjectInfo* target_obj;
		std::vector<ObjectInfo*> obj_infos;

		void readBoxInfo();
	};
}

#endif /* _GRASPDATAGEN_BOXINFO_H_ */
