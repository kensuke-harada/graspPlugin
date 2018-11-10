#ifndef _PCL_POINTCLOUDHOLDER_H_
#define _PCL_POINTCLOUDHOLDER_H_

#include <string>

#include <boost/utility.hpp>

#include <cnoid/EigenTypes>

#include "PointCloudTypes.h"

#include "exportdef.h"

class EXCADE_API PointCloudHolder :
boost::noncopyable {
 public:
	static PointCloudHolder* instance();

	PointCloudTConstPtr getCloudPtr() const;
	ColorPointCloudConstPtr getColorCloudPtr() const;

	bool capture(bool do_load_file, bool do_save_file,
							 const std::string& load_filepath, const std::string& save_filepath,
							 bool do_merge = false, bool do_trans = true);

	bool capture(bool do_load_file, bool do_save_file,
							 const std::string& load_filepath, const std::string& save_filepath,
							 const cnoid::Vector3& p, const cnoid::Matrix3& R,
							 bool do_merge = false);

	void clear();

 private:
	PointCloudHolder();

	PointCloudTPtr cloud_;
	ColorPointCloudPtr color_cloud_;

	bool do_correction_;
	double k1_;
	double k2_;
	double d1_;
	double d2_;

	void initialSetting();

	void push(ColorPointCloudPtr cloud, bool do_append,
						const cnoid::Vector3& p, const cnoid::Matrix3& R);
};

class BoxCloudHolder :
boost::noncopyable {
 public:
	static BoxCloudHolder* instance();

	PointCloudTConstPtr getBoxModelPtr() const;
	void setBoxModelCloud(PointCloudTPtr box);
	PointCloudTPtr getBoxPtr() const;
	void setBoxCloud(PointCloudTPtr box);

	void clearBox();
	bool hasBox() const;

 private:
	BoxCloudHolder();

	PointCloudTPtr box_model_cloud_;
	PointCloudTPtr box_cloud_;
	bool has_box_;
};

#endif /* _PCL_POINTCLOUDHOLDER_H_ */
