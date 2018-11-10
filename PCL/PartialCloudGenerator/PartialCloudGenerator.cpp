#include <iostream>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#if (PCL_VERSION_COMPARE(<, 1, 7, 1) && !defined(_WIN32))
#define USE_PCLAPP
#endif

#ifdef USE_PCLAPP
#include <pcl/apps/render_views_tesselated_sphere.h>
#endif

using std::string;
using std::vector;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

class ProgramOption {
 public:
	bool parse(int argc, char** argv) {
		po::options_description opt("option");
		opt.add_options()
			("help,h",                                                             "show help")
			("input_file,i", po::value<string>()->default_value("model.ply"),      "input model file(ply)")
			("output_dir,o", po::value<string>()->default_value("./partialcloud"), "output directory")
			("viewangle,a",  po::value<double>()->default_value(45.0),             "viewangle [deg]");

		try {
			po::store(po::parse_command_line(argc, argv, opt), vm);
			po::notify(vm);
		} catch(const std::exception& e) {
			std::cout << e.what() << std::endl;
			return false;
		}

		if (vm.count("help")) {
			std::cout << opt << std::endl;
			return false;
		}
		return true;
	}
	string getTargetPly() const {
		return vm["input_file"].as<string>();
	}
	string getSaveDir() const {
		return vm["output_dir"].as<string>();
	}
	double getVeiwAngle() const {
		return vm["viewangle"].as<double>();
	}

 private:
	po::variables_map vm;
};

bool writeMatrix(const std::string& file_path, const Eigen::Matrix4f& mat) {
	std::ofstream out(file_path.c_str());
  if (!out) {
    std::cout << "Cannot write file:" << file_path <<  std::endl;
    return false;
  }

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      out << mat(i, j);
      if (!(i == 3 && j == 3))
        out << " ";
    }
  }
  out.close();

  return true;
}

int main(int argc, char** argv) {
	ProgramOption options;
	if (!options.parse(argc, argv)) {
		return -1;
	}

	string ply_path;
	ply_path = options.getTargetPly();
	string save_dir;
	save_dir = options.getSaveDir();

	const fs::path save_dir_path(save_dir);
	if (!fs::exists(save_dir_path)) {
		fs::create_directories(save_dir_path);
	}

#ifndef USE_PCLAPP
	vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud< pcl::PointXYZ> > >  clouds;
#else
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
#endif
	vector<Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > poses;
	vector<float> f;

	double view_angle = options.getVeiwAngle();


#ifndef _WIN32
	// read ply file
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(ply_path.c_str());
	reader->Update();

	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	trans->Scale(1.0, 1.0, 1.0);
	trans->Modified();
	trans->Update();

	vtkSmartPointer<vtkTransformPolyDataFilter> filter_scale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter_scale->SetTransform(trans);
	filter_scale->SetInputConnection(reader->GetOutputPort());
	filter_scale->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();

	vtkSmartPointer<vtkPolyData> polydata = mapper->GetInput();
#endif

#ifndef USE_PCLAPP
	pcl::visualization::PCLVisualizer vis("PartialCloudGenrator");

#ifdef _WIN32
	bool is_read = vis.addModelFromPLYFile(ply_path, "mesh1", 0);
	if (!is_read) {
		std::cerr << "Cannot open ply file:" << ply_path << std::endl;
		return -1;
	}
#else
	vis.addModelFromPolyData(polydata);
#endif

	vis.renderViewTesselatedSphere(256, 256, clouds, poses, f, 1, view_angle);
#else

	// generate views
	pcl::apps::RenderViewsTesselatedSphere render_views;
	render_views.setResolution(256);
	render_views.setUseVertices(true);
	render_views.setRadiusSphere(1.f);
	render_views.setComputeEntropies(false);
	render_views.setTesselationLevel(1);
	render_views.setViewAngle(view_angle);
	render_views.addModelFromPolyData(polydata);
	render_views.generateViews();

	render_views.getViews(clouds);
	render_views.getPoses(poses);
#endif

	pcl::PointCloud<pcl::PointXYZ>::Ptr whole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> sampled_cloud;
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	
	for (size_t i = 0; i < clouds.size(); i++) {
		string num_str = boost::lexical_cast<string>(i);
		string pcd_filename = "model_" + num_str + ".pcd";
		string pose_filename = "pose_" + num_str + ".txt";
		fs::path pcd_path = save_dir_path / fs::path(pcd_filename);
		fs::path pose_path = save_dir_path / fs::path(pose_filename);
#ifndef USE_PCLAPP
		if (pcl::io::savePCDFile(pcd_path.string(), clouds[i]) == -1) {
#else
		if (pcl::io::savePCDFile(pcd_path.string(), *(clouds[i])) == -1) {
#endif
			std::cerr << "Cannot save ply file:" << save_dir + pcd_filename << std::endl;
			return -1;
		}
		transformed_cloud.clear();
		Eigen::Matrix3f rt = poses[i].block(0,0,3,3).transpose();
		Eigen::Matrix4f inv_pose;
		inv_pose.block(0,0,3,3) << rt;
		inv_pose.col(3).head(3) << -rt * poses[i].col(3).head(3);
		inv_pose.row(3) << 0,0,0,1;
#ifndef USE_PCLAPP
		pcl::transformPointCloud(clouds[i], transformed_cloud, inv_pose);
#else
		pcl::transformPointCloud(*(clouds[i]), trnasformed_cloud, inv_pose);
#endif
		*whole_cloud += transformed_cloud;
		writeMatrix(pose_path.string(), poses[i]);
	}

		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setLeafSize(0.001, 0.001, 0.001);
		vg.setInputCloud(whole_cloud);
		vg.filter(sampled_cloud);
		
		pcl::io::savePCDFile(save_dir + "/whole.pcd", sampled_cloud);
	return 0;
}
