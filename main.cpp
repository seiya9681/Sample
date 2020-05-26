#include <iostream>

#include "math.h"
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <Windows.h>
#include <mmsystem.h>
#include <stdlib.h>

#pragma comment(lib,"winmm.lib")

using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main(int argc, char** argv)
{
	const float Intensity = 20;//反射強度除去
	static int scan_start = 6481;//開始スキャン数while文該当箇所変更必要あり
	int scan = 9400;//終了スキャン
	std::string pcd_file_path = "D:/DataBox/同志社-三山木/LS";	//地図生成に使用するpcdファイルが存在するフォルダへのパス
	std::string pose_file_path = "D:/DataBox/同志社-三山木/統合1-2-3/Optimized_pose.csv";	//Optimized_pose
	//std::string pcd_file_path = "D:/DataBox/同志社山手/LS";	//地図生成に使用するpcdファイルが存在するフォルダへのパス
	//std::string pose_file_path = "D:/DataBox/同志社山手/部分地図2-3-1/Optimized_pose.csv";	//地図生成に使用するcsvファイルが存在するフォルダへのパス
	//pcl::visualization::CloudViewer viewer("Map");//viewerを使わないときはコメント

	Vector6d robot_pose;
	int scan_id;
	Eigen::Affine3d affine_trans;
	Eigen::Matrix4d trans_matrix;
	std::string pcd_file;

	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_downsampling(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;

	std::ifstream ifs_csv(pose_file_path, std::ios::in);
	if (!ifs_csv)
		std::cout << "error:can not read csv file!!" << std::endl;	// File open failed

	std::string buf;
	std::getline(ifs_csv, buf);//1行読みとばし
	std::cout << buf << std::endl;
	while (ifs_csv && std::getline(ifs_csv, buf)) {
		std::vector<std::string> v;
		boost::algorithm::split(v, buf, boost::is_any_of(","));
		if (v.size() < 7)
			continue;

		scan_id = std::atoi(v[0].c_str());
		robot_pose(0) = std::stod(v[1].c_str());
		robot_pose(1) = std::stod(v[2].c_str());
		robot_pose(2) = std::stod(v[3].c_str());
		robot_pose(3) = std::stod(v[4].c_str());
		robot_pose(4) = std::stod(v[5].c_str());
		robot_pose(5) = std::stod(v[6].c_str());

		if (scan_id < scan_start) continue;//途中から始めたい
		if (scan_id > scan) break;//途中で止めたいとき(位置把握などに利用)

		affine_trans = Eigen::Translation3d(robot_pose(0), robot_pose(1), robot_pose(2))
			* Eigen::AngleAxisd(robot_pose(5), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(robot_pose(4), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(robot_pose(3), Eigen::Vector3d::UnitX());

		pcd_file.empty();
		pcd_file = pcd_file_path + "/cloud_" + std::to_string(scan_id) + ".pcd";
		std::cout << "rot_ead file:" << pcd_file << std::endl;
		pcl::io::loadPCDFile(pcd_file, *input_cloud);


		//フィルタリング処理

		pass.setInputCloud(input_cloud);
		pass.setFilterFieldName("intensity");
		pass.setFilterLimits(Intensity, FLT_MAX);
		//pass.setFilterLimitsNegative (true);
		pass.filter(*input_cloud);
		//pcl::io::savePCDFileBinary("Output/map_intensity.pcd", *map_cloud_downsampling);

		pcl::transformPointCloud(*input_cloud, *input_cloud, affine_trans.matrix().cast<float>());

		*map_cloud += *input_cloud;

		//pcl::io::savePCDFileBinary("Output/map.pcd", *map_cloud);

		//viewer.showCloud(map_cloud);
	}
	pcl::io::savePCDFileBinary("Output/map.pcd", *map_cloud);
	return 0;
}