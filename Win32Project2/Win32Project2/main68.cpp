#include "common.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>  //NDT配准类对应头文件
#include <pcl/filters/approximate_voxel_grid.h>

int main()
{
	std::string targetFileName = "E:\\room_scan1.pcd";
	std::string inputFileName = "E:\\room_scan2.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(targetFileName, *target_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>(inputFileName, *input_cloud);

	std::cout << target_cloud->points.size() << "," << input_cloud->points.size() << std::endl;
	
	//将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);

	std::cout << "filterd_cloud size:" << filtered_cloud->size() << std::endl;

	//初始化正态分布变换(NDT)对象
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//为终止条件设置最小转换差异
	ndt.setTransformationEpsilon(0.01);
	//为More-Thuente线搜索设置最大步长
	ndt.setStepSize(0.1);
	//设置NDT网格结构的分辨率
	ndt.setResolution(1.0);
	//设置匹配迭代的最大次数
	ndt.setMaximumIterations(35);
	//设置源点云
	ndt.setInputCloud(filtered_cloud);
	//设置目标点云
	ndt.setInputTarget(target_cloud);

	//设置使用机器人测距法得到的粗略初始变换矩阵结果(非必要） 
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud, init_guess);
	//此处output_cloud不能作为最终的源点云变换，因为上面对源点云进行了滤波处理
	std::cout << "ndt has coverged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;

	//使用创建的变换对未过滤的点云进行变换
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

	int sizeofOutputCloud = output_cloud->size();
	std::cout << sizeofOutputCloud << std::endl;

	return 0;
}