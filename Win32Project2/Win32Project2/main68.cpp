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
#include <pcl/registration/ndt.h>  //NDT��׼���Ӧͷ�ļ�
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
	
	//�������ɨ����˵�ԭʼ�ߴ�Ĵ��10%�����ƥ����ٶ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);

	std::cout << "filterd_cloud size:" << filtered_cloud->size() << std::endl;

	//��ʼ����̬�ֲ��任(NDT)����
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//Ϊ��ֹ����������Сת������
	ndt.setTransformationEpsilon(0.01);
	//ΪMore-Thuente������������󲽳�
	ndt.setStepSize(0.1);
	//����NDT����ṹ�ķֱ���
	ndt.setResolution(1.0);
	//����ƥ�������������
	ndt.setMaximumIterations(35);
	//����Դ����
	ndt.setInputCloud(filtered_cloud);
	//����Ŀ�����
	ndt.setInputTarget(target_cloud);

	//����ʹ�û����˲�෨�õ��Ĵ��Գ�ʼ�任������(�Ǳ�Ҫ�� 
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud, init_guess);
	//�˴�output_cloud������Ϊ���յ�Դ���Ʊ任����Ϊ�����Դ���ƽ������˲�����
	std::cout << "ndt has coverged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;

	//ʹ�ô����ı任��δ���˵ĵ��ƽ��б任
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

	int sizeofOutputCloud = output_cloud->size();
	std::cout << sizeofOutputCloud << std::endl;

	return 0;
}