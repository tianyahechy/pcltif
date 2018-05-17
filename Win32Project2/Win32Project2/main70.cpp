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
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>  //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��

#include <boost/thread/thread.hpp>

typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
fpfhFeature::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//������
	pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	est_normal.compute(*point_normal);

	//fpfh����
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);

	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;
}
int main()
{
	//std::string targetFileName1 = "E:\\room_scan1.pcd";
	//std::string inputFileName1 = "E:\\room_scan2.pcd";
	std::string targetFileName = "E:\\DEM-2013.pcd";
	std::string inputFileName = "E:\\DEM-2016.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(targetFileName, *target_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>(inputFileName, *input_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < 1125860; i++)
	{
		double x = target_cloud->points[i].x;
		double y = target_cloud->points[i].y;
		double z = target_cloud->points[i].z;
		pcl::PointXYZ thePt;
		thePt.x = x;
		thePt.y = y;
		thePt.z = z;
		target_cloud2->push_back(thePt);

		double x2 = input_cloud->points[i].x;
		double y2 = input_cloud->points[i].y;
		double z2 = input_cloud->points[i].z;
		pcl::PointXYZ thePt2;
		thePt2.x = x2;
		thePt2.y = y2;
		thePt2.z = z2;
		input_cloud2->push_back(thePt2);
	}
	std::cout << target_cloud->points.size() << "," << input_cloud->points.size() << std::endl;

	//�������ɨ����˵�ԭʼ�ߴ�Ĵ��10%�����ƥ����ٶ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	//approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setLeafSize(70, 70, 70);
	approximate_voxel_filter.setInputCloud(input_cloud2);
	approximate_voxel_filter.filter(*filtered_cloud);

	std::cout << "filterd_cloud size:" << filtered_cloud->size() << std::endl;
	
	//��ʼ����̬�ֲ��任(NDT)����
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//Ϊ��ֹ����������Сת������
	ndt.setTransformationEpsilon(0.01);
	//ΪMore-Thuente������������󲽳�
	ndt.setStepSize(0.1);
	//����NDT����ṹ�ķֱ���
	//ndt.setResolution(1.0);
	ndt.setResolution(10000);
	//����ƥ�������������
	ndt.setMaximumIterations(35);
	//����Դ����
	ndt.setInputCloud(filtered_cloud);
	//����Ŀ�����
	ndt.setInputTarget(target_cloud2);
	
	//������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud);
	//�˴�output_cloud������Ϊ���յ�Դ���Ʊ任����Ϊ�����Դ���ƽ������˲�����
	std::cout << "ndt has coverged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;

	//ʹ�ô����ı任��δ���˵ĵ��ƽ��б任
	pcl::transformPointCloud(*input_cloud2, *output_cloud, ndt.getFinalTransformation());

	int sizeofOutputCloud = output_cloud->size();
	std::cout << sizeofOutputCloud << std::endl;
	
	return 0;
}