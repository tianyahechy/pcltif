#pragma once 
#include "common.h"
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/sample_consensus/ransac.h>

//平移点云

void getTransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double deltaX, double deltaY )
{
	//平移放缩点云,查找特征点匹配结果不对的问题
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].x += deltaX;
		cloud->points[i].y += deltaY;
	}

}
//根据采样个数采样
pcl::PointCloud<pcl::PointXYZ>::Ptr getSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, int numberOfCloudSample)
{
	//点云稀疏化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_sampled->height = 1;
	cloud_sampled->width = numberOfCloudSample;
	cloud_sampled->points.resize(cloud_sampled->width * cloud_sampled->height);
	for (size_t i = 0; i < cloud_sampled->points.size(); i++)
	{
		cloud_sampled->points[i].x = inputCloud->points[i].x;
		cloud_sampled->points[i].y = inputCloud->points[i].y;
		cloud_sampled->points[i].z = inputCloud->points[i].z;
	}
	
	return cloud_sampled;
}
//根据点云和搜索半径返回该点云的估计法线
pcl::PointCloud<pcl::Normal>::Ptr getCloudNormalFromCloudAndSearchRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double searchRadius)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(inputCloud);
	//创建一个空的kdtree对象，并把它传递给法线估计对象
	//基于给出的输入数据集，kdtree将被建立
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normalKDTree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(normalKDTree);
	//输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//使用半径在查询点周围的所有邻元素（以米为单位）
	ne.setRadiusSearch(searchRadius);
	//计算特征值
	ne.compute(*cloud_normals);

	return cloud_normals;
}

//根据输入点云和sift参数，得到sift关键点
pcl::PointCloud<pcl::PointXYZ>::Ptr getSiftFromCloudAndParameter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
	float min_scale,
	int nr_octaves,
	int nr_scales_per_octave,
	float min_contrast)
{

	//将pcl::PointCloud<pcl::PointXYZ>转成pcl::PointCloud<pcl::PointXYZI>
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudWithIntensity(new pcl::PointCloud<pcl::PointXYZI>);
	cloudWithIntensity->height = 1;
	cloudWithIntensity->width = inputCloud->width;
	cloudWithIntensity->resize(cloudWithIntensity->height * cloudWithIntensity->width);
	for (size_t i = 0; i < inputCloud->size(); i++)
	{
		cloudWithIntensity->points[i].x = inputCloud->points[i].x;
		cloudWithIntensity->points[i].y = inputCloud->points[i].y;
		cloudWithIntensity->points[i].z = inputCloud->points[i].z;
		cloudWithIntensity->points[i].intensity = inputCloud->points[i].z;
	}

	pcl::PointCloud<pcl::PointWithScale>::Ptr cloud_sift(new pcl::PointCloud<pcl::PointWithScale>);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr siftKDTree(new pcl::search::KdTree<pcl::PointXYZI>());

	pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
	sift.setSearchMethod(siftKDTree);
	sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloudWithIntensity);
	sift.compute(*cloud_sift);

	std::cout << "cloud_sift个数:" <<cloud_sift->size() <<std::endl;
	//pcl::PointWithScale格式的点云转化为pcl::PointXYZ格式的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_siftOutput->height = 1;
	cloud_siftOutput->width = cloud_sift->width;
	cloud_siftOutput->resize(cloud_siftOutput->height * cloud_siftOutput->width);
	for (size_t i = 0; i < cloud_sift->size(); i++)
	{
		double x = cloud_sift->at(i).x;
		double y = cloud_sift->at(i).y;
		double z = cloud_sift->at(i).z;

		cloud_siftOutput->points[i].x = x;
		cloud_siftOutput->points[i].y = y;
		cloud_siftOutput->points[i].z = z;

	}
	cloud_sift->clear();
	cloudWithIntensity->clear();

	return cloud_siftOutput;
}

//根据sift关键点，估计法线和输入点云，得到pfh特征描述
pcl::PointCloud<pcl::PFHSignature125>::Ptr getPFHFromSiftAndNormalsAndCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput,
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	int numberOfNearestNeighbor )
{
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_pfhOutput(new pcl::PointCloud<pcl::PFHSignature125>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr pfhKDTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setSearchMethod(pfhKDTree);
	pfh.setKSearch(numberOfNearestNeighbor);
	pfh.setInputCloud(cloud_siftOutput);
	pfh.setInputNormals(cloud_normals);
	pfh.setSearchSurface(cloud_sampled);
	pfh.compute(*cloud_pfhOutput);

	return cloud_pfhOutput;

}

int main()
{

	std::string strInputFileName1 = "E:\\DEM-2013.pcd";
	std::string strInputFileName2 = "E:\\DEM-2016.pcd";
	//std::string strInputFileName1 = "E:\\room_scan1.pcd";
	//std::string strInputFileName2 = "E:\\room_scan2.pcd";
	int numberOfCloudSample = 1000000;  // 点云采样数量
	double deltaX = -432000;				//x方向偏移
	double deltaY = -4374000;				//x方向偏移
	//SIFT关键点
	float min_scale = 10;
	int nr_octaves = 6;
	int nr_scales_per_octave = 4;
	float min_contrast = 0;
	//近邻个数
	int numberOfNearestNeighbor = 1000;
	//法线估计kdtree寻找半径
	double normalKDTree_SearchRadius = 0.03;

	//输入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	//打开输入点云
	if (pcl::io::loadPCDFile(strInputFileName1, *cloud_in1) == -1)
	{
		return -1;
	}
	//平移放缩点云,查找特征点匹配结果不对的问题
	getTransformPointCloud(cloud_in1, deltaX, deltaY);
	//点云稀疏化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled1 = getSampleCloud(cloud_in1, numberOfCloudSample);
	//释放原始点云
	cloud_in1->clear();
	//估计法线
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled1, normalKDTree_SearchRadius);
	//sift关键点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput1 = getSiftFromCloudAndParameter(cloud_sampled1, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);
	//pfh特征描述子提取
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH1 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled1,
		cloud_siftOutput1,
		cloud_normals1,
		numberOfNearestNeighbor);

	//输入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	//打开输入点云
	if (pcl::io::loadPCDFile(strInputFileName2, *cloud_in2) == -1)
	{
		return -1;
	}
	//平移放缩点云,查找特征点匹配结果不对的问题
	getTransformPointCloud(cloud_in2, deltaX, deltaY);
	//点云稀疏化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled2 = getSampleCloud(cloud_in2, numberOfCloudSample);
	//释放原始点云
	cloud_in2->clear();
	//估计法线
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled2, normalKDTree_SearchRadius);
	//sift关键点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput2 = getSiftFromCloudAndParameter(cloud_sampled2, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);
	//pfh特征描述子提取
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH2 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled2,
		cloud_siftOutput2,
		cloud_normals2,
		numberOfNearestNeighbor);

	//点云配准
	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> cens;
	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	cens.setInputSource(cloud_PFH1);
	cens.setInputTarget(cloud_PFH2);
	cens.determineReciprocalCorrespondences(*correspondences); //双向一致性的匹配点对
	std::cout << "correspondences个数:" << correspondences->size() << std::endl;

	//SAC剔除粗差
	double epsilon_sac = 10000;
	int iter_sac = 1500;
	//基于RANSAC方法确定内部点 
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
	sac.setInputCloud(cloud_siftOutput1);
	sac.setInputTarget(cloud_siftOutput2);
	//sac.setInputSource(cloud_siftOutput1);
	//sac.setTargetCloud(cloud_siftOutput2);
	sac.setInlierThreshold(epsilon_sac);
	sac.setMaxIterations(iter_sac);
	sac.setInputCorrespondences(correspondences);
	

	//计算变换矩阵
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	cloud_in_filtered_Raw->clear();
	Eigen::Matrix4f transformation_matrix = sac.getBestTransformation();
	pcl::transformPointCloud(*cloud_sampled1, *cloud_in_filtered_Raw, transformation_matrix);

	//准备ICP精配准
	double correspondenceDistance = 0.5;
	int ICPmaximumIterations = 20;
	double ICPTransformationEpsilon = 1e-6;
	double ICPEuclideanFitnessEpsilon = 0.5;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in_filtered_Raw);
	icp.setInputTarget(cloud_sampled2);
	icp.setMaxCorrespondenceDistance(correspondenceDistance);
	icp.setMaximumIterations(ICPmaximumIterations);
	icp.setTransformationEpsilon(ICPTransformationEpsilon);
	icp.setEuclideanFitnessEpsilon(ICPEuclideanFitnessEpsilon);
	icp.align(*finalPointCloud);

	cloud_normals1->clear();
	cloud_sampled1->clear();
	cloud_siftOutput1->clear();
	cloud_PFH1->clear();
	cloud_normals2->clear();
	cloud_sampled2->clear();
	cloud_siftOutput2->clear();
	cloud_PFH2->clear();
	cloud_in_filtered_Raw->clear();

	return 0;

}