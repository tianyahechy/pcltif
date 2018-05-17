#pragma once 
#include "common.h"
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/sample_consensus/ransac.h>

//ƽ�Ƶ���

void getTransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double deltaX, double deltaY )
{
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].x += deltaX;
		cloud->points[i].y += deltaY;
	}

}
//���ݲ�����������
pcl::PointCloud<pcl::PointXYZ>::Ptr getSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, int numberOfCloudSample)
{
	//����ϡ�軯
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
//���ݵ��ƺ������뾶���ظõ��ƵĹ��Ʒ���
pcl::PointCloud<pcl::Normal>::Ptr getCloudNormalFromCloudAndSearchRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double searchRadius)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(inputCloud);
	//����һ���յ�kdtree���󣬲��������ݸ����߹��ƶ���
	//���ڸ������������ݼ���kdtree��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normalKDTree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(normalKDTree);
	//������ݼ�
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//ʹ�ð뾶�ڲ�ѯ����Χ��������Ԫ�أ�����Ϊ��λ��
	ne.setRadiusSearch(searchRadius);
	//��������ֵ
	ne.compute(*cloud_normals);

	return cloud_normals;
}

//����������ƺ�sift�������õ�sift�ؼ���
pcl::PointCloud<pcl::PointXYZ>::Ptr getSiftFromCloudAndParameter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
	float min_scale,
	int nr_octaves,
	int nr_scales_per_octave,
	float min_contrast)
{

	//��pcl::PointCloud<pcl::PointXYZ>ת��pcl::PointCloud<pcl::PointXYZI>
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

	std::cout << "cloud_sift����:" <<cloud_sift->size() <<std::endl;
	//pcl::PointWithScale��ʽ�ĵ���ת��Ϊpcl::PointXYZ��ʽ�ĵ���
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

//����sift�ؼ��㣬���Ʒ��ߺ�������ƣ��õ�pfh��������
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
	int numberOfCloudSample = 1000000;  // ���Ʋ�������
	double deltaX = -432000;				//x����ƫ��
	double deltaY = -4374000;				//x����ƫ��
	//SIFT�ؼ���
	float min_scale = 10;
	int nr_octaves = 6;
	int nr_scales_per_octave = 4;
	float min_contrast = 0;
	//���ڸ���
	int numberOfNearestNeighbor = 1000;
	//���߹���kdtreeѰ�Ұ뾶
	double normalKDTree_SearchRadius = 0.03;

	//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strInputFileName1, *cloud_in1) == -1)
	{
		return -1;
	}
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	getTransformPointCloud(cloud_in1, deltaX, deltaY);
	//����ϡ�軯
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled1 = getSampleCloud(cloud_in1, numberOfCloudSample);
	//�ͷ�ԭʼ����
	cloud_in1->clear();
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled1, normalKDTree_SearchRadius);
	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput1 = getSiftFromCloudAndParameter(cloud_sampled1, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);
	//pfh������������ȡ
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH1 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled1,
		cloud_siftOutput1,
		cloud_normals1,
		numberOfNearestNeighbor);

	//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strInputFileName2, *cloud_in2) == -1)
	{
		return -1;
	}
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	getTransformPointCloud(cloud_in2, deltaX, deltaY);
	//����ϡ�軯
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled2 = getSampleCloud(cloud_in2, numberOfCloudSample);
	//�ͷ�ԭʼ����
	cloud_in2->clear();
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled2, normalKDTree_SearchRadius);
	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput2 = getSiftFromCloudAndParameter(cloud_sampled2, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);
	//pfh������������ȡ
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH2 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled2,
		cloud_siftOutput2,
		cloud_normals2,
		numberOfNearestNeighbor);

	//������׼
	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> cens;
	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	cens.setInputSource(cloud_PFH1);
	cens.setInputTarget(cloud_PFH2);
	cens.determineReciprocalCorrespondences(*correspondences); //˫��һ���Ե�ƥ����
	std::cout << "correspondences����:" << correspondences->size() << std::endl;

	//SAC�޳��ֲ�
	double epsilon_sac = 10000;
	int iter_sac = 1500;
	//����RANSAC����ȷ���ڲ��� 
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
	sac.setInputCloud(cloud_siftOutput1);
	sac.setInputTarget(cloud_siftOutput2);
	//sac.setInputSource(cloud_siftOutput1);
	//sac.setTargetCloud(cloud_siftOutput2);
	sac.setInlierThreshold(epsilon_sac);
	sac.setMaxIterations(iter_sac);
	sac.setInputCorrespondences(correspondences);
	

	//����任����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	cloud_in_filtered_Raw->clear();
	Eigen::Matrix4f transformation_matrix = sac.getBestTransformation();
	pcl::transformPointCloud(*cloud_sampled1, *cloud_in_filtered_Raw, transformation_matrix);

	//׼��ICP����׼
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