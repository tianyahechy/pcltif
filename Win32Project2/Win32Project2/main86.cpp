#pragma once 
#include "common.h"
#include <pcl/correspondence.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
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
	cloud_sampled->is_dense = true;
	cloud_sampled->points.resize(cloud_sampled->width * cloud_sampled->height);
	for (size_t i = 0; i < cloud_sampled->points.size(); i++)
	{
		cloud_sampled->points[i].x = inputCloud->points[i].x;
		cloud_sampled->points[i].y = inputCloud->points[i].y;
		cloud_sampled->points[i].z = inputCloud->points[i].z;
	}

	return cloud_sampled;
}

//����votex����ϡ�軯
pcl::PointCloud<pcl::PointXYZ>::Ptr getSampleCloudByVotex(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double leafSize )
{
	//����ϡ�軯
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_sampled->is_dense = true;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(inputCloud);
	vg.setLeafSize(leafSize, leafSize, leafSize);
	vg.filter(*cloud_sampled);

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
	cloud_normals->is_dense = true;
	//ʹ�ð뾶�ڲ�ѯ����Χ��������Ԫ�أ�����Ϊ��λ��
	ne.setRadiusSearch(searchRadius);
	//��������ֵ
	ne.compute(*cloud_normals);

	std::cout << "����ʸ��������" << cloud_normals->size() << std::endl;

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
	cloud_sift->is_dense = true;
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
	cloud_siftOutput->is_dense = true;
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
	//double searchRadius)
{
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_pfhOutput(new pcl::PointCloud<pcl::PFHSignature125>);
	cloud_pfhOutput->is_dense = true;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr pfhKDTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setSearchMethod(pfhKDTree);
	pfh.setKSearch(numberOfNearestNeighbor);
	pfh.setInputCloud(cloud_siftOutput);
	pfh.setInputNormals(cloud_normals);
	pfh.setSearchSurface(cloud_sampled);
	//pfh.setRadiusSearch(searchRadius);
	pfh.compute(*cloud_pfhOutput);

	return cloud_pfhOutput;

}

int main()
{

	//std::string strInputFileName1 = "E:\\xyz.pcd";
	///std::string strInputFileName1 = "E:\\xyz.pcd";
	//std::string strInputFileName2 = "E:\\xyz_translation_del_building.pcd";
	std::string strInputFileName1 = "E:\\DEM-2013.pcd";
	std::string strInputFileName2 = "E:\\DEM-2013.pcd";
	int numberOfCloudSample = 100000;  // ���Ʋ�������
	double deltaX = -432000;				//x����ƫ��
	double deltaY = -4374000;				//x����ƫ��
	//SIFT�ؼ���
	float min_scale = 0.5;
	int nr_octaves = 16;     //����
	int nr_scales_per_octave = 64; //���ڲ���
	float min_contrast = 0.02;
	//���ڸ���
	int numberOfNearestNeighbor = 10;
	//���߹���kdtreeѰ�Ұ뾶
	double normalKDTree_SearchRadius = 20;
	//PFHѰ�Ұ뾶��������ڷ��߹���Ѱ�Ұ뾶
	double pfhKDTree_SearchRadius = 100;

	//�������
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strInputFileName1, *cloud_in1) == -1)
	{
		return -1;
	}
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	//getTransformPointCloud(cloud_in1, deltaX, deltaY);
	//����ϡ�軯
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled1 = getSampleCloudByVotex(cloud_in1, leafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled1 = getSampleCloud(cloud_in1, numberOfCloudSample);
	std::string strCloud_sampled1 = "E:\\test\\cloud_sampled1.pcd";
	pcl::io::savePCDFile(strCloud_sampled1, *cloud_sampled1);
	//�ͷ�ԭʼ����
	cloud_in1->clear();
	*/
	std::string strCloud_sampled1 = "E:\\test\\cloud_sampled1.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled1(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strCloud_sampled1, *cloud_sampled1) == -1)
	{
		return -1;
	}
	
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled1, normalKDTree_SearchRadius);
	FILE *fp_cloud_normals1;
	fp_cloud_normals1 = fopen("E:\\test\\fp_cloud_normals1.txt", "w");
	for (size_t i = 0; i < cloud_normals1->size(); i++)
	{
		fprintf(fp_cloud_normals1, " <%f,%f,%f>, ", cloud_normals1->at(i).normal_x, cloud_normals1->at(i).normal_y, cloud_normals1->at(i).normal_z );
	}
	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput1 = getSiftFromCloudAndParameter(cloud_sampled1, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);

	std::string strcloud_siftOutput1 = "E:\\test\\cloud_siftOutput1.pcd";
	pcl::io::savePCDFile(strcloud_siftOutput1, *cloud_siftOutput1);
	//pfh������������ȡ
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH1 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled1,
		cloud_siftOutput1,
		cloud_normals1,
		numberOfNearestNeighbor);
		//pfhKDTree_SearchRadius);

	FILE *fp_cloud_PFH1;
	fp_cloud_PFH1 = fopen("E:\\test\\fp_cloud_PFH1.txt", "w");
	for (size_t i = 0; i < cloud_PFH1->size(); i++)
	{
		fprintf(fp_cloud_PFH1, " <%d>, ", cloud_PFH1->at(i).descriptorSize());
		/*
		fprintf(fp_cloud_PFH1, "{ \n ");
		for (size_t j = 0; j < 125; j++)
		{
			fprintf(fp_cloud_PFH1, " <%f>, ", cloud_PFH1->at(i).histogram[j]);
		}
		fprintf(fp_cloud_PFH1, "}\n ");
		*/
	}
	
	//�������
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strInputFileName2, *cloud_in2) == -1)
	{
		return -1;
	}
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	getTransformPointCloud(cloud_in2, deltaX, deltaY);
	//����ϡ�軯
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled2 = getSampleCloudByVotex(cloud_in2, leafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled2 = getSampleCloud(cloud_in2, numberOfCloudSample);
	std::string strcloud_sampled2 = "E:\\test\\cloud_sampled2.pcd";
	pcl::io::savePCDFile(strcloud_sampled2, *cloud_sampled2);
	//�ͷ�ԭʼ����
	cloud_in2->clear();
	*/
	std::string strCloud_sampled2 = "E:\\test\\cloud_sampled2.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled2(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strCloud_sampled2, *cloud_sampled2) == -1)
	{
		return -1;
	}
	
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled2, normalKDTree_SearchRadius);
	FILE *fp_cloud_normals2;
	fp_cloud_normals2 = fopen("E:\\test\\fp_cloud_normals2.txt", "w");
	for (size_t i = 0; i < cloud_normals2->size(); i++)
	{
		fprintf(fp_cloud_normals2, " <%f,%f,%f>, ", cloud_normals2->at(i).normal_x, cloud_normals2->at(i).normal_y, cloud_normals2->at(i).normal_z);
	}
	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput2 = getSiftFromCloudAndParameter(cloud_sampled2, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);

	std::string strcloud_siftOutput2 = "E:\\test\\cloud_siftOutput2.pcd";
	pcl::io::savePCDFile(strcloud_siftOutput2, *cloud_siftOutput2);
	//pfh������������ȡ
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH2 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled2,
		cloud_siftOutput2,
		cloud_normals2,
		numberOfNearestNeighbor);
		//pfhKDTree_SearchRadius);
	FILE *fp_cloud_PFH2;
	fp_cloud_PFH2 = fopen("E:\\test\\fp_cloud_PFH2.txt", "w");
	for (size_t i = 0; i < cloud_PFH2->size(); i++)
	{
		fprintf(fp_cloud_PFH2, " <%d>, ", cloud_PFH2->at(i).descriptorSize());
	}

	//������׼
	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> cens;
	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	cens.setInputSource(cloud_PFH1);
	cens.setInputTarget(cloud_PFH2);
	cens.determineReciprocalCorrespondences(*correspondences); //˫��һ���Ե�ƥ����

	std::cout << "correspondences����:" << correspondences->size() << std::endl;
	FILE *fp_correspondences;
	fp_correspondences = fopen("E:\\test\\fp_correspondences.txt", "w");
	for (size_t i = 0; i < correspondences->size(); i++)
	{
		fprintf(fp_correspondences, "<%d,%d>", correspondences->at(i).index_match, correspondences->at(i).index_query);
		if (i % 10 == 0)
		{
			std::cout << std::endl;
		}
	}
	FILE *fp_sac;
	fp_sac = fopen("E:\\test\\fp_sac.txt", "w");
	for (size_t i = -100; i < 100; i++)
	{
		//SAC�޳��ֲ�
		double epsilon_sac = pow(10,i);//2.5;
		int iter_sac = 1000000;
		//����RANSAC����ȷ���ڲ��� 

		pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
		//sac.setInputCloud(cloud_siftOutput1);
		sac.setInputSource(cloud_siftOutput1);
		sac.setInputTarget(cloud_siftOutput2);
		//sac.setTargetCloud(cloud_siftOutput2);
		sac.setInlierThreshold(epsilon_sac);
		sac.setMaxIterations(iter_sac);
		sac.setInputCorrespondences(correspondences);
		boost::shared_ptr<pcl::Correspondences> cor_inliners_ptr(new pcl::Correspondences);
		sac.getRemainingCorrespondences(*correspondences, *cor_inliners_ptr);

		Eigen::Matrix4f transformation_matrix = sac.getBestTransformation();
		if (!transformation_matrix.isIdentity())
		{
			fprintf(fp_sac, "<%d, %f>", i, epsilon_sac);
			
		}
	}
	fclose(fp_sac);
	
	/*
	FILE *fp_cor_inliners_ptr;
	fp_cor_inliners_ptr = fopen("E:\\test\\fp_cor_inliners_ptr.txt", "w");
	for (size_t i = 0; i < cor_inliners_ptr->size(); i++)
	{
		fprintf(fp_cor_inliners_ptr, "<%d,%d>", cor_inliners_ptr->at(i).index_match, cor_inliners_ptr->at(i).index_query );
		if ( i % 10 == 0 )
		{
			std::cout << std::endl;
		}
	}

	//����任����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Rawת��ǰ������" << cloud_in_filtered_Raw->size() << std::endl;
	//pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_east;
	//Eigen::Matrix4f transformation_matrix;
	//trans_east.estimateRigidTransformation(*cloud_siftOutput1, *cloud_siftOutput2, *correspondences, transformation_matrix);
	pcl::transformPointCloud(*cloud_sampled1, *cloud_in_filtered_Raw, transformation_matrix);
	std::cout << "�任����:" << std::endl 
		<< transformation_matrix << std::endl;
	std::cout << "cloud_in_filtered_Rawת���������" << cloud_in_filtered_Raw->size() << std::endl;

	std::string strcloud_in_filtered_Raw = "E:\\test\\cloud_in_filtered_Raw.pcd";
	pcl::io::savePCDFile(strcloud_in_filtered_Raw, *cloud_in_filtered_Raw);
	//׼��ICP����׼
	//��ʹ��֮ǰҪ����set���������� 
	//1,setMaximumIterations�� ������������icp��һ�������ķ�������������Щ�Σ�

	//2,setTransformationEpsilon�� ǰһ���任����͵�ǰ�任����Ĳ���С����ֵʱ������Ϊ�Ѿ������ˣ���һ������������

	//3,setEuclideanFitnessEpsilon�� ����һ�����������Ǿ�������С����ֵ�� ֹͣ������

	double correspondenceDistance = 0.5;
	int ICPmaximumIterations = 200;
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

	std::string strFinalPointCloud = "E:\\test\\finalPointCloud.pcd";
	pcl::io::savePCDFile(strFinalPointCloud, *finalPointCloud);
	*/
	cloud_normals1->clear();
	cloud_sampled1->clear();
	cloud_siftOutput1->clear();
	cloud_PFH1->clear();
	cloud_normals2->clear();
	cloud_sampled2->clear();
	cloud_siftOutput2->clear();
	cloud_PFH2->clear();
	//cloud_in_filtered_Raw->clear();

	return 0;

}