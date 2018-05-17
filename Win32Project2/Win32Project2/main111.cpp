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
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/range_image/range_image.h>  //���ͼ��ͷ�ļ�
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include "AfxUtil.h"

//ƽ�Ƶ���
void getTransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double deltaX, double deltaY, double deltaZ)
{
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].x += deltaX;
		cloud->points[i].y += deltaY;
		cloud->points[i].z += deltaZ;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr getSampleCloudByVotex(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double leafSize)
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
pcl::PointCloud<pcl::Normal>::Ptr getCloudNormalFromCloudAndSearchRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, int numberOfNormalKDTreeNeighbor)
{
	//����������Ƶ����ĵ�
	double xResolution = 1.0;
	double yResolution = -1.0;
	Pt3 midPoint(0, 0, 0);
	double maxDistance = 0;
	double minZ = 0;
	double maxZ = 0;
	int xSize = 0;
	int ySize = 0;
	util::getPCLMidPointAndDistance(inputCloud, xResolution, yResolution, midPoint, maxDistance, minZ, maxZ, xSize, ySize);

	double midPointX = midPoint.x();
	double midPointY = midPoint.y();
	double midPointZ = midPoint.z();
	std::cout << "�������ĵ����꣺(" << midPointX << "," << midPointY << "," << midPointZ << ")" << std::endl;

	//������������ķ��õ�ԭ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
	tempCloud->clear();
	tempCloud->width = inputCloud->width;
	tempCloud->height = inputCloud->height;
	tempCloud->resize(tempCloud->width * tempCloud->height);
	for (size_t i = 0; i < inputCloud->size(); i++)
	{
		tempCloud->points[i].x = inputCloud->points[i].x - midPointX;
		tempCloud->points[i].y = inputCloud->points[i].y - midPointY;
		tempCloud->points[i].z = inputCloud->points[i].z - midPointZ;
	}

	//���㷨���������������Ǿֲ�����ϵ�ĸ��
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(tempCloud);
	//����һ���յ�kdtree���󣬲��������ݸ����߹��ƶ���
	//���ڸ������������ݼ���kdtree��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normalKDTree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(normalKDTree);
	//������ݼ�
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	cloud_normals->is_dense = true;
	//ʹ�ð뾶�ڲ�ѯ����Χ��������Ԫ�أ�����Ϊ��λ��
	//ne.setRadiusSearch(searchRadius);
	ne.setKSearch(numberOfNormalKDTreeNeighbor);
	//��������ֵ
	ne.compute(*cloud_normals);

	std::cout << "����ʸ��������" << cloud_normals->size() << std::endl;
	tempCloud->clear();
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

	std::cout << "cloud_sift����:" << cloud_sift->size() << std::endl;
	//pcl::PointWithScale��ʽ�ĵ���ת��Ϊpcl::PointXYZ��ʽ�ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_siftOutput->height = 1;
	cloud_siftOutput->width = cloud_sift->width;
	cloud_siftOutput->is_dense = true;
	cloud_siftOutput->resize(cloud_siftOutput->height * cloud_siftOutput->width);
	for (size_t i = 0; i < cloud_sift->size(); i++)
	{
		cloud_siftOutput->points[i].x = cloud_sift->at(i).x;
		cloud_siftOutput->points[i].y = cloud_sift->at(i).y;
		cloud_siftOutput->points[i].z = cloud_sift->at(i).z;
	}
	cloud_sift->clear();
	cloudWithIntensity->clear();

	return cloud_siftOutput;
}

//����sift�ؼ��㣬���Ʒ��ߺ�������ƣ��õ�pfh��������
pcl::PointCloud<pcl::PFHSignature125>::Ptr getPFHFromSiftAndNormalsAndCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput,
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	int numberOfNearestNeighbor)
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

//�������ͼ
pcl::PointCloud<pcl::Narf36>::Ptr getNarfFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float supportSize)
{
	Pt3 midPoint(0, 0, 0);
	double maxDistance = 0;
	double xResolution = 1.0;
	double yResolution = -1.0;
	double minZ = 0;
	double maxZ = 0;
	int xSize = 0;
	int ySize = 0;
	util::getPCLMidPointAndDistance(inputCloud, xResolution, yResolution, midPoint, maxDistance, minZ, maxZ, xSize, ySize);

	double midZ = midPoint.z() + maxDistance;
	Pt3 cameraPos(midPoint.x(), midPoint.y(), midZ);

	//����(1)
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));
	//����(360)
	float maxAngleWidth = angularResolution * 360.0f;
	//����(180)
	float maxAngleHeight = angularResolution * 180.0f;
	//�ɼ�λ��
	Eigen::Affine3f sensorPose = (Eigen::Affine3f) Eigen::Translation3f(cameraPos.x(), cameraPos.y(), cameraPos.z());
	//���ͼ����ѭ������ϵͳ
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0;
	float minRange = 0;
	int borderSize = 1;
	//�γ����ͼ
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*inputCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << rangeImage << std::endl;

	//��ȡ���ͼ�߽�
	pcl::RangeImageBorderExtractor border_extractor(&rangeImage);
	pcl::PointCloud<pcl::BorderDescription> border_descritpions;
	border_extractor.compute(border_descritpions);

	//��ȡNARF�ؼ��� 
	pcl::NarfKeypoint narf_keypoint_detector(&border_extractor);
	narf_keypoint_detector.setRangeImage(&rangeImage);
	narf_keypoint_detector.getParameters().support_size = supportSize;  //����������ý��ڵ�������

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);

	//Ϊ��������ȡnarf������
	std::vector<int> keyPoint_indices2;
	keyPoint_indices2.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoint_indices.size(); i++)
	{
		keyPoint_indices2[i] = keypoint_indices.points[i];
	}
	pcl::NarfDescriptor narf_descriptor(&rangeImage, &keyPoint_indices2);
	narf_descriptor.getParameters().support_size = supportSize;
	narf_descriptor.getParameters().rotation_invariant = true; //rotation_invariant

	pcl::PointCloud<pcl::Narf36>::Ptr cloud_narfDescriptor(new pcl::PointCloud<pcl::Narf36>);
	narf_descriptor.compute(*cloud_narfDescriptor);

	return cloud_narfDescriptor;

}

int main()
{

	//std::string strInputFileName1 = "E:\\xyz.pcd";
	std::string strInputFileName1 = "E:\\xyz.pcd";
	//std::string strInputFileName2 = "E:\\xyz_translation_del_building.pcd";
	//std::string strInputFileName1 = "E:\\DEM-2013.pcd";
	//std::string strInputFileName2 = "E:\\DEM-2016.pcd";
	int numberOfCloudSample = 10000;  // ���Ʋ�������
	double deltaX = 1000.0;			  //x����ƫ��
	double deltaY = 1000.0;				  //y����ƫ��
	double deltaZ = 1000.0;
	//SIFT�ؼ���
	double min_scale = 0.9;
	int nr_octaves = 11;     //����
	int nr_scales_per_octave = 18; //���ڲ���
	double min_contrast = 0.02;
	//���ڸ���
	//���߹���kdtreeѰ�Ұ뾶
	//double normalKDTree_SearchRadius = 20;
	//���߹���kdtreeѰ�Ҹ���
	int numberOfNormalKDTreeNeighbor = 7;
	//PFHѰ�Ұ뾶��������ڷ��߹���Ѱ�Ұ뾶
	//double pfhKDTree_SearchRadius = 100;

	int numberOfNearestNeighbor = 4;

	//NARF supportSize
	int narfSupportSize = numberOfCloudSample;
	
	/*
	//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strInputFileName1, *cloud_in1) == -1)
	{
		return -1;
	}
	//����ϡ�軯
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
	pcl::PointCloud<pcl::Narf36>::Ptr cloud_Narf1 = getNarfFromPointCloud(cloud_sampled1, narfSupportSize);
	/*
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled1, numberOfNormalKDTreeNeighbor);
	FILE *fp_cloud_normals1;
	fp_cloud_normals1 = fopen("E:\\test\\fp_cloud_normals1.txt", "w");
	for (size_t i = 0; i < cloud_normals1->size(); i++)
	{
		fprintf(fp_cloud_normals1, " <%f,%f,%f>, ", cloud_normals1->at(i).normal_x, cloud_normals1->at(i).normal_y, cloud_normals1->at(i).normal_z);
	}
	fclose(fp_cloud_normals1);

	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput1 = getSiftFromCloudAndParameter(cloud_sampled1, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);

	//ȥ��sift�ؼ�����ظ�ֵ��
	std::string strcloud_siftOutput1 = "E:\\test\\cloud_siftOutput1.pcd";
	pcl::io::savePCDFile(strcloud_siftOutput1, *cloud_siftOutput1);

	//pfh������������ȡ
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH1 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled1,
		cloud_siftOutput1,
		//cloud_sift1,
		cloud_normals1,
		numberOfNearestNeighbor);
	//pfhKDTree_SearchRadius);

	FILE *fp_cloud_PFH1;
	fp_cloud_PFH1 = fopen("E:\\test\\fp_cloud_PFH1.txt", "w");
	for (size_t i = 0; i < cloud_PFH1->size(); i++)
	{
		fprintf(fp_cloud_PFH1, "{\n");
		for (size_t j = 0; j < 125; j++)
		{
			fprintf(fp_cloud_PFH1, " %f, ", cloud_PFH1->at(i).histogram[j]);
			if (j % 10 == 0)
			{
				fprintf(fp_cloud_PFH1, "\n");
			}
		}
		fprintf(fp_cloud_PFH1, "{\n");
	}
	fclose(fp_cloud_PFH1);

	
	//�������
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	//���������
	if (pcl::io::loadPCDFile(strInputFileName1, *cloud_in2) == -1)
	{
		return -1;
	}

	//����ϡ�軯
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled2 = getSampleCloud(cloud_in2, numberOfCloudSample);
	getTransformPointCloud(cloud_sampled2, deltaX, deltaY, deltaZ);
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

	pcl::PointCloud<pcl::Narf36>::Ptr cloud_Narf2 = getNarfFromPointCloud(cloud_sampled2, narfSupportSize);
	/*
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled2, numberOfNormalKDTreeNeighbor);
	FILE *fp_cloud_normals2;
	fp_cloud_normals2 = fopen("E:\\test\\fp_cloud_normals2.txt", "w");
	for (size_t i = 0; i < cloud_normals2->size(); i++)
	{
		fprintf(fp_cloud_normals2, " <%f,%f,%f>, ", cloud_normals2->at(i).normal_x, cloud_normals2->at(i).normal_y, cloud_normals2->at(i).normal_z);
	}
	fclose(fp_cloud_normals2);

	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput2 = getSiftFromCloudAndParameter(cloud_sampled2, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);
	std::string strcloud_siftOutput2 = "E:\\test\\cloud_siftOutput2.pcd";
	pcl::io::savePCDFile(strcloud_siftOutput2, *cloud_siftOutput2);
	//pfh������������ȡ
	pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_PFH2 = getPFHFromSiftAndNormalsAndCloud(cloud_sampled2,
		//cloud_sift2,
		cloud_siftOutput2,
		//cloud_normals2,
		cloud_normals1,
		numberOfNearestNeighbor);
	//pfhKDTree_SearchRadius);
	FILE *fp_cloud_PFH2;
	fp_cloud_PFH2 = fopen("E:\\test\\fp_cloud_PFH2.txt", "w");
	for (size_t i = 0; i < cloud_PFH2->size(); i++)
	{
		fprintf(fp_cloud_PFH2, "{\n");
		for (size_t j = 0; j < 125; j++)
		{
			fprintf(fp_cloud_PFH2, " %f, ", cloud_PFH2->at(i).histogram[j]);
			if (j % 10 == 0)
			{
				fprintf(fp_cloud_PFH2, "\n");
			}
		}
		fprintf(fp_cloud_PFH2, "{\n");
	}
	fclose(fp_cloud_PFH2);
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
		int firstID = correspondences->at(i).index_query;
		int secondID = correspondences->at(i).index_match;
		double distance = correspondences->at(i).distance;
		double weight = correspondences->at(i).weight;
		double pointX1 = cloud_sampled1->points[firstID].x;
		double pointY1 = cloud_sampled1->points[firstID].y;
		double pointZ1 = cloud_sampled1->points[firstID].z;
		double pointX2 = cloud_sampled2->points[secondID].x;
		//float pointX2 = cloud_sampled2->points[secondID].x;
		double pointY2 = cloud_sampled2->points[secondID].y;
		//float pointY2 = cloud_sampled2->points[secondID].y ;
		double pointZ2 = cloud_sampled2->points[secondID].z;
		double dx = pointX2 - pointX1;
		double dy = pointY2 - pointY1;
		double dz = pointZ2 - pointZ1;

		fprintf(fp_correspondences, "(<%d,%d>,%f,%f,{<%f,%f,%f>})\n",
			firstID, secondID, correspondences->at(i).distance, weight,
			dx, dy, dz);

		if (i % 10 == 0)
		{
			std::cout << std::endl;
		}
	}
	fclose(fp_correspondences);

	//��RANSAC�����ڵ�
	double epsilon_sac = 1.0;
	int iter_sac = 1000000;
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);
	sac.setInputSource(cloud_siftOutput1);
	sac.setInputTarget(cloud_siftOutput2);
	sac.setInlierThreshold(epsilon_sac);
	sac.setInputCorrespondences(correspondences);
	sac.getRemainingCorrespondences(*correspondences, *cor_inliers_ptr);
	std::cout << "cor_inliers_ptr����:" << cor_inliers_ptr->size() << std::endl;
	FILE *fp_cor_inliers_ptr;
	fp_cor_inliers_ptr = fopen("E:\\test\\fp_cor_inliers_ptr.txt", "w");
	for (size_t i = 0; i < cor_inliers_ptr->size(); i++)
	{
		int firstID = cor_inliers_ptr->at(i).index_query;
		int secondID = cor_inliers_ptr->at(i).index_match;
		double distance = cor_inliers_ptr->at(i).distance;
		double weight = cor_inliers_ptr->at(i).weight;
		double pointX1 = cloud_sampled1->points[firstID].x;
		double pointY1 = cloud_sampled1->points[firstID].y;
		double pointZ1 = cloud_sampled1->points[firstID].z;
		double pointX2 = cloud_sampled2->points[secondID].x;
		//float pointX2 = cloud_sampled2->points[secondID].x;
		double pointY2 = cloud_sampled2->points[secondID].y;
		//float pointY2 = cloud_sampled2->points[secondID].y ;
		double pointZ2 = cloud_sampled2->points[secondID].z;
		double dx = pointX2 - pointX1;
		double dy = pointY2 - pointY1;
		double dz = pointZ2 - pointZ1;

		fprintf(fp_cor_inliers_ptr, "(<%d,%d>,%f,%f,{<%f,%f,%f>})\n",
			firstID, secondID, cor_inliers_ptr->at(i).distance, weight,
			dx, dy, dz);

		if (i % 10 == 0)
		{
			std::cout << std::endl;
		}
	}
	fclose(fp_cor_inliers_ptr);

	//����任����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Rawת��ǰ������" << cloud_in_filtered_Raw->size() << std::endl;
	//Eigen::Matrix4f transformation_matrix = sac.getBestTransformation();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_east;
	Eigen::Matrix4f transformation_matrix;
	trans_east.estimateRigidTransformation(*cloud_siftOutput1, *cloud_siftOutput2, *cor_inliers_ptr, transformation_matrix);
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

	//��ֵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr diffCloud(new pcl::PointCloud<pcl::PointXYZ>);
	diffCloud->clear();
	diffCloud->height = 1;
	diffCloud->width = 10000;
	diffCloud->resize(diffCloud->width * diffCloud->height);
	for (size_t i = 0; i < diffCloud->size(); i++)
	{
		diffCloud->points[i].x = finalPointCloud->points[i].x - cloud_sampled2->points[i].x;
		diffCloud->points[i].y = finalPointCloud->points[i].y - cloud_sampled2->points[i].y;
		diffCloud->points[i].z = finalPointCloud->points[i].z - cloud_sampled2->points[i].z;

	}

	std::string strDiffCloud = "E:\\test\\diffCloud.pcd";
	pcl::io::savePCDFile(strDiffCloud, *diffCloud);

	cloud_normals1->clear();
	cloud_sampled1->clear();
	cloud_siftOutput1->clear();
	cloud_PFH1->clear();
	cloud_normals2->clear();
	cloud_sampled2->clear();
	cloud_siftOutput2->clear();
	cloud_PFH2->clear();
	cloud_in_filtered_Raw->clear();
	*/
	cloud_Narf1->clear();
	cloud_Narf2->clear();
	return 0;

}