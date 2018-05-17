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
#include <pcl/range_image/range_image.h>  //���ͼ��ͷ�ļ�
#include "AfxUtil.h"

//ƽ�Ƶ���

void getTransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float deltaX, float deltaY, float deltaZ)
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
	//����������Ƶ����ĵ�
	double xResolution = 1.0;
	double yResolution = -1.0;
	Pt3 midPoint(0, 0, 0);
	double maxDistance = 0;
	double minZ = 0;
	double maxZ = 0;
	int xSize = 0;
	int ySize = 0;
	util::getPCLMidPointAndDistance( inputCloud,  xResolution,  yResolution,  midPoint,  maxDistance,  minZ,  maxZ,  xSize,  ySize);

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
	ne.setRadiusSearch(searchRadius);
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

//ȥ��sift�ؼ�����ظ�ֵ��
pcl::PointCloud<pcl::PointXYZ>::Ptr getFinalSift(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
	std::list<pcl::PointXYZ> listCloudPoint;
	listCloudPoint.clear();
	std::list<pcl::PointXYZ>::iterator
		iterCurCloudPt = listCloudPoint.begin(),
		iterEndCloudPt = listCloudPoint.end();
	for (size_t i = 0; i < inputCloud->size(); i++)
	{
		pcl::PointXYZ thePt = inputCloud->points[i];
	
		float x = thePt.x;
		float y = thePt.y;
		float z = thePt.z;

		iterCurCloudPt = listCloudPoint.begin();
		iterEndCloudPt = listCloudPoint.end();
		for (; iterCurCloudPt != iterEndCloudPt; iterCurCloudPt++)
		{
			pcl::PointXYZ theListPoint = *iterCurCloudPt;
			float ptX = theListPoint.x;
			float ptY = theListPoint.y;
			float ptZ = theListPoint.z;
			if ( x == ptX && y == ptY && z == ptZ )
			{
				break;
			}

		}
		if ( iterCurCloudPt == iterEndCloudPt )
		{
			listCloudPoint.push_back(thePt);
		}
		
	}
	
	//listCloudPoint.sort();
	//listCloudPoint.unique();

	int filterSize = listCloudPoint.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>);
	filterCloud->clear();
	filterCloud->is_dense = true;
	filterCloud->height = 1;
	filterCloud->width = filterSize;
	//filterCloud->resize(filterCloud->width * filterCloud->height);
	std::list<pcl::PointXYZ>::iterator
		iterCurPoint = listCloudPoint.begin(),
		iterEndPoint = listCloudPoint.end();
	for (; iterCurPoint != iterEndPoint; iterCurPoint++)
	{
		pcl::PointXYZ thePt = *iterCurPoint;
		filterCloud->points.push_back(thePt);
	}

	listCloudPoint.clear();

	return filterCloud;
}
//�������ͼ
void createRangeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
	Pt3 midPoint(0, 0, 0);
	double maxDistance = 0;
	double xResolution = 1.0;
	double yResolution = -1.0;
	double minZ = 0;
	double maxZ = 0;
	int xSize = 0; 
	int ySize = 0;
	util::getPCLMidPointAndDistance(inputCloud,xResolution,yResolution, midPoint, maxDistance, minZ, maxZ, xSize, ySize);

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
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*inputCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << rangeImage << std::endl;
	
}

int main()
{

	//std::string strInputFileName1 = "E:\\xyz.pcd";
	std::string strInputFileName1 = "E:\\xyz.pcd";
	//std::string strInputFileName2 = "E:\\xyz_translation_del_building.pcd";
	//std::string strInputFileName1 = "E:\\DEM-2013.pcd";
	//std::string strInputFileName2 = "E:\\DEM-2013.pcd";
	int numberOfCloudSample = 10000;  // ���Ʋ�������
	float deltaX = 100.0;			  //x����ƫ��
	float deltaY = 80.0;				  //y����ƫ��
	float deltaZ = 100.0;
	//SIFT�ؼ���
	float min_scale = 0.9;
	int nr_octaves = 11;     //����
	int nr_scales_per_octave = 18; //���ڲ���
	float min_contrast = 0.02;
	//���ڸ���
	int numberOfNearestNeighbor = 4;//10,5,50,25,3,4,
	//���߹���kdtreeѰ�Ұ뾶
	double normalKDTree_SearchRadius = 20;
	//PFHѰ�Ұ뾶��������ڷ��߹���Ѱ�Ұ뾶
	double pfhKDTree_SearchRadius = 100;
	/*
	//�������
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
	//createRangeImageFromPointCloud(cloud_sampled1);
	
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled1, normalKDTree_SearchRadius);
	FILE *fp_cloud_normals1;
	fp_cloud_normals1 = fopen("E:\\test\\fp_cloud_normals1.txt", "w");
	for (size_t i = 0; i < cloud_normals1->size(); i++)
	{
		fprintf(fp_cloud_normals1, " <%f,%f,%f>, ", cloud_normals1->at(i).normal_x, cloud_normals1->at(i).normal_y, cloud_normals1->at(i).normal_z );
	}
	fclose(fp_cloud_normals1);
	
	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sift1 = getSiftFromCloudAndParameter(cloud_sampled1, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);

	//ȥ��sift�ؼ�����ظ�ֵ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput1 = getFinalSift(cloud_sift1);
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
	//if (pcl::io::loadPCDFile(strInputFileName2, *cloud_in2) == -1)
	if (pcl::io::loadPCDFile(strInputFileName1, *cloud_in2) == -1)
	{
		return -1;
	}
	//ƽ�Ʒ�������,����������ƥ�������Ե�����
	getTransformPointCloud(cloud_in2, deltaX, deltaY, deltaZ);
	
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
	
	//createRangeImageFromPointCloud(cloud_sampled2);
	
	//���Ʒ���
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 = getCloudNormalFromCloudAndSearchRadius(cloud_sampled2, normalKDTree_SearchRadius);
	FILE *fp_cloud_normals2;
	fp_cloud_normals2 = fopen("E:\\test\\fp_cloud_normals2.txt", "w");
	for (size_t i = 0; i < cloud_normals2->size(); i++)
	{
		fprintf(fp_cloud_normals2, " <%f,%f,%f>, ", cloud_normals2->at(i).normal_x, cloud_normals2->at(i).normal_y, cloud_normals2->at(i).normal_z);
	}
	fclose(fp_cloud_normals2);
	
	//sift�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sift2 = getSiftFromCloudAndParameter(cloud_sampled2, min_scale, nr_octaves, nr_scales_per_octave, min_contrast);

	//ȥ��sift�ؼ�����ظ�ֵ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_siftOutput2 = getFinalSift(cloud_sift2);
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
			if ( j % 10 == 0)
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
		float distance = correspondences->at(i).distance;
		float weight = correspondences->at(i).weight;
		float pointX1 = cloud_sampled1->points[firstID].x;
		float pointY1 = cloud_sampled1->points[firstID].y;
		float pointZ1 = cloud_sampled1->points[firstID].z;
		float pointX2 = cloud_sampled2->points[secondID].x - deltaX;
		//float pointX2 = cloud_sampled2->points[secondID].x;
		float pointY2 = cloud_sampled2->points[secondID].y - deltaY;
		//float pointY2 = cloud_sampled2->points[secondID].y ;
		float pointZ2 = cloud_sampled2->points[secondID].z;
		float dx = pointX2 - pointX1;
		float dy = pointY2 - pointY1;
		float dz = pointZ2 - pointZ1;

		fprintf(fp_correspondences, "(<%d,%d>,%f,%f,{<%f,%f,%f>})\n", 
			firstID, secondID,correspondences->at(i).distance, weight,
			dx,dy,dz);

		if (i % 10 == 0)
		{
			std::cout << std::endl;
		}
	}
	fclose(fp_correspondences);
	//�鿴��0����
	//����任����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Rawת��ǰ������" << cloud_in_filtered_Raw->size() << std::endl;
	//Eigen::Matrix4f transformation_matrix = sac.getBestTransformation();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_east;
	//pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation_matrix;
	Eigen::Matrix4f transformation_matrix;
	trans_east.estimateRigidTransformation(*cloud_siftOutput1, *cloud_siftOutput2, *correspondences, transformation_matrix);
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
	
	cloud_normals1->clear();
	cloud_sampled1->clear();
	cloud_sift1->clear();
	cloud_siftOutput1->clear();
	cloud_PFH1->clear();
	cloud_normals2->clear();
	cloud_sampled2->clear();
	cloud_sift2->clear();
	cloud_siftOutput2->clear();
	cloud_PFH2->clear();
	cloud_in_filtered_Raw->clear();
	
	return 0;

}