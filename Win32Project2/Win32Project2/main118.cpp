#pragma once 
#include "common.h"
#include "AfxUtil.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>


//����ͼ���е�SIFT������ƥ��
cv::Mat computeSIFTFeatureAndCompare(cv::Mat srcImage1, cv::Mat srcImage2)
{
	
	//����sift������
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

	//���ҹؼ���
	std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
	f2d->detect(srcImage1, keyPoints_1);
	f2d->detect(srcImage2, keyPoints_2);

	//����������
	cv::Mat descriptor_1, descriptor_2;
	f2d->compute(srcImage1, keyPoints_1, descriptor_1);
	f2d->compute(srcImage2, keyPoints_2, descriptor_2);
 
	//������ƥ��
	cv::BFMatcher matcher;
	std::vector<cv::DMatch> matchesVector;
	matcher.match(descriptor_1, descriptor_2, matchesVector);
	
	//�޳��ӵ�(һ�룩
	int matcherSize = matchesVector.size() / 2;
	std::cout << matcherSize << std::endl;
	std::nth_element(matchesVector.begin(), matchesVector.begin() + matcherSize / 2, matchesVector.end());
	//�޳������ƥ����
	matchesVector.erase(matchesVector.begin() + matcherSize / 2 + 1, matchesVector.end());

	//matchesVector.erase()

	//�õ�ת������
	cv::Mat matchMat;
	cv::drawMatches(srcImage1, keyPoints_1, srcImage2, keyPoints_2, matchesVector, matchMat);

	return matchMat;
	
}
int main()
{
	//��ȡ�ļ�
	std::string strInputTifName1 = "E:\\DEM-2013.tif";
	std::string strInputTifName2 = "E:\\DEM-2016.tif";
	//ͨ��gdal��ȡ.tif���м������Ӧ������ͼ�������
	int xSize1 = 0;
	int ySize1 = 0;
	double xResolution1 = 0;
	double yResolution1 = 0;
	double topLeftX1 = 0;
	double topLeftY1 = 0;
	std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strInputTifName1.c_str(),
		xSize1, ySize1,
		xResolution1, yResolution1,
		topLeftX1, topLeftY1);
	std::cout << yResolution1 << std::endl;

	int xSize2 = 0;
	int ySize2 = 0;
	double xResolution2 = 0;
	double yResolution2 = 0;
	double topLeftX2 = 0;
	double topLeftY2 = 0;
	std::vector<std::vector<Pt3>> rastervecvec2 = util::getRasterVecVecFromTif(strInputTifName2.c_str(),
		xSize2, ySize2,
		xResolution2, yResolution2,
		topLeftX2, topLeftY2);
	std::cout << yResolution2 << std::endl;

	//��ȡ��������,�Թ������siftƥ�䣬����׼
	int xRoi = 2000;
	int yRoi = 2000;
	int widthRoi = 1000;
	int heightRoi = 1000;

	//��һ��ͼ���ȡ1000*1000
	std::vector<std::vector<Pt3>> selectImage1Vector;
	selectImage1Vector.clear();
	selectImage1Vector.resize(heightRoi);
	for (int i = 0; i <  heightRoi; i++)
	{
		selectImage1Vector[i].resize(widthRoi);
		for (int j = 0; j < widthRoi; j++)
		{
			double x = rastervecvec1[xRoi + i][yRoi + j].x();
			double y = rastervecvec1[xRoi + i][yRoi + j].y();
			double z = rastervecvec1[xRoi + i][yRoi + j].z();
			Pt3 thePt(x, y, z);
			selectImage1Vector[i][j] = thePt;
		}
	}

	//�ڶ���ͼ���ֵ���һ��ͼ��x,y������ͬ
	std::vector<std::vector<Pt3>> selectImage2Vector;
	selectImage2Vector.clear();
	selectImage2Vector.resize(heightRoi);
	for (int i = 0; i < heightRoi; i++)
	{
		selectImage2Vector[i].resize(widthRoi);
		for (int j = 0; j < widthRoi; j++)
		{
			double x = selectImage1Vector[i][j].x();
			double y = selectImage1Vector[i][j].y();
			//�����ڵ�һ��ͼ�е�λ���ڵڶ���ͼ�е�������꣬��i,j)
			int xID2 = (x - topLeftX2) / xResolution2;
			int yID2 = (y - topLeftY2) / yResolution2;
			double value1 = rastervecvec2[yID2][xID2].z();
			Pt3 thePt(xID2, yID2, value1);
			selectImage2Vector[i][j] = thePt;
		}
	}
	rastervecvec1.clear();
	rastervecvec2.clear();

	//д���ݵ�opencv��cv::Mat

	cv::Mat srcImage1(widthRoi, heightRoi, CV_32FC1);
	for (int i = 0; i < heightRoi; i++)
	{
		for (int j = 0; j < widthRoi; j++)
		{
			float z = selectImage1Vector[i][j].z();
			srcImage1.at<float>(i, j) = z;
		}
	}
	selectImage1Vector.clear();

	cv::Mat srcImage2(widthRoi, heightRoi, CV_32FC1);
	for (int i = 0; i < heightRoi; i++)
	{
		for (int j = 0; j < widthRoi; j++)
		{
			float z = selectImage2Vector[i][j].z();
			srcImage2.at<float>(i, j) = z;
		}
	}
	selectImage2Vector.clear();

	srcImage1.convertTo(srcImage1, CV_8UC1);
	srcImage2.convertTo(srcImage2, CV_8UC1);
	
	cv::Mat matchMat = computeSIFTFeatureAndCompare(srcImage1, srcImage2);

	
	//����ȡ��ͼ��ת��Ϊ����
	cv::imshow("matchMat", matchMat);

	cv::waitKey(0);

	//���ƾ���׼icp

	return 0;

}