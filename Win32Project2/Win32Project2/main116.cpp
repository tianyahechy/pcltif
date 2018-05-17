#pragma once 
#include "common.h"
#include "AfxUtil.h"
#include <opencv2/xfeatures2d/nonfree.hpp>

//计算图像中的SIFT特征及匹配
cv::Mat computeSIFTFeatureAndCompare(cv::Mat srcImage1, cv::Mat srcImage2)
{
	
	//定义sift描述子
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

	//查找关键点
	std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
	f2d->detect(srcImage1, keyPoints_1);
	f2d->detect(srcImage2, keyPoints_2);

	//计算描述子
	cv::Mat descriptor_1, descriptor_2;
	f2d->compute(srcImage1, keyPoints_1, descriptor_1);
	f2d->compute(srcImage2, keyPoints_2, descriptor_2);
 
	//特征点匹配
	cv::BFMatcher matcher;
	std::vector<cv::DMatch> matchesVector;
	matcher.match(descriptor_1, descriptor_2, matchesVector);
	
	//剔除杂点(暂未做）
	//matchesVector.erase()

	//得到转换矩阵
	cv::Mat matchMat;
	cv::drawMatches(srcImage1, keyPoints_1, srcImage2, keyPoints_2, matchesVector, matchMat);

	return matchMat;
	
}
int main()
{
	//读取文件
	std::string strInputTifName1 = "E:\\DEM-2013.tif";
	std::string strInputTifName2 = "E:\\DEM-2016.tif";

	cv::Mat srcImage1 = cv::imread(strInputTifName1.c_str(), cv::ImreadModes::IMREAD_LOAD_GDAL);
	cv::Mat srcImage2 = cv::imread(strInputTifName2.c_str(), cv::ImreadModes::IMREAD_LOAD_GDAL);

	srcImage1.convertTo(srcImage1, CV_8UC1);
	srcImage2.convertTo(srcImage2, CV_8UC1);
	
	//提取矩形区域,得出sift匹配，粗配准
	int xRoi = 2000;
	int yRoi = 2000;
	int widthRoi = 1000;
	int heightRoi = 1000;

	cv::Mat roiImage1(widthRoi, heightRoi, CV_8UC1);
	cv::Mat roiImage2(widthRoi, heightRoi, CV_8UC1);

	srcImage1(cv::Rect(xRoi, yRoi, widthRoi, heightRoi)).copyTo(roiImage1);
	srcImage2(cv::Rect(xRoi, yRoi, widthRoi, heightRoi)).copyTo(roiImage2);

	cv::Mat matchMat = computeSIFTFeatureAndCompare(roiImage1, roiImage2);

	//将提取的图像转换为点云
	cv::imshow("matchMat", matchMat);

	cv::waitKey(0);

	//点云精配准icp

	return 0;

}