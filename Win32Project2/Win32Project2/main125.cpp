#pragma once 
#include "common.h"
#include "AfxUtil.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>

//计算图像中的SIFT特征及匹配
cv::Mat computeSIFTFeatureAndCompare(cv::Mat srcImage1, cv::Mat srcImage2)
{
	//定义sift描述子
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create(0,3,0.004,50,1.6);

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
	
	//输出匹配结果
	FILE * fp = fopen("e:\\test\\opencvSift.txt", "w");
	fprintf(fp,"firstID        secondID                第一个坐标                       第二个坐标                            坐标差值\n");
	for (size_t i = 0; i < matchesVector.size(); i++)
	{
		int idFirst = matchesVector[i].queryIdx;
		int idSecond = matchesVector[i].trainIdx;
		cv::Point2f ptFirst = keyPoints_1[idFirst].pt;
		float firstPtX = ptFirst.x;
		float firstPtY = ptFirst.y;
		cv::Point2f ptSecond = keyPoints_2[idSecond].pt;
		float secondPtX = ptSecond.x;
		float secondPtY = ptSecond.y;
		float diffX = ptSecond.x - ptFirst.x;
		float diffY = ptSecond.y - ptFirst.y;
		//如果diffX在(-50,50)和diffy在（-10，10）之间，则输出 

		if (abs(diffX) < 50 && abs(diffY) < 10)
		{
			fprintf(fp, "    %-15d%-15d%-10.3f%-25.3f%-10.3f%-25.3f%-10.3f%-25.3f\n", idFirst, idSecond, firstPtX, firstPtY, secondPtX, secondPtY, diffX, diffY);

		}
	}
	fclose(fp);
	//剔除杂点(一半）
	//int matcherSize = matchesVector.size() / 2;
	//std::cout << matcherSize << std::endl;
	//std::nth_element(matchesVector.begin(), matchesVector.begin() + matcherSize / 2, matchesVector.end());
	//剔除其余的匹配结果
	//matchesVector.erase(matchesVector.begin() + matcherSize / 2 + 1, matchesVector.end());

	//得到转换矩阵
	cv::Mat matchMat;
	cv::drawMatches(srcImage1, keyPoints_1, srcImage2, keyPoints_2, matchesVector, matchMat);

	return matchMat;
	
}

struct pt3Int
{
	double x;
	double y;
	int z;
};

int main()
{
	//读取文件
	std::string strInputTifName1 = "E:\\DEM-2013-8bit_stretch.tif";
	std::string strInputTifName2 = "E:\\DEM-2016-8bit-stretched.tif";
	//提取矩形区域,以供后面的sift匹配，粗配准
	int xRoi1 = 2000;
	int yRoi1 = 2000;
	int widthRoi = 2000;
	int heightRoi = 100;
	double leftTopX1 = 0;
	double leftTopY1 = 0;
	std::vector<uchar> rastervecvec1 = util::getSegRasterVecVecFromTif_8bit(strInputTifName1.c_str(),
		xRoi1, yRoi1,
		widthRoi, heightRoi, leftTopX1, leftTopY1);
	std::cout << "输出rasterVecvec1" << std::endl;

	std::cout << "leftTopX1 =" << leftTopX1 << "leftTopY1 = " << leftTopY1 << std::endl;
	std::vector<uchar> rastervecvec2 = util::getSegRasterVecVecFromTifAndLeftTop_8bit(strInputTifName2.c_str(),
		widthRoi, heightRoi,leftTopX1,leftTopY1);

	//写数据到opencv的cv::Mat
	cv::Mat srcImage1(heightRoi, widthRoi, CV_8UC1);
	cv::imwrite("e:\\test\\emptysrc1.jpg", srcImage1);
	for (int i = 0; i < heightRoi; i++)
	{
		for (int j = 0; j < widthRoi; j++)
		{
			srcImage1.at<uchar>(i, j) = rastervecvec1[i * widthRoi + j];
		}
	}
	rastervecvec1.clear();

	std::cout << "输出srcImage1" << std::endl;

	cv::Mat srcImage2(heightRoi, widthRoi, CV_8UC1);
	cv::imwrite("e:\\test\\emptysrc2.jpg", srcImage2);
	for (int i = 0; i < heightRoi; i++)
	{
		for (int j = 0; j < widthRoi; j++)
		{
			srcImage2.at<uchar>(i, j) = rastervecvec2[i * widthRoi + j];
		}
	}
	rastervecvec2.clear();

	std::cout << "输出srcImage2" << std::endl;
	//srcImage1.convertTo(srcImage1, CV_8UC1);
	//srcImage2.convertTo(srcImage2, CV_8UC1);
	
	cv::Mat matchMat = computeSIFTFeatureAndCompare(srcImage1, srcImage2);

	std::cout << "输出matchMat" << std::endl;
	//写图像
	//将提取的图像转换为点云
	cv::imshow("matchMat", matchMat);
	cv::imshow("srcImage1", srcImage1);
	cv::imshow("srcImage2", srcImage2);

	cv::imwrite("e:\\test\\src1.jpg", srcImage1);
	cv::imwrite("e:\\test\\src2.jpg", srcImage2);

	cv::waitKey(0);

	//点云精配准icp

	return 0;

}