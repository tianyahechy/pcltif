

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

int main()
{
	//读取源图像并转化为灰度图像
	std::string inputFileName = "E:\\2016-2013.tif";
	cv::Mat srcImage = cv::imread( inputFileName, 8);
	//定义结构元素
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
	cv::Mat openMat;
	cv::morphologyEx(srcImage, openMat, cv::MORPH_OPEN, element);
	int row = openMat.rows;
	int col = openMat.cols;
	
	std::vector<std::vector<Pt3>> rasterOutput;
	rasterOutput.clear();
	rasterOutput.resize(row);
	for ( int  i = 0; i < row; i++)
	{
		rasterOutput[i].resize(col);
	}
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			float theVaue = openMat.at<float>(i, j);
			Pt3 thePt(j,i,theVaue);
			rasterOutput[i][j] = thePt;
		}
	}

	//输出TIF

	int xSize1 = 0;
	int ySize1 = 0;
	double xResolution1 = 0;
	double yResolution1 = 0;
	double topLeftX1 = 0;
	double topLeftY1 = 0;
	const char * inputChar = inputFileName.c_str();
	std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(inputChar,
		xSize1, ySize1,
		xResolution1, yResolution1,
		topLeftX1, topLeftY1);

	xSize1 = col;
	ySize1 = row;
	const char * outPutFileName = "E:\\openMat.tif";
	util::createRasterFile(outPutFileName, 1, xSize1, ySize1, xResolution1, yResolution1, topLeftX1, topLeftY1);
	util::UpdateRasterFile(outPutFileName, rasterOutput);
	
	return 0;
}