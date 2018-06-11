#pragma once 
#include "siftProcess.h"

struct pt3Int
{
	double x;
	double y;
	int z;
};

int main()
{
	
	//提取矩形区域,以供后面的sift匹配，粗配准
	int xRoi1 = 2000;
	int yRoi1 = 2000;
	int widthRoi = 500;
	int heightRoi = 500;

	std::string strInputTifName1 = "E:\\DEM-2013.tif";
	std::string strInputTifName2 = "E:\\DEM-2016.tif";
	std::string strInputPCDName1 = "E:\\DEM-2013.pcd";
	std::string strInputPCDName2 = "E:\\DEM-2016.pcd";
	
	siftProcess *theProcess = new siftProcess(widthRoi, heightRoi, strInputTifName1, strInputTifName2, strInputPCDName1, strInputPCDName2);
	theProcess->processAll();
	
	//通过读入点云写tif
	//
	/*
	std::string strInputPointCloudName1 = "E:\\test\\cloud_test_corrected_2013.pcd";
	std::string strOutPutTifName1 = "E:\\test\\2013.tif";
	std::string strInputPointCloudName2 = "E:\\test\\cloud_test2_2016.pcd";
	std::string strOutPutTifName2 = "E:\\test\\2016.tif";
	double xResolution = 10;
	double yResolution = -10;
	util::writeTifFromPointCloud(strInputPointCloudName1.c_str(), strOutPutTifName1.c_str(), xResolution, yResolution);
	util::writeTifFromPointCloud(strInputPointCloudName2.c_str(), strOutPutTifName2.c_str(), xResolution, yResolution);
	*/
	return 0;

}