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
	/*
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
	*/
	//通过读入点云写tif
	//
	
	std::string strTifName1 = "E:\\test\\tif\\2013.tif";
	std::string strTifName2 = "E:\\test\\tif\\2016.tif";
	std::string diffTifName = "E:\\test\\diff.tif";
	util::getDifTifBetweenTwoTifs(strTifName1.c_str(), strTifName2.c_str(), diffTifName.c_str());
	
	return 0;

}