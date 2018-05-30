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
	//读取文件
	std::string strInputTifName1 = "E:\\DEM-2013-8bit_stretch.tif";
	std::string strInputTifName2 = "E:\\DEM-2016-8bit-stretched.tif";
	//提取矩形区域,以供后面的sift匹配，粗配准
	int xRoi1 = 2000;
	int yRoi1 = 2000;
	int widthRoi = 2000;
	int heightRoi = 100;

	std::string strOriginalTif1 = "E:\\DEM-2013.tif";
	std::string strOriginalTif2 = "E:\\DEM-2016.tif";
	
	siftProcess *theProcess = new siftProcess(xRoi1, yRoi1, widthRoi, heightRoi, strInputTifName1, strInputTifName2, strOriginalTif1, strOriginalTif2);
	theProcess->processAll();
	return 0;

}