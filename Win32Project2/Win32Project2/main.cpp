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
	//��ȡ��������,�Թ������siftƥ�䣬����׼
	int xRoi1 = 2000;
	int yRoi1 = 2000;
	int widthRoi = 2000;
	int heightRoi = 100;

	std::string strInputTifName1 = "E:\\DEM-2013.tif";
	std::string strInputTifName2 = "E:\\DEM-2016.tif";
	
	siftProcess *theProcess = new siftProcess(widthRoi, heightRoi, strInputTifName1, strInputTifName2);
	theProcess->processAll();
	return 0;

}