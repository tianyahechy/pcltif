

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main()
{
	//记录开始时的当前时间和结束时间
	time_t startTime, endTime;
	double diff;
	time(&startTime);

	//设置X，Y方向的像素分辨率
	double xResolution = 1.0;
	double yResolution = 1.0;

	PCLTif * thePCLTif = new PCLTif("e:\\pt666000001_del.pcd", xResolution, yResolution);
	thePCLTif->process();

	//创建网格文件
	const char * pszRasterFile = "E:\\pt666000001_del9.tif";
	int bandSize = 1;
	int xSize = thePCLTif->getXSize();
	int ySize = thePCLTif->getYSize();
	//设置左上角为minx,maxY
	PCLDetail theDetail = thePCLTif->getDetailOfthePointCloud();
	double topLeftX = theDetail.minX;
	double topLeftY = theDetail.maxY;
	util::createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//更新网格文件,生成.tiff文件
	rasterVec3Set imageSet = thePCLTif->getRasterVec3Set();
	util::UpdateRasterFile(pszRasterFile, imageSet);

	//记录结束时间
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "总共花费时间" << diff << "秒" << std::endl;

	return 0;
}