

#pragma once
#include "PCLTif.h"

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
	
	//根据输入点集合来等分XYZ间距，得到图像坐标集合vector,并初始化灰度值为0
	thePCLTif->getEqualXYZVectorFromDataSet();
	//三角化数据集合中的X,Y坐标,
	std::cout << "三角化数据集合中的X,Y坐标,重组带Z值的三角形集合" << std::endl;
	thePCLTif->getTriangleSetFromDataSet();
	//根据三角形集合计算出栅格集合，插值
	thePCLTif->getRasterVec3SetFromTriangleSet();

	rasterVec3Set imageSet = thePCLTif->getRasterVec3Set();
	//创建网格文件
	PCLDetail theDetail = thePCLTif->getDetailOfthePointCloud();
	//设置左上角为minx,maxY
	double minX = theDetail.minX;
	double minY = theDetail.minY;
	int xSize = thePCLTif->getXSize();
	int ySize = thePCLTif->getYSize();
	const char * pszRasterFile = "E:\\pt666000001_del8.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//设定图像左上角
	float topLeftX = minX;
	float topLeftY = maxY;
	thePCLTif->createRasterFile(pszRasterFile, bandSize, xSize, ySize, topLeftX, topLeftY);
	//更新网格文件,生成.tiff文件
	thePCLTif->UpdateRasterFile(pszRasterFile, imageSet);
	
	//记录结束时间
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "总共花费时间" << diff << "秒" << std::endl;

	return 0;
}