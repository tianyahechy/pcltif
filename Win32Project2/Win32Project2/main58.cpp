

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main(int argc, char **argv)
{
	//记录开始时的当前时间和结束时间
	time_t startTime, endTime;
	double diff;
	time(&startTime);

	//设置X，Y方向的像素分辨率
	double xResolution = 1.0;
	double yResolution = -1.0;

	//const char * inputName = "e:\\CSite1origC.pcd";
	const char * inputName = argv[1];
	PCLTif * thePCLTif1 = new PCLTif(inputName, xResolution, yResolution);
	thePCLTif1->process();
	std::vector<std::vector<Pt3>> rastervec = thePCLTif1->getRasterVecVec3();
	
	//创建网格文件
	//const char * pszRasterFile = "E:\\CSite1origC.tif";
	const char * pszRasterFile = argv[2];
	int bandSize = 1;
	int xSize = thePCLTif1->getXSize();
	int ySize = thePCLTif1->getYSize();
	//设置左上角为minx,maxY
	PCLDetail theDetail = thePCLTif1->getDetailOfthePointCloud();
	double topLeftX = theDetail.leftTopX;
	double topLeftY = theDetail.leftTopY;
	util::createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//更新网格文件,生成.tiff文件
	util::UpdateRasterFile(pszRasterFile, rastervec);

	//记录结束时间
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "总共花费时间" << diff << "秒" << std::endl;

	return 0;
	return 0;
}