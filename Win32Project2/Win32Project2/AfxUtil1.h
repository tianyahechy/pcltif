#pragma once
#include "common.h"

namespace util
{
	//通过栅格创建TIF文件
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);

	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet);
	//通过读取tif文件创建栅格集合
	rasterVec3Set getRasterSetFromTif(const char* strImageName, 
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);

	//用数组更合适
	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, std::vector<std::vector<Pt3>> theRasterVecVec);
	//通过读取tif文件创建栅格集合
	std::vector<std::vector<Pt3>> getRasterVecVecFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);

	//通过读入tif,写点云
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName);
	
}