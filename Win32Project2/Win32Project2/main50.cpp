

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main()
{
	const char* strImageName = "e:\\DEM-2013-corrected.tif";
	int xSize = 0;
	int ySize = 0;
	double xResolution = 0;
	double yResolution = 0;
	double topLeftX = 0;
	double topLeftY = 0;
	rasterVec3Set rasterSet = util::getRasterSetFromTif(strImageName,
		xSize, ySize,
		xResolution, yResolution,
		topLeftX, topLeftY);

	//Êä³öÐÂÍ¼Ïñ
	const char * strFileName = "e:\\test3.tif";
	util::createRasterFile(strFileName, 1, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	util::UpdateRasterFile(strFileName, rasterSet);

	return 0;
}