

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main()
{
	const char* strImageName1 = "e:\\DEM-2013-corrected.tif";
	int xSize1 = 0;
	int ySize1 = 0;
	double xResolution1 = 0;
	double yResolution1 = 0;
	double topLeftX1 = 0;
	double topLeftY1 = 0;
	std::vector<std::vector<Pt3>> rastervecvec = util::getRasterVecVecFromTif(strImageName1,
		xSize1, ySize1,
		xResolution1, yResolution1,
		topLeftX1, topLeftY1);
	/*
	const char* strImageName2 = "e:\\DEM-2016.tif";
	int xSize2 = 0;
	int ySize2 = 0;
	double xResolution2 = 0;
	double yResolution2 = 0;
	double topLeftX2 = 0;
	double topLeftY2 = 0;
	rasterVec3Set rasterSet2 = util::getRasterSetFromTif(strImageName2,
		xSize2, ySize2,
		xResolution2, yResolution2,
		topLeftX2, topLeftY2);

	rasterVec3Set::iterator
		iterCur2013 = rasterSet1.begin(),
		iterEnd2013 = rasterSet1.end(),
		iterCur2016 = rasterSet2.begin(),
		iterEnd2016 = rasterSet2.end();
	for (; iterCur2013 != iterEnd2013; iterCur2013++)
	{
		std::vector<Pt3> vec2013 = iterCur2013->second;
		std::vector<Pt3>::iterator
			iterCurPt2013 = vec2013.begin(),
			iterEndPt2013 = vec2013.end();
		{
			Pt3 curPt2013 = * 
		}

	}

	//先求出两部分的交集(minx,miny,max,maxy)的最小值
	double minX = topLeftX1;
	if (minX > topLeftX2)
	{
		minX = topLeftX2;
	}
	double minY = topLeftY1 - (ySize1 - 1) * yResolution1;
	if (minY > topLeftX2 - (ySize2 - 1) * yResolution2)
	{
		minY = topLeftX2 - (ySize2 - 1) * yResolution2;
	}
	double maxX = topLeftX1 + (xSize1 - 1) * xResolution1;
	if (maxX > topLeftX2 + (xSize2 - 1) * xResolution2)
	{
		maxX = topLeftX2 + (xSize2 - 1) * xResolution2;
	}
	double maxY = topLeftY1;
	if (maxY > topLeftY2)
	{
		maxY = topLeftY2;
	}

	int minXID1 = (minX - topLeftX1) / xResolution1;
	int minYID1 = (minY - ( topLeftY1 - (ySize1 - 1) * yResolution1 ) ) / 
	*/
	//输出新图像
	const char * strFileName = "e:\\2013-2016.tif";
	util::createRasterFile(strFileName, 1, xSize1, ySize1, xResolution1, yResolution1, topLeftX1, topLeftY1);
	util::UpdateRasterFile(strFileName, rastervecvec);

	return 0;
}