

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
	std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strImageName1,
		xSize1, ySize1,
		xResolution1, yResolution1,
		topLeftX1, topLeftY1);
	
	const char* strImageName2 = "e:\\DEM-2016.tif";
	int xSize2 = 0;
	int ySize2 = 0;
	double xResolution2 = 0;
	double yResolution2 = 0;
	double topLeftX2 = 0;
	double topLeftY2 = 0;
	std::vector<std::vector<Pt3>> rastervecvec2 = util::getRasterVecVecFromTif(strImageName2,
		xSize2, ySize2,
		xResolution2, yResolution2,
		topLeftX2, topLeftY2);

	//先求出两部分的交集(minx,miny)的最大值,(max,maxy)的最小值
	double minX = topLeftX1;
	if (minX < topLeftX2)
	{
		minX = topLeftX2;
	}
	double minY = topLeftY1 - (ySize1 - 1) * yResolution1;
	if (minY < topLeftX2 - (ySize2 - 1) * yResolution2)
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
	int minYID1 = (minY - (topLeftY1 - (ySize1 - 1) * yResolution1)) / yResolution1;
	int maxXID1 = (maxX - minX) / xResolution1;
	int maxYID1 = (maxY - minY) / yResolution1;

	for (int i = minYID1; i <= maxYID1; i++)
	{
		for (int j = minXID1; j <= maxXID1; j++)
		{
			double x = topLeftX1 + xResolution1 * j;
			double minY1 = topLeftY1 - (ySize1 - 1) * yResolution1;
			double y = minY1 + i * yResolution1;
			int minXID2 = (x - topLeftX2) / xResolution2;
			double minY2 = topLeftY2 - (ySize2 - 1) * yResolution2;
			int minYID2 = (y - minY2) / yResolution2;

			double value1 = rastervecvec1[i][j].z();
			double value2 = rastervecvec2[minYID2][minXID2].z();
			double difvalue = value2 - value1;
			Pt3 difPt = Pt3(x, y, difvalue);
			rastervecvec2[minYID2][minXID2] = difPt;
		}
	}
	
	//输出新图像
	const char * strFileName = "e:\\2016-2013.tif";
	util::createRasterFile(strFileName, 1, xSize2, ySize2, xResolution2, yResolution2, topLeftX2, topLeftY2);
	util::UpdateRasterFile(strFileName, rastervecvec2);

	return 0;
}