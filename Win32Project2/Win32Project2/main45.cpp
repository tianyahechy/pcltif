

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

	PCLTif * thePCLTif1 = new PCLTif("e:\\pt666000001_del.pcd", xResolution, yResolution);
	thePCLTif1->process();
	//PCLTif * thePCLTif2 = new PCLTif("e:\\pt666000001_del.pcd", xResolution, yResolution);
	//thePCLTif2->process();

	rasterVec3Set imageSet1 = thePCLTif1->getRasterVec3Set();
	//rasterVec3Set imageSet2 = thePCLTif2->getRasterVec3Set();
	/*
	//求差值
	rasterVec3Set outputSet;
	outputSet.clear();
	rasterVec3Set::iterator
		iterCur1 = imageSet1.begin(),
		iterEnd1 = imageSet1.end(),
		iterCur2 = imageSet2.begin(),
		iterEnd2 = imageSet2.end();
	for (; iterCur1 != iterEnd1, iterCur2 != iterEnd2; iterCur1++, iterCur2++)
	{
		int id = iterCur1->first;
		std::vector<Pt3> vec1 = iterCur1->second;
		std::vector<Pt3> vec2 = iterCur2->second;
		std::vector<Pt3> outVec;
		outVec.clear();
		std::vector<Pt3>::iterator
			iterCurPt1 = vec1.begin(),
			iterEndPt1 = vec1.end(),
			iterCurPt2 = vec2.begin(),
			iterEndPt2 = vec2.end();
		for (; iterCurPt1 != iterEndPt1, iterCurPt2 != iterEndPt2; iterCurPt1++, iterCurPt2++ )
		{
			Pt3 p1 = *iterCurPt1;
			double x1 = p1.x();
			double y1 = p1.y();
			double z1 = p1.z();
			Pt3 p2 = *iterCurPt2;
			double x2 = p2.x();
			double y2 = p2.y();
			double z2 = p2.z();
			double xDiff = x1 - x2;
			double yDiff = y1 - y2;
			double zDiff = z1 - z2;

			Pt3 difPt3(xDiff, yDiff, zDiff);
			outVec.push_back(difPt3);
		}
		outputSet.insert(rasterVec3Pair(id, outVec));

	}
	*/

	//创建网格文件
	const char * pszRasterFile = "E:\\pt666000001_del.tif";
	int bandSize = 1;
	int xSize = thePCLTif1->getXSize();
	int ySize = thePCLTif1->getYSize();
	//设置左上角为minx,maxY
	PCLDetail theDetail = thePCLTif1->getDetailOfthePointCloud();
	double topLeftX = theDetail.minX;
	double topLeftY = theDetail.maxY;
	util::createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//更新网格文件,生成.tiff文件
	util::UpdateRasterFile(pszRasterFile, imageSet1);

	//记录结束时间
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "总共花费时间" << diff << "秒" << std::endl;

	return 0;
}