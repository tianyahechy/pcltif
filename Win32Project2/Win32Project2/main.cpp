#pragma once 
#include "siftProcess.h"
#include "myOpenCl.h"
struct pt3Int
{
	double x;
	double y;
	int z;
};

int main()
{
	/*
	//提取矩形区域,以供后面的sift匹配，粗配准
	int xRoi1 = 2000;
	int yRoi1 = 2000;
	int widthRoi = 500;
	int heightRoi = 500;

	std::string strInputTifName1 = "E:\\DEM-2013.tif";
	std::string strInputTifName2 = "E:\\DEM-2016.tif";
	std::string strInputPCDName1 = "E:\\DEM-2013.pcd";
	std::string strInputPCDName2 = "E:\\DEM-2016.pcd";
	
	siftProcess *theProcess = new siftProcess(widthRoi, heightRoi, strInputTifName1, strInputTifName2, strInputPCDName1, strInputPCDName2);
	theProcess->processAll();
	*/
	//通过读入点云写tif

	std::string strInputPointCloudName1 = "E:\\DEM-2013.pcd";
	std::string strOutPutTifName1 = "E:\\test\\tif\\test10w.tif";
	std::string strInputPointCloudName2 = "E:\\DEM-2016.pcd";
	std::string strOutPutTifName2 = "E:\\test\\tif\\DEM-2016.tif";
	double xResolution = 10;
	double yResolution = -10;
	int segementSize = 3;
	//通过读入点云写tif
	//util::writeTifFromPointCloud(strInputPointCloudName1.c_str(), strOutPutTifName1.c_str(), xResolution, yResolution);
	//util::writeTifFromPointCloud(strInputPointCloudName2.c_str(), strOutPutTifName2.c_str(), xResolution, yResolution);
	util::writeTifFromPointCloudBySegment(strInputPointCloudName1.c_str(), strOutPutTifName1.c_str(), segementSize, xResolution, yResolution);
	/*
	//设定一个值
	int testID = 0;
	std::map<int,Pt3> testSet;
	testSet.clear();
	for (size_t i = 0; i < 100; i++)
	{
		float ptX = i;
		float ptY = i;
		float ptZ = i;
		Pt3 thePt(ptX, ptY, ptZ);
		testSet.insert(std::pair<int, Pt3>(i, thePt));
	}
	//三角化数据集合中的X,Y坐标
	time_t startTime;
	time_t endTime;

	time(&startTime);
	int sizeOfPointCloud = testSet.size();
	std::vector<int> idVector;
	idVector.clear();
	std::vector<Pt3> thePt3Vector;
	thePt3Vector.clear();
	std::vector<float> xVector;
	xVector.clear();
	std::vector<float> yVector;
	yVector.clear();

	Delaunay tr;
	pt3Set::iterator iterCurVector3 = testSet.begin(),
		iterEndVector3 = testSet.end();
	for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
	{
		int id = iterCurVector3->first;
		Pt3 thePt3 = iterCurVector3->second;
		float x = thePt3.x();
		float y = thePt3.y();

		idVector.push_back(id);
		thePt3Vector.push_back(thePt3);
		xVector.push_back(x);
		yVector.push_back(y);
	}
	std::string strOpenCLFileName = "test.cl";
	std::string strOpenCLKernalEntry = "hello_kernel";
	int sizeOfInputType = 2;
	int sizeOfInputObject = sizeOfPointCloud;
	int sizeOfEachInputUnit = sizeof(float);
	std::vector<std::vector<float>> inputVec2;
	//设定各单元数值
	inputVec2.clear();
	inputVec2.resize(sizeOfInputType);
	for (size_t j = 0; j < sizeOfInputType; j++)
	{
		inputVec2[j].resize(sizeOfInputObject);
	}

	for (size_t i = 0; i < sizeOfInputObject; i++)
	{
		inputVec2[0][i] = xVector[i];
		inputVec2[1][i] = yVector[i];
	}

	int sizeOfOutputType = 1;
	int sizeOfOutputObject = sizeOfPointCloud;
	int sizeOfEachOutputUnit = sizeof(float);
	std::vector<std::vector<float>> outputVec2;
	outputVec2.clear();
	outputVec2.resize(sizeOfOutputType);
	for (size_t j = 0; j < sizeOfOutputType; j++)
	{
		outputVec2[j].resize(sizeOfOutputObject);
	}

	myOpenCL theOpenCL(strOpenCLFileName,
		strOpenCLKernalEntry,
		sizeOfInputType,
		sizeOfInputObject,
		sizeOfEachInputUnit,
		inputVec2,
		sizeOfOutputType,
		sizeOfOutputObject,
		sizeOfEachOutputUnit,
		outputVec2);

	theOpenCL.process();

	time(&endTime);
	double theTime = difftime(endTime, startTime);
	std::cout << "纯三角化时间:" << theTime << std::endl;
	*/
	return 0;

}