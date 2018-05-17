

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

int main()
{

	const char* strInputTifName = "E:\\DEM-2013-corrected.tif";
	const char* strOutPutPointCloudName = "E:\\DEM-2013-corrected.pcd";	//读取源图像并转化为灰度图像
	int xSize1 = 0;
	int ySize1 = 0;
	double xResolution1 = 0;
	double yResolution1 = 0;
	double topLeftX1 = 0;
	double topLeftY1 = 0;
	//从TIF读到二维数组
	std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strInputTifName,
		xSize1, ySize1,
		xResolution1, yResolution1,
		topLeftX1, topLeftY1);

	//写点云
	pcl::PointCloud<pcl::PointXYZ> cloudOutput;
	cloudOutput.clear();

	for (int i = 0; i < ySize1; i++)
	{
		int id = ySize1 - 1 - i;
		for (int j = 0; j < xSize1; j++)
		{
			Pt3 thePt = rastervecvec1[id][j];
			double x = thePt.x();
			double y = thePt.y();
			double z = thePt.z();
			if (z != 0)
			{
				pcl::PointXYZ thePt;
				thePt.x = x;
				thePt.y = y;
				thePt.z = z;
				cloudOutput.push_back(thePt);
			}
		}

	}
	cloudOutput.width = cloudOutput.size();
	cloudOutput.height = 1;
	cloudOutput.is_dense = false;
	cloudOutput.resize(cloudOutput.width * cloudOutput.height);

	pcl::io::savePCDFileASCII(strOutPutPointCloudName, cloudOutput);

	rastervecvec1.clear();
	cloudOutput.clear();

	return 0;
}