

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

int main()
{
	const char* strInputFileName = "E:\\DEM-2013-corrected.tif";
	const char* strOutputFileName = "E:\\DEM-2013-corrected.pcd";
	util::writePointCloudFromTif(strInputFileName, strOutputFileName);
	return 0;
}