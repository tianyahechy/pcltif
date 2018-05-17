

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main()
{
	//设定支持中文路径
	CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
	//注册栅格驱动
	GDALAllRegister();
	//使用只读方式打开图像
	const char * pszFile = "E:\\DEM-2016.tif";
	GDALDataset * poDataSet = (GDALDataset *)GDALOpen(pszFile, GA_ReadOnly);
	if (poDataSet == NULL)
	{
		std::cout << pszFile << "不能打开!" << std::endl;
		return -1;
	}

	//输出图像的格式信息
	std::cout << "driver:" << poDataSet->GetDriver()->GetDescription() << "/" << poDataSet->GetDriver()->GetMetadataItem(GDAL_DMD_LONGNAME)
		<< std::endl;
	//输出图像的大小和波段
	std::cout << "size is " << poDataSet->GetRasterXSize() << "x" << poDataSet->GetRasterYSize() << "x" << poDataSet->GetRasterCount() << std::endl;

	int xSize = poDataSet->GetRasterXSize();
	int ySize = poDataSet->GetRasterYSize();

	//输出图像的投影信息
	if ( poDataSet->GetProjectionRef() != NULL )
	{
		printf("Projection is '%s'\n", poDataSet->GetProjectionRef());
	}

	//输出图像的坐标和分辨率信息
	double adfGeoTransform[6];
	if ( poDataSet->GetGeoTransform(adfGeoTransform) == CE_None )
	{
		std::cout << "origin:x=" << adfGeoTransform[0] << ", y=" << adfGeoTransform[3] << std::endl;
		std::cout << "Pixel size:x= " << adfGeoTransform[1] << ",y = " << adfGeoTransform[5] << std::endl;
	}

	double minX = adfGeoTransform[0];
	double minY = adfGeoTransform[3];
	double xResolution = adfGeoTransform[1];
	double yResolution = adfGeoTransform[5] * (-1);

	//读取第一个波段
	GDALRasterBand * poBand = poDataSet->GetRasterBand(1);
	int bGotMin, bGotMax;
	double adfMinMax[2];
	adfMinMax[0] = poBand->GetMinimum(&bGotMin);
	adfMinMax[1] = poBand->GetMaximum(&bGotMax);

	if ( !(bGotMax && bGotMin ))
	{
		GDALComputeRasterMinMax((GDALRasterBandH)poBand, TRUE, adfMinMax);
		std::cout << "min = " << adfMinMax[0] << "max=" << adfMinMax[1] << std::endl;
	}

	rasterVec3Set rasterSet;
	rasterSet.clear();

	for ( int i = 0; i < ySize; i++)
	{
		std::vector<Pt3> outputVec;
		outputVec.clear();
		std::vector<float> testFloat;
		testFloat.clear();
		testFloat.resize(xSize);
		//读取图像的第一行数据
		poBand->RasterIO(GF_Read, 0, i, xSize, 1, (float*)&testFloat[0], xSize, 1, GDT_Float32, 0, 0);

		for ( int j = 0; j < xSize; j++)
		{
			double x = minX + xResolution * j;
			double y = minY + yResolution * i;
			double z = testFloat[j];
			Pt3 thisPoint = Pt3(x, y, z);
			outputVec.push_back(thisPoint);
		}
		rasterSet.insert(rasterVec3Pair(i, outputVec));
	}

	GDALClose((GDALDatasetH)poDataSet);

	//输出新图像
	const char * strFileName = "e:\\test2.tif";
	util::createRasterFile(strFileName, 1, xSize, ySize, xResolution, yResolution, minX, minY);
	util::UpdateRasterFile(strFileName, rasterSet);

	return 0;
}