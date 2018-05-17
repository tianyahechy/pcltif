

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main()
{
	//�趨֧������·��
	CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
	//ע��դ������
	GDALAllRegister();
	//ʹ��ֻ����ʽ��ͼ��
	const char * pszFile = "E:\\DEM-2016.tif";
	GDALDataset * poDataSet = (GDALDataset *)GDALOpen(pszFile, GA_ReadOnly);
	if (poDataSet == NULL)
	{
		std::cout << pszFile << "���ܴ�!" << std::endl;
		return -1;
	}

	//���ͼ��ĸ�ʽ��Ϣ
	std::cout << "driver:" << poDataSet->GetDriver()->GetDescription() << "/" << poDataSet->GetDriver()->GetMetadataItem(GDAL_DMD_LONGNAME)
		<< std::endl;
	//���ͼ��Ĵ�С�Ͳ���
	std::cout << "size is " << poDataSet->GetRasterXSize() << "x" << poDataSet->GetRasterYSize() << "x" << poDataSet->GetRasterCount() << std::endl;

	int xSize = poDataSet->GetRasterXSize();
	int ySize = poDataSet->GetRasterYSize();

	//���ͼ���ͶӰ��Ϣ
	if ( poDataSet->GetProjectionRef() != NULL )
	{
		printf("Projection is '%s'\n", poDataSet->GetProjectionRef());
	}

	//���ͼ�������ͷֱ�����Ϣ
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

	//��ȡ��һ������
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
		//��ȡͼ��ĵ�һ������
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

	//�����ͼ��
	const char * strFileName = "e:\\test2.tif";
	util::createRasterFile(strFileName, 1, xSize, ySize, xResolution, yResolution, minX, minY);
	util::UpdateRasterFile(strFileName, rasterSet);

	return 0;
}