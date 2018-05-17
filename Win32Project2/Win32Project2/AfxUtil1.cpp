#include "AfxUtil.h"
namespace util
{
	//ͨ��դ�񴴽�TIF�ļ�
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX, double rotationY)
	{
		//�趨֧������·��
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//ע��դ������
		GDALAllRegister();
		//�����ȡָ����ʽ�����������ڴ���ͼ��
		const char * pszFormat = "GTiff";
		GDALDriver * poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		if (poDriver == NULL)
		{
			std::cout << "��ʽ" << pszFormat << "��֧��create()����" << std::endl;
		}
		else
		{
			std::cout << "����OK��" << std::endl;
		}

		//��ȡ��������һЩԪ������Ϣ
		char ** papszMetadata = poDriver->GetMetadata();
		if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
		{
			std::cout << pszFormat << "֧��Create()����" << std::endl;
		}
		if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
		{
			std::cout << pszFormat << "֧��CreateCopy()����" << std::endl;
		}

		//��������ļ�����СΪ512*512*3����������8bit������
		GDALDataset * poDS = poDriver->Create(strImageName, xSize, ySize, bandSize, GDT_Float32, NULL);
		if (poDS == NULL)
		{
			std::cout << "����ͼ��" << strImageName << "ʧ��" << std::endl;
			return;
		}
		//��������������һ���͵��ĸ���ͼ�����Ͻǵ����꣬��2���͵�6��Ϊ���������ֱ��ʣ���������Ϊ��ת�Ƕ�
		double dGeotransform[6] = { 0, 0, 0, 0, 0, 0 };
		dGeotransform[0] = topLeftX;   //��������X
		dGeotransform[1] = xResolution;	//����ֱ���
		dGeotransform[2] = rotationX;	//��ת�Ƕ�
		dGeotransform[3] = topLeftY;	//��������Y
		dGeotransform[4] = rotationY;	//��ת�Ƕ�
		dGeotransform[5] = yResolution;	//y�ֱ��ʣ���ֵ��

		poDS->SetGeoTransform(dGeotransform);

		GDALClose((GDALDatasetH)poDS);
		std::cout << "���ݼ��رգ�" << std::endl;


	}
	
	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
	{
		//�趨֧������·��
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//ע��դ������
		GDALAllRegister();
		//��Ҫ���µ����ݣ�ע��ڶ�������ʹ��GA_UPDATE
		GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
		if (poDS == NULL)
		{
			std::cout << "��ͼ��" << strImageName << "ʧ��" << std::endl;
		}

		//��ȡͼ���С
		int iWidth = poDS->GetRasterXSize();
		int iHeight = poDS->GetRasterYSize();

		//����һ������ֵ��Vector,���Ը�TIF�ļ�ÿ�и�ֵ
		std::vector<float> imagePerLineVector;
		imagePerLineVector.clear();

		//��ȡ��һ����
		GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
		pBand1->SetNoDataValue(-9999);	//ȥ��-9999ֵ

		//ѭ��ͼ��ߣ�����ͼ�����������ֵ
		rasterVec3Set::iterator
			iterRasterCur = theRasterSet.begin(),
			iterRasterEnd = theRasterSet.end();
		for (; iterRasterCur != iterRasterEnd; iterRasterCur++)
		{
			//���µߵ���
			int id = iterRasterCur->first;
			std::vector<Pt3> theLine = iterRasterCur->second;

			//��������ڴ�
			imagePerLineVector.clear();
			std::vector<Pt3>::iterator
				iterCurLine = theLine.begin(),
				iterEndLine = theLine.end();
			for (; iterCurLine != iterEndLine; iterCurLine++)
			{
				Pt3 theRasterPoint = *iterCurLine;
				double theValue = theRasterPoint.z();
				//���������ظ��Ҷ�ֵ��VECTOR
				imagePerLineVector.push_back(theValue);
			}

			//����Ԫֵд��ͼ��
			pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, (float*)&imagePerLineVector[0], iWidth, 1, GDT_Float32, 0, 0);
		}
		//��������ڴ�
		imagePerLineVector.clear();

		GDALClose((GDALDatasetH)poDS);
	}

	//ͨ����ȡtif�ļ�����դ�񼯺�
	rasterVec3Set getRasterSetFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{
		rasterVec3Set rasterSet;
		rasterSet.clear();

		//�趨֧������·��
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//ע��դ������
		GDALAllRegister();
		//ʹ��ֻ����ʽ��ͼ��
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "���ܴ�!" << std::endl;
			return rasterSet;
		}

		xSize = poDataSet->GetRasterXSize();
		ySize = poDataSet->GetRasterYSize();

		//���ͼ�������ͷֱ�����Ϣ
		double adfGeoTransform[6];
		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5] * (-1);
		}

		//��ȡ��һ������
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);

		for (int i = 0; i < ySize; i++)
		{
			std::vector<Pt3> outputVec;
			outputVec.clear();
			std::vector<float> testFloat;
			testFloat.clear();
			testFloat.resize(xSize);
			//��ȡͼ��ĵ�һ������
			poBand->RasterIO(GF_Read, 0, i, xSize, 1, (float*)&testFloat[0], xSize, 1, GDT_Float32, 0, 0);

			for (int j = 0; j < xSize; j++)
			{
				double x = topLeftX + xResolution * j;
				double y = topLeftY + yResolution * i;
				double z = testFloat[j];
				Pt3 thisPoint = Pt3(x, y, z);
				outputVec.push_back(thisPoint);
			}
			rasterSet.insert(rasterVec3Pair(i, outputVec));
		}

		GDALClose((GDALDatasetH)poDataSet);
		return rasterSet;
	}

	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile(const char* strImageName, std::vector<std::vector<Pt3>> rasterVecVec)
	{
		//�趨֧������·��
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//ע��դ������
		GDALAllRegister();
		//��Ҫ���µ����ݣ�ע��ڶ�������ʹ��GA_UPDATE
		GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
		if (poDS == NULL)
		{
			std::cout << "��ͼ��" << strImageName << "ʧ��" << std::endl;
		}

		//��ȡͼ���С
		int iWidth = poDS->GetRasterXSize();
		int iHeight = poDS->GetRasterYSize();

		//����һ������ֵ��Vector,���Ը�TIF�ļ�ÿ�и�ֵ
		std::vector<float> imagePerLineVector;
		imagePerLineVector.clear();

		//��ȡ��һ����
		GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
		pBand1->SetNoDataValue(-9999);	//ȥ��-9999ֵ

		//ѭ��ͼ��ߣ�����ͼ�����������ֵ
		for ( int i = 0; i < iHeight; i++)
		{
			
			std::vector<Pt3> theLine = rasterVecVec[i];

			//��������ڴ�
			imagePerLineVector.clear();
			std::vector<Pt3>::iterator
				iterCurLine = theLine.begin(),
				iterEndLine = theLine.end();
			for (; iterCurLine != iterEndLine; iterCurLine++)
			{
				Pt3 theRasterPoint = *iterCurLine;
				double theValue = theRasterPoint.z();
				//���������ظ��Ҷ�ֵ��VECTOR
				imagePerLineVector.push_back(theValue);
			}

			//����Ԫֵд��ͼ��
			pBand1->RasterIO(GF_Write, 0, i, iWidth, 1, (float*)&imagePerLineVector[0], iWidth, 1, GDT_Float32, 0, 0);
				
		}
		//����ڴ�
		imagePerLineVector.clear();
		GDALClose((GDALDatasetH)poDS);
	}

	//ͨ����ȡtif�ļ�����դ�񼯺�
	std::vector<std::vector<Pt3>> getRasterVecVecFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{
		std::vector<std::vector<Pt3>> rasterVecVec;
		rasterVecVec.clear();

		//�趨֧������·��
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//ע��դ������
		GDALAllRegister();
		//ʹ��ֻ����ʽ��ͼ��
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "���ܴ�!" << std::endl;
			return rasterVecVec;
		}

		xSize = poDataSet->GetRasterXSize();
		ySize = poDataSet->GetRasterYSize();

		//���ͼ�������ͷֱ�����Ϣ
		double adfGeoTransform[6];
		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5] * (-1);
		}

		//��ȡ��һ������
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);

		rasterVecVec.resize(ySize);
		for (int i = 0; i < ySize; i++)
		{
			std::vector<Pt3> outputVec;
			outputVec.clear();
			std::vector<float> testFloat;
			testFloat.clear();
			testFloat.resize(xSize);
			//��ȡͼ��ĵ�һ������
			poBand->RasterIO(GF_Read, 0, i, xSize, 1, (float*)&testFloat[0], xSize, 1, GDT_Float32, 0, 0);

			for (int j = 0; j < xSize; j++)
			{
				double x = topLeftX + xResolution * j;
				double y = topLeftY + yResolution * i;
				double z = testFloat[j];
				Pt3 thisPoint = Pt3(x, y, z);
				outputVec.push_back(thisPoint);
			}
			rasterVecVec[i] = outputVec;
		}

		GDALClose((GDALDatasetH)poDataSet);
		return rasterVecVec;
	}

	//ͨ������tif,д����
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName)
	{

		int xSize1 = 0;
		int ySize1 = 0;
		double xResolution1 = 0;
		double yResolution1 = 0;
		double topLeftX1 = 0;
		double topLeftY1 = 0;
		//��TIF������ά����
		std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strInputTifName,
			xSize1, ySize1,
			xResolution1, yResolution1,
			topLeftX1, topLeftY1);

		//д����
		pcl::PointCloud<pcl::PointXYZ> cloudOutput;
		cloudOutput.clear();

		for (int i = 0; i < ySize1 / 5; i++)
		{
			for (int j = 0; j < xSize1; j++)
			{
				Pt3 thePt = rastervecvec1[i][j];
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

	}
}