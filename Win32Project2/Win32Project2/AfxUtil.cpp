#include "AfxUtil.h"
#include "PCLTif.h"
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
	
	//ͨ��դ�񴴽�TIF�ļ�
	void createRasterFile_8bit(const char* strImageName, int bandSize, int xSize, int ySize,
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
		GDALDataset * poDS = poDriver->Create(strImageName, xSize, ySize, bandSize, GDT_Byte, NULL);
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
		for (int i = 0; i < iHeight; i++)
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
	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile_8bit(const char* strImageName, std::vector<std::vector<Pt8Bit>> rasterVecVec)
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
		std::vector<uchar> imagePerLineVector;
		imagePerLineVector.clear();

		//��ȡ��һ����
		GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
		pBand1->SetNoDataValue(-9999);	//ȥ��-9999ֵ

		//ѭ��ͼ��ߣ�����ͼ�����������ֵ
		for (int i = 0; i < iHeight; i++)
		{

			std::vector<Pt8Bit> theLine = rasterVecVec[i];

			//��������ڴ�
			imagePerLineVector.clear();
			std::vector<Pt8Bit>::iterator
				iterCurLine = theLine.begin(),
				iterEndLine = theLine.end();
			for (; iterCurLine != iterEndLine; iterCurLine++)
			{
				Pt8Bit theRasterPoint = *iterCurLine;
				int theValue = theRasterPoint.z;
				//���������ظ��Ҷ�ֵ��VECTOR
				imagePerLineVector.push_back(theValue);
			}

			//����Ԫֵд��ͼ��
			pBand1->RasterIO(GF_Write, 0, i, iWidth, 1, (uchar*)&imagePerLineVector[0], iWidth, 1, GDT_Byte, 0, 0);

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
			yResolution = adfGeoTransform[5];
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


	//ͨ����ȡtif�ļ�����դ�񼯺�(8λ��
	std::vector<std::vector<Pt8Bit>> getRasterVecVecFromTif_8bit(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{
		std::vector<std::vector<Pt8Bit>> rasterVecVec;
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
			yResolution = adfGeoTransform[5];
		}

		//��ȡ��һ������
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);

		rasterVecVec.resize(ySize);
		for (int i = 0; i < ySize; i++)
		{
			std::vector<Pt8Bit> outputVec;
			outputVec.clear();
			std::vector<uchar> testIntVec;
			testIntVec.clear();
			testIntVec.resize(xSize);
			//��ȡͼ��ĵ�һ������
			poBand->RasterIO(GF_Read, 0, i, xSize, 1, (uchar*)&testIntVec[0], xSize, 1, GDT_Byte, 0, 0);

			for (int j = 0; j < xSize; j++)
			{
				double x = topLeftX + xResolution * j;
				double y = topLeftY + yResolution * i;
				int z = testIntVec[j];
				Pt8Bit thisPoint;
				thisPoint.x = x;
				thisPoint.y = y;
				thisPoint.z = z;
				outputVec.push_back(thisPoint);
			}
			rasterVecVec[i] = outputVec;
		}

		GDALClose((GDALDatasetH)poDataSet);
		return rasterVecVec;
	}

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�8λ��
	std::vector<uchar> getSegRasterVecVecFromTif_8bit(const char* strImageName, int xID, int yID, int width, int height,
		double &topLeftX, double &topLeftY, double &xResolu, double &yResolu)
	{
		std::vector<uchar> rasterVecVec;
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

		int xSize = poDataSet->GetRasterXSize();
		int ySize = poDataSet->GetRasterYSize();
		//���ܳ�����ֵ
		int minXID = xID;
		int minYID = yID;
		int maxXID = minXID + width;
		int maxYID = minYID + height;
		if ( maxXID  > xSize - 1)
		{
			maxXID = xSize - 1;
		}
		if (maxYID> ySize - 1)
		{
			maxYID = ySize - 1;
		}
		if (minXID < 0)
		{
			minXID = 0;
		}
		if ( minYID < 0 )
		{
			minYID = 0;
		}
		//���ͼ�������ͷֱ�����Ϣ
		double adfGeoTransform[6];
		double xResolution = 0;
		double yResolution = 0;

		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5];
		}

		topLeftX = topLeftX + xID * xResolution;
		topLeftY = topLeftY + yID * yResolution;
		xResolu = xResolution;
		yResolu = yResolution;
		//��ȡͼ��
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);
		rasterVecVec.resize(width * height);
		poBand->RasterIO(GF_Read, minXID, minYID, width, height, (uchar*)&rasterVecVec[0], width, height, GDT_Byte, 0, 0);

		return rasterVecVec;
	}

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�8λ��
	std::vector<uchar> getSegRasterVecVecFromTifAndLeftTop_8bit(const char* strImageName, int width, int height, double inputleftTopX, double inputleftTopY, 
		int& xRoi, int& yRoi, double &outputLeftTopX, double& outputLeftTopY, double& xResolu, double& yResolu)
	{
		std::vector<uchar> rasterVecVec;
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

		//���ͼ�������ͷֱ�����Ϣ
		double adfGeoTransform[6];
		double xResolution = 0;
		double yResolution = 0;
		double topLeftX = 0;
		double topLeftY = 0;

		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5];
		}

		int xID = (inputleftTopX - topLeftX) / xResolution;
		int yID = (inputleftTopY - topLeftY) / yResolution;
		int xSize = poDataSet->GetRasterXSize();
		int ySize = poDataSet->GetRasterYSize();

		//���ܳ�����ֵ
		int minXID = xID;
		int minYID = yID;
		int maxXID = minXID + width;
		int maxYID = minYID + height;
		if (maxXID  > xSize - 1)
		{
			maxXID = xSize - 1;
		}
		if (maxYID> ySize - 1)
		{
			maxYID = ySize - 1;
		}
		if (minXID < 0)
		{
			minXID = 0;
		}
		if (minYID < 0)
		{
			minYID = 0;
		}

		//������ʼid
		xRoi = minXID;
		yRoi = minYID;
		outputLeftTopX = topLeftX;
		outputLeftTopY = topLeftY;
		xResolu = xResolution;
		yResolu = yResolution;
		//��ȡͼ��
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);
		rasterVecVec.resize(width * height);
		poBand->RasterIO(GF_Read, minXID, minYID, width, height, (uchar*)&rasterVecVec[0], width, height, GDT_Byte, 0, 0);
		return rasterVecVec;
	}

	//ͨ������tif,д����
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName, bool bOrganized)
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

		for (int i = 0; i < ySize1 ; i++)
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
		if (bOrganized == true)   //�������
		{
			cloudOutput.width = xSize1;
			cloudOutput.height = ySize1;
		}
		else  //�������
		{
			cloudOutput.width = cloudOutput.size();
			cloudOutput.height = 1;
		}

		cloudOutput.is_dense = false;
		cloudOutput.resize(cloudOutput.width * cloudOutput.height);

		pcl::io::savePCDFileASCII(strOutPutPointCloudName, cloudOutput);

		rastervecvec1.clear();
		cloudOutput.clear();

	}

	//ͨ���������дtif
	//�ȸ��ݵ��ƴ�С������.tif,�ٷֿ鴦��ÿ�飬���ǻ���ֵ�����
	void writeTifFromPointCloud(const char* strInputPointCloudName, const char* strOutPutTifName, double xResolution, double yResolution, int bandSize)
	{

		//��¼��ʼʱ�ĵ�ǰʱ��ͽ���ʱ��
		time_t startTime, endTime;
		time_t readPointCloudTime;  //��ȡ����ʱ��
		time_t triangulationTime;	//���ǻ�ʱ��
		time_t chazhiTime;			//��ֵդ��ʱ��
		time_t createTifTime;		//����.tiffʱ��
		time_t preUpdateTif;		//����.tiff֮ǰʱ��
		time_t afterUpdateTif;		//����.tiff֮��ʱ��

		time(&startTime);
		PCLTif * thePCLTif1 = new PCLTif(strInputPointCloudName, xResolution, yResolution);
		time(&readPointCloudTime);
		double timeofreadPointCloudTime = difftime(readPointCloudTime, startTime);
		std::cout << "�����ƺ�ʱ" << timeofreadPointCloudTime << std::endl;
		//���������ļ�
		//�������Ͻ�Ϊminx,maxY
		PCLDetail theDetail = thePCLTif1->getDetailOfthePointCloud();
		//��X,Y��������ظ���,
		double xSizef = abs(theDetail.xDistance / xResolution);
		double ySizef = abs(theDetail.yDistance / yResolution);
		//ȡ��
		int xSize = (int)xSizef;
		int ySize = (int)ySizef;
		double topLeftX = theDetail.leftTopX;
		double topLeftY = theDetail.leftTopY;
		//����.tiff�ļ�
		util::createRasterFile(strOutPutTifName, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
		time(&createTifTime);
		double timeofcreateTifTime = difftime(createTifTime, readPointCloudTime);
		std::cout << "����.tif�ļ���ʱ" << timeofcreateTifTime << std::endl;
		//����ÿ��
		thePCLTif1->process();
		std::vector<std::vector<Pt3>> rastervec = thePCLTif1->getRasterVecVec3();

		//���������ļ�,
		time(&preUpdateTif);
		util::UpdateRasterFile(strOutPutTifName, rastervec);
		time(&afterUpdateTif);
		double timeofUpdateTif = difftime(afterUpdateTif, preUpdateTif);
		std::cout << "���������ļ���ʱ" << timeofUpdateTif << std::endl;
		double allTime = difftime(afterUpdateTif, startTime);
		std::cout << "�ܹ���ʱ" << allTime << std::endl;

	}

	//����.tif�ļ���������Χ���õ��µ�vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromTif(std::string strTifFileName, int xRoil, int yRoil, int width, int height)
	{
		//�����һ����
		int xSize1 = 0;
		int ySize1 = 0;
		double xResolution1 = 0;
		double yResolution1 = 0;
		double topLeftX1 = 0;
		double topLeftY1 = 0;
		std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strTifFileName.c_str(),
			xSize1, ySize1,
			xResolution1, yResolution1,
			topLeftX1, topLeftY1);

		//��ֵ
		std::vector<std::vector<Pt3>> selectRaster = getSegRasterVecVecFromVecVec2(rastervecvec1, xRoil, yRoil, width, height);
		rastervecvec1.clear();
		return selectRaster;
	}
	//����.tif�����е��vector������Χ���õ��µ�vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromVecVec2(std::vector<std::vector<Pt3>> inputVecVec, int xRoil, int yRoil, int width, int height)
	{
		//��ֵ
		std::vector<std::vector<Pt3>> selectRaster;
		selectRaster.clear();
		selectRaster.resize(height);
		for (size_t i = 0; i < height; i++)
		{
			selectRaster[i].resize(width);
		}

		for (size_t j = 0; j < height; j++)
		{
			for (size_t i = 0; i < width; i++)
			{
				double x = inputVecVec[j + yRoil][i + xRoil].x();
				double y = inputVecVec[j + yRoil][i + xRoil].y();
				double z = inputVecVec[j + yRoil][i + xRoil].z();

				//std::cout << "j=" << j << ",i=" << i << ",x=" << x << ",y=" << y << ",z=" << z << std::endl;
				Pt3 thePt(x, y, z);
				selectRaster[j][i] = thePt;
			}

		}
		return selectRaster;
	}
	//����
	void getDifTifBetweenTwoTifs(const char* strInputTifName1, const char* strInputTifName2, const char* strOutPutTifName)
	{
		int xSize1 = 0;
		int ySize1 = 0;
		double xResolution1 = 0;
		double yResolution1 = 0;
		double topLeftX1 = 0;
		double topLeftY1 = 0;
		std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strInputTifName1,
			xSize1, ySize1,
			xResolution1, yResolution1,
			topLeftX1, topLeftY1);

		std::cout << yResolution1 << std::endl;
		int xSize2 = 0;
		int ySize2 = 0;
		double xResolution2 = 0;
		double yResolution2 = 0;
		double topLeftX2 = 0;
		double topLeftY2 = 0;
		std::vector<std::vector<Pt3>> rastervecvec2 = util::getRasterVecVecFromTif(strInputTifName2,
			xSize2, ySize2,
			xResolution2, yResolution2,
			topLeftX2, topLeftY2);
		std::cout << yResolution2 << std::endl;

		std::vector<std::vector<Pt3>> outputVecVec;
		outputVecVec.clear();
		outputVecVec.resize(ySize2);
		for (int i = 0; i < ySize2; i++)
		{
			outputVecVec[i].resize(xSize2);
			for (int j = 0; j < xSize2; j++)
			{
				double x = rastervecvec2[i][j].x();
				double y = rastervecvec2[i][j].y();
				double z = -9999;
				Pt3 thePt(x, y, z);
				outputVecVec[i][j] = thePt;
			}
		}

		//����������ֵĽ���(minx,miny)�����ֵ,(max,maxy)����Сֵ
		double minX = topLeftX1;
		if (minX < topLeftX2)
		{
			minX = topLeftX2;
		}
		double minY = topLeftY1 + (ySize1 - 1) * yResolution1;
		if (minY < topLeftX2 + (ySize2 - 1) * yResolution2)
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

		int minX_ID1 = (minX - topLeftX1) / xResolution1;
		int minY_ID1 = (minY - topLeftY1) / yResolution1;
		int maxX_ID1 = (maxX - topLeftX1) / xResolution1;
		int maxY_ID1 = (maxY - topLeftY1) / yResolution1;

		for (int i = maxY_ID1; i <= minY_ID1; i++)
		{
			for (int j = minX_ID1; j <= maxX_ID1; j++)
			{
				double x = topLeftX1 + xResolution1 * j;
				double y = topLeftY1 + i * yResolution1;
				int XID2 = (x - topLeftX2) / xResolution2;
				int yID2 = (y - topLeftY2) / yResolution2;

				double value1 = rastervecvec1[i][j].z();
				double value2 = rastervecvec2[yID2][XID2].z();
				if (value2 != 0 && value1 != 0)
				{
					double difvalue = value2 - value1 + 140;
					Pt3 difPt = Pt3(x, y, difvalue);
					outputVecVec[yID2][XID2] = difPt;

				}
			}
		}

		//�����ͼ��
		util::createRasterFile(strOutPutTifName, 1, xSize2, ySize2, xResolution2, yResolution2, topLeftX2, topLeftY2);

		util::UpdateRasterFile(strOutPutTifName, outputVecVec);

		rastervecvec1.clear();
		rastervecvec2.clear();
		outputVecVec.clear();


	}

	//�˲�TIF
	void filterTif(const char* strInputTifName, const char* strOutPutTifName)
	{
		//��ȡԴͼ��ת��Ϊ�Ҷ�ͼ��
		cv::Mat srcImage = cv::imread(strInputTifName, cv::ImreadModes::IMREAD_LOAD_GDAL);
		//����ṹԪ��
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
		cv::Mat openMat;
		cv::morphologyEx(srcImage, openMat, cv::MORPH_OPEN, element);
		int row = openMat.rows;
		int col = openMat.cols;

		std::vector<std::vector<Pt3>> rasterOutput;
		rasterOutput.clear();
		rasterOutput.resize(row);
		for (int i = 0; i < row; i++)
		{
			rasterOutput[i].resize(col);
		}
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				float theVaue = openMat.at<float>(i, j);
				Pt3 thePt(j, i, theVaue);
				rasterOutput[i][j] = thePt;
			}
		}

		//���TIF

		int xSize1 = 0;
		int ySize1 = 0;
		double xResolution1 = 0;
		double yResolution1 = 0;
		double topLeftX1 = 0;
		double topLeftY1 = 0;
		std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(strInputTifName,
			xSize1, ySize1,
			xResolution1, yResolution1,
			topLeftX1, topLeftY1);

		xSize1 = col;
		ySize1 = row;
		util::createRasterFile(strOutPutTifName, 1, xSize1, ySize1, xResolution1, yResolution1, topLeftX1, topLeftY1);
		util::UpdateRasterFile(strOutPutTifName, rasterOutput);
	}
	//�˲�����
	void filterPCD(const char* strInputPCDName, const char* strOutPutPCDName)
	{
		//�ӵ��ƻ�õ㼯
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;

		pcl::PCDReader reader;
		//��ȡ��������
		std::cout << "��ȡ��������:" << std::endl;
		reader.read(strInputPCDName, *inputCloud);

		filter.setInputCloud(inputCloud);
		filter.setRadiusSearch(10);
		filter.setMinNeighborsInRadius(2);
		filter.filter(*filteredCloud);

		pcl::PCDWriter writer;
		writer.write(strOutPutPCDName, *filteredCloud);
	}
	//����.las��д����
	void writePointCloudFromLas(const char* strInputLasName, const char* strOutPutPointCloudName)
	{
		//��las�ļ�
		std::ifstream ifs;
		ifs.open(strInputLasName, std::ios::in | std::ios::binary);

		liblas::ReaderFactory readerFactory;
		liblas::Reader reader = readerFactory.CreateWithStream(ifs);

		//д����
		pcl::PointCloud<pcl::PointXYZ> cloudOutput;
		cloudOutput.clear();

		while (reader.ReadNextPoint())
		{
			double x = reader.GetPoint().GetX();
			double y = reader.GetPoint().GetY();
			double z = reader.GetPoint().GetZ();

			pcl::PointXYZ thePt(x, y, z);
			cloudOutput.push_back(thePt);

		}

		cloudOutput.width = cloudOutput.size();
		cloudOutput.height = 1;
		cloudOutput.is_dense = false;
		cloudOutput.resize(cloudOutput.width * cloudOutput.height);

		pcl::io::savePCDFileASCII("E:\\DEM-2016_2.pcd", cloudOutput);

		cloudOutput.clear();
	}
	//������ƣ�д.las
	void writeLasFromPointCloud(const char* strInputPointCloudName, const char* strOutLasName)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDReader pcdreader;
		pcdreader.read(strInputPointCloudName, *cloud);

		std::cout << "����:" << cloud->points.size() << std::endl;
		//дliblas,

		std::ios::openmode m = std::ios::out | std::ios::in | std::ios::binary | std::ios::ate;

		std::ofstream ofs;
		if (!liblas::Create(ofs, strOutLasName))
		{
			std::cout << "�޷�����.las" << std::endl;
			return;
		}
		ofs.close();

		std::ofstream * ofs2 = new std::ofstream(strOutLasName, m);
		if (!ofs2->is_open())
		{
			std::cout << "�򲻿�.las" << std::endl;
			return;
		}
		else
		{
			std::cout << "�ܴ�.las" << std::endl;

		}

		liblas::Header header;
		liblas::Writer writer(*ofs2, header);
		liblas::Point point(&header);
	
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			double x = cloud->points[i].x;
			double y = cloud->points[i].y;
			double z = cloud->points[i].z;

			point.SetX(x);
			point.SetY(y);
			point.SetZ(z);
			writer.WritePoint(point);
			std::cout << x << "," << y << "," << z << std::endl;
		}
		
	}

	//���ݵ������ƣ�����Ƶ��е㼰���x,y,z����
	void getPCLMidPointAndDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xResolution, double yResolution, Pt3& midPoint, double& maxDistance, double &minZ, double& maxZ, int &xSize, int &ySize)
	{
		std::cout << "����:" << cloud->points.size() << std::endl;
		double minX = cloud->points[0].x;
		double minY = cloud->points[0].y;
		minZ = cloud->points[0].z;
		double maxX = cloud->points[0].x;
		double maxY = cloud->points[0].y;
		maxZ = cloud->points[0].z;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			double x = cloud->points[i].x;
			double y = cloud->points[i].y;
			double z = cloud->points[i].z;
			if (x < minX)
			{
				minX = x;
			}
			if (x > maxX)
			{
				maxX = x;
			}
			if (y < minY)
			{
				minY = y;
			}
			if (y > maxY)
			{
				maxY = y;
			}
			if (z < minZ)
			{
				minZ = z;
			}
			if (z > maxZ)
			{
				maxZ = z;
			}
			
		}
		//ȡ��ֵ
		double midX = (minX + maxX) / 2.0;
		double midY = (minY + maxY) / 2.0;
		double midZ = (minZ + maxZ) / 2.0;

		midPoint = Pt3(midX, midY, midZ);

		double distanceX = abs( maxX - minX );
		double distanceY = abs( maxY - minY );
		double distanceZ = abs( maxZ - minZ );
		
		maxDistance = distanceX;
		if ( distanceY > maxDistance)
		{
			maxDistance = distanceY;
		}
		if (distanceZ > maxDistance)
		{
			maxDistance = distanceZ;
		}

		//��X,Y��������ظ���,
		 xSize = (int) abs( distanceX / xResolution );
		 ySize = (int) abs(distanceY / yResolution);
	
	}
	//�ӵ�һ��ͼ��ȡ��Χ�����Ͻ�(xRoi,yRoi)�������һ��ͼ��Ľ�ȡ��Χ�����Ͻ�(xRoi2,yRoi2)
	void getRoil2FromRoi1AndTif(int widthRoil, int heightRoil, double inputTopLeftX, double inputTopLeftY,
		double xResolution2, double yResolution2, double topLeftX2, double topLeftY2, int xSize2, int ySize2,
		int& xRoi2, int& yRoi2)
	{

		int xID = (inputTopLeftX - topLeftX2) / xResolution2;
		int yID = (inputTopLeftY - topLeftY2) / yResolution2;

		//���ܳ�����ֵ
		int minXID = xID;
		int minYID = yID;
		int maxXID = minXID + widthRoil;
		int maxYID = minYID + heightRoil;
		if (maxXID  > xSize2 - 1)
		{
			maxXID = xSize2 - 1;
		}
		if (maxYID> ySize2 - 1)
		{
			maxYID = ySize2 - 1;
		}
		if (minXID < 0)
		{
			minXID = 0;
		}
		if (minYID < 0)
		{
			minYID = 0;
		}
		if ( minXID >= maxXID )
		{
			minXID = maxXID;
		}
		if (minYID >= maxYID)
		{
			minYID = maxYID;
		}

		//������ʼid
		xRoi2 = minXID;
		yRoi2 = minYID;
	}

	//�õ����ص�����
	std::vector<float> getPixel32bitFromTifVecVec(std::vector<std::vector<Pt3>> vecvec)
	{
		std::vector<float> pixel32Vec;
		pixel32Vec.clear();
		for (size_t j = 0; j < vecvec.size(); j++)
		{
			for (size_t i = 0; i < vecvec[j].size(); i++)
			{
				float thePixel32 = vecvec[j][i].z();
				pixel32Vec.push_back(thePixel32);
			}

		}
		return pixel32Vec;
	}

	//����.tif���Ƶõ���.tif�ĸ�Ҫ��
	void getDetailFromTifName(std::string strTifName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{

		//�趨֧������·��
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//ע��դ������
		GDALAllRegister();
		//ʹ��ֻ����ʽ��ͼ��
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strTifName.c_str(), GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strTifName << "���ܴ�!" << std::endl;
			return;
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
			yResolution = adfGeoTransform[5];
		}


		GDALClose((GDALDatasetH)poDataSet);
	}

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�32λ��
	std::vector<float> getSegRasterVecVecFromTif_32bit(const char* strImageName, int xID, int yID, int width, int height)
	{
		std::vector<float> rasterVecVec;
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

		//��ȡͼ��
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);
		rasterVecVec.resize(width * height);
		poBand->RasterIO(GF_Read, xID, yID, width, height, (float*)&rasterVecVec[0], width, height, GDT_Float32, 0, 0);
		GDALClose(poDataSet);

		return rasterVecVec;
	}

	std::vector<float> getSegRasterVecVecFromTif_32bit(const char* strImageName, zone theZone)
	{
		int xID = theZone.xRoi;
		int yID = theZone.yRoi;
		int width = theZone.widthRoi;
		int height = theZone.heightRoi;
		
		std::vector<float> rasterVec32bit = getSegRasterVecVecFromTif_32bit(strImageName, xID, yID, width, height);
		return rasterVec32bit;
	}
	//����.tif���Ƶõ���.tif�ĸ�Ҫ��
	void getTifParameterFromTifName(std::string strTifName,
		tifParameter & theTifParameter)
	{
		int xSize = 0;
		int ySize = 0;
		double  xResolution = 0;
		double  yResolution = 0;
		double  topLeftX = 0;
		double  topLeftY = 0;
		util::getDetailFromTifName(strTifName, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
		
		theTifParameter.xSize = xSize;
		theTifParameter.ySize = ySize;
		theTifParameter.xResolution = xResolution;
		theTifParameter.yResolution = yResolution;
		theTifParameter.leftTopX = topLeftX;
		theTifParameter.leftTopY = topLeftY;

	}

	//�ӵ�һ��ͼ��ȡ��Χ�Ĳ����������һ��ͼ��Ľ�ȡ��Χ�Ĳ���
	void getZone2FromZone1(zone zone1, tifParameter tif1, tifParameter tif2, zone& zone2)
	{
		int xRoi1 = zone1.xRoi;
		int yRoi1 = zone1.yRoi;
		int widthRoi1 = zone1.widthRoi;
		int heightRoi1 = zone1.heightRoi;

		double topLeftX1 = tif1.leftTopX;
		double topLeftY1 = tif1.leftTopY;
		double xResolution1 = tif1.xResolution;
		double yResolution1 = tif1.yResolution;

		double topLeftX2 = tif2.leftTopX;
		double topLeftY2 = tif2.leftTopY;
		double xResolution2 = tif2.xResolution;
		double yResolution2 = tif2.yResolution;
		int xSize2 = tif2.xSize;
		int ySize2 = tif2.ySize;

		double topLeftXRoi = topLeftX1 + xResolution1 * xRoi1;
		double topLeftYRoi = topLeftY1 + yResolution1 * yRoi1;

		int minXID = (topLeftXRoi - topLeftX2) / xResolution2;
		int minYID = (topLeftYRoi - topLeftY2) / yResolution2;
		std::cout << "minYID = " << minYID << std::endl;
		int maxXID = minXID + widthRoi1;
		int maxYID = minYID + heightRoi1;
		//���ܳ�����ֵ
		if (maxXID  > xSize2 - 1)
		{
			maxXID = xSize2 - 1;
		}
		if (maxYID> ySize2 - 1)
		{
			maxYID = ySize2 - 1;
		}
		if (minXID < 0)
		{
			minXID = 0;
		}
		if (minYID < 0)
		{
			minYID = 0;
		}
		if (minXID >= maxXID)
		{
			minXID = maxXID;
		}
		if (minYID >= maxYID)
		{
			minYID = maxYID;
		}

		//��������Χ
		zone2.xRoi = minXID;
		zone2.yRoi = minYID;
		zone2.widthRoi = maxXID - minXID;
		zone2.heightRoi = maxYID - minYID;
	}
	
	//��x�Ӵ�С����
	bool greaterSortX(diffVec a, diffVec b)
	{
		return a.diffPt.x() > b.diffPt.x();
	}
	//��y�Ӵ�С����
	bool greaterSortY(diffVec a, diffVec b)
	{
		return a.diffPt.y() > b.diffPt.y();
	}
	//��z�Ӵ�С����
	bool greaterSortZ(diffVec a, diffVec b)
	{
		return a.diffPt.z() > b.diffPt.z();
	}
}