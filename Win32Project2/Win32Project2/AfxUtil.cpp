#include "AfxUtil.h"
#include "PCLTif.h"
namespace util
{
	//通过栅格创建TIF文件
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX, double rotationY)
	{
		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//下面获取指定格式的驱动，用于创建图像
		const char * pszFormat = "GTiff";
		GDALDriver * poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		if (poDriver == NULL)
		{
			std::cout << "格式" << pszFormat << "不支持create()方法" << std::endl;
		}
		else
		{
			std::cout << "驱动OK！" << std::endl;
		}

		//获取该驱动的一些元数据信息
		char ** papszMetadata = poDriver->GetMetadata();
		if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
		{
			std::cout << pszFormat << "支持Create()方法" << std::endl;
		}
		if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
		{
			std::cout << pszFormat << "支持CreateCopy()方法" << std::endl;
		}

		//创建输出文件，大小为512*512*3，三个波段8bit的数据
		GDALDataset * poDS = poDriver->Create(strImageName, xSize, ySize, bandSize, GDT_Float32, NULL);
		if (poDS == NULL)
		{
			std::cout << "创建图像" << strImageName << "失败" << std::endl;
			return;
		}
		//设置六参数，第一个和第四个是图像左上角的坐标，第2个和第6个为横向和纵向分辨率，余下两个为旋转角度
		double dGeotransform[6] = { 0, 0, 0, 0, 0, 0 };
		dGeotransform[0] = topLeftX;   //左上坐标X
		dGeotransform[1] = xResolution;	//横向分辨率
		dGeotransform[2] = rotationX;	//旋转角度
		dGeotransform[3] = topLeftY;	//左上坐标Y
		dGeotransform[4] = rotationY;	//旋转角度
		dGeotransform[5] = yResolution;	//y分辨率（负值）

		poDS->SetGeoTransform(dGeotransform);

		GDALClose((GDALDatasetH)poDS);
		std::cout << "数据集关闭！" << std::endl;


	}
	
	//通过栅格创建TIF文件
	void createRasterFile_8bit(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX, double rotationY)
	{
		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//下面获取指定格式的驱动，用于创建图像
		const char * pszFormat = "GTiff";
		GDALDriver * poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		if (poDriver == NULL)
		{
			std::cout << "格式" << pszFormat << "不支持create()方法" << std::endl;
		}
		else
		{
			std::cout << "驱动OK！" << std::endl;
		}

		//获取该驱动的一些元数据信息
		char ** papszMetadata = poDriver->GetMetadata();
		if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
		{
			std::cout << pszFormat << "支持Create()方法" << std::endl;
		}
		if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
		{
			std::cout << pszFormat << "支持CreateCopy()方法" << std::endl;
		}

		//创建输出文件，大小为512*512*3，三个波段8bit的数据
		GDALDataset * poDS = poDriver->Create(strImageName, xSize, ySize, bandSize, GDT_Byte, NULL);
		if (poDS == NULL)
		{
			std::cout << "创建图像" << strImageName << "失败" << std::endl;
			return;
		}
		//设置六参数，第一个和第四个是图像左上角的坐标，第2个和第6个为横向和纵向分辨率，余下两个为旋转角度
		double dGeotransform[6] = { 0, 0, 0, 0, 0, 0 };
		dGeotransform[0] = topLeftX;   //左上坐标X
		dGeotransform[1] = xResolution;	//横向分辨率
		dGeotransform[2] = rotationX;	//旋转角度
		dGeotransform[3] = topLeftY;	//左上坐标Y
		dGeotransform[4] = rotationY;	//旋转角度
		dGeotransform[5] = yResolution;	//y分辨率（负值）

		poDS->SetGeoTransform(dGeotransform);

		GDALClose((GDALDatasetH)poDS);
		std::cout << "数据集关闭！" << std::endl;


	}

	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
	{
		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//打开要更新的数据，注意第二个参数使用GA_UPDATE
		GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
		if (poDS == NULL)
		{
			std::cout << "打开图像" << strImageName << "失败" << std::endl;
		}

		//获取图像大小
		int iWidth = poDS->GetRasterXSize();
		int iHeight = poDS->GetRasterYSize();

		//声明一个像素值的Vector,用以给TIF文件每行赋值
		std::vector<float> imagePerLineVector;
		imagePerLineVector.clear();

		//获取第一波段
		GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
		pBand1->SetNoDataValue(-9999);	//去除-9999值

		//循环图像高，更新图像里面的像素值
		rasterVec3Set::iterator
			iterRasterCur = theRasterSet.begin(),
			iterRasterEnd = theRasterSet.end();
		for (; iterRasterCur != iterRasterEnd; iterRasterCur++)
		{
			//上下颠倒下
			int id = iterRasterCur->first;
			std::vector<Pt3> theLine = iterRasterCur->second;

			//清空两段内存
			imagePerLineVector.clear();
			std::vector<Pt3>::iterator
				iterCurLine = theLine.begin(),
				iterEndLine = theLine.end();
			for (; iterCurLine != iterEndLine; iterCurLine++)
			{
				Pt3 theRasterPoint = *iterCurLine;
				double theValue = theRasterPoint.z();
				//该行逐像素赋灰度值给VECTOR
				imagePerLineVector.push_back(theValue);
			}

			//将像元值写入图像
			pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, (float*)&imagePerLineVector[0], iWidth, 1, GDT_Float32, 0, 0);
		}
		//清空两段内存
		imagePerLineVector.clear();

		GDALClose((GDALDatasetH)poDS);
	}

	//通过读取tif文件创建栅格集合
	rasterVec3Set getRasterSetFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{
		rasterVec3Set rasterSet;
		rasterSet.clear();

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "不能打开!" << std::endl;
			return rasterSet;
		}

		xSize = poDataSet->GetRasterXSize();
		ySize = poDataSet->GetRasterYSize();

		//输出图像的坐标和分辨率信息
		double adfGeoTransform[6];
		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5] * (-1);
		}

		//读取第一个波段
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);

		for (int i = 0; i < ySize; i++)
		{
			std::vector<Pt3> outputVec;
			outputVec.clear();
			std::vector<float> testFloat;
			testFloat.clear();
			testFloat.resize(xSize);
			//读取图像的第一行数据
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

	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, std::vector<std::vector<Pt3>> rasterVecVec)
	{
		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//打开要更新的数据，注意第二个参数使用GA_UPDATE
		GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
		if (poDS == NULL)
		{
			std::cout << "打开图像" << strImageName << "失败" << std::endl;
		}

		//获取图像大小
		int iWidth = poDS->GetRasterXSize();
		int iHeight = poDS->GetRasterYSize();

		//声明一个像素值的Vector,用以给TIF文件每行赋值
		std::vector<float> imagePerLineVector;
		imagePerLineVector.clear();

		//获取第一波段
		GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
		pBand1->SetNoDataValue(-9999);	//去除-9999值

		//循环图像高，更新图像里面的像素值
		for (int i = 0; i < iHeight; i++)
		{

			std::vector<Pt3> theLine = rasterVecVec[i];

			//清空两段内存
			imagePerLineVector.clear();
			std::vector<Pt3>::iterator
				iterCurLine = theLine.begin(),
				iterEndLine = theLine.end();
			for (; iterCurLine != iterEndLine; iterCurLine++)
			{
				Pt3 theRasterPoint = *iterCurLine;
				double theValue = theRasterPoint.z();
				//该行逐像素赋灰度值给VECTOR
				imagePerLineVector.push_back(theValue);
			}

			//将像元值写入图像
			pBand1->RasterIO(GF_Write, 0, i, iWidth, 1, (float*)&imagePerLineVector[0], iWidth, 1, GDT_Float32, 0, 0);

		}
		//清空内存
		imagePerLineVector.clear();
		GDALClose((GDALDatasetH)poDS);
	}
	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile_8bit(const char* strImageName, std::vector<std::vector<Pt8Bit>> rasterVecVec)
	{
		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//打开要更新的数据，注意第二个参数使用GA_UPDATE
		GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
		if (poDS == NULL)
		{
			std::cout << "打开图像" << strImageName << "失败" << std::endl;
		}

		//获取图像大小
		int iWidth = poDS->GetRasterXSize();
		int iHeight = poDS->GetRasterYSize();

		//声明一个像素值的Vector,用以给TIF文件每行赋值
		std::vector<uchar> imagePerLineVector;
		imagePerLineVector.clear();

		//获取第一波段
		GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
		pBand1->SetNoDataValue(-9999);	//去除-9999值

		//循环图像高，更新图像里面的像素值
		for (int i = 0; i < iHeight; i++)
		{

			std::vector<Pt8Bit> theLine = rasterVecVec[i];

			//清空两段内存
			imagePerLineVector.clear();
			std::vector<Pt8Bit>::iterator
				iterCurLine = theLine.begin(),
				iterEndLine = theLine.end();
			for (; iterCurLine != iterEndLine; iterCurLine++)
			{
				Pt8Bit theRasterPoint = *iterCurLine;
				int theValue = theRasterPoint.z;
				//该行逐像素赋灰度值给VECTOR
				imagePerLineVector.push_back(theValue);
			}

			//将像元值写入图像
			pBand1->RasterIO(GF_Write, 0, i, iWidth, 1, (uchar*)&imagePerLineVector[0], iWidth, 1, GDT_Byte, 0, 0);

		}
		//清空内存
		imagePerLineVector.clear();
		GDALClose((GDALDatasetH)poDS);
	}

	//通过读取tif文件创建栅格集合
	std::vector<std::vector<Pt3>> getRasterVecVecFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{
		std::vector<std::vector<Pt3>> rasterVecVec;
		rasterVecVec.clear();

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "不能打开!" << std::endl;
			return rasterVecVec;
		}

		xSize = poDataSet->GetRasterXSize();
		ySize = poDataSet->GetRasterYSize();

		//输出图像的坐标和分辨率信息
		double adfGeoTransform[6];
		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5];
		}

		//读取第一个波段
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);

		rasterVecVec.resize(ySize);
		for (int i = 0; i < ySize; i++)
		{
			std::vector<Pt3> outputVec;
			outputVec.clear();
			std::vector<float> testFloat;
			testFloat.clear();
			testFloat.resize(xSize);
			//读取图像的第一行数据
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


	//通过读取tif文件创建栅格集合(8位）
	std::vector<std::vector<Pt8Bit>> getRasterVecVecFromTif_8bit(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{
		std::vector<std::vector<Pt8Bit>> rasterVecVec;
		rasterVecVec.clear();

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "不能打开!" << std::endl;
			return rasterVecVec;
		}

		xSize = poDataSet->GetRasterXSize();
		ySize = poDataSet->GetRasterYSize();

		//输出图像的坐标和分辨率信息
		double adfGeoTransform[6];
		if (poDataSet->GetGeoTransform(adfGeoTransform) == CE_None)
		{
			topLeftX = adfGeoTransform[0];
			topLeftY = adfGeoTransform[3];
			xResolution = adfGeoTransform[1];
			yResolution = adfGeoTransform[5];
		}

		//读取第一个波段
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);

		rasterVecVec.resize(ySize);
		for (int i = 0; i < ySize; i++)
		{
			std::vector<Pt8Bit> outputVec;
			outputVec.clear();
			std::vector<uchar> testIntVec;
			testIntVec.clear();
			testIntVec.resize(xSize);
			//读取图像的第一行数据
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

	//通过读取一部分.tif，创建栅格集合（8位）
	std::vector<uchar> getSegRasterVecVecFromTif_8bit(const char* strImageName, int xID, int yID, int width, int height,
		double &topLeftX, double &topLeftY, double &xResolu, double &yResolu)
	{
		std::vector<uchar> rasterVecVec;
		rasterVecVec.clear();

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "不能打开!" << std::endl;
			return rasterVecVec;
		}

		int xSize = poDataSet->GetRasterXSize();
		int ySize = poDataSet->GetRasterYSize();
		//不能超过阈值
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
		//输出图像的坐标和分辨率信息
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
		//读取图像
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);
		rasterVecVec.resize(width * height);
		poBand->RasterIO(GF_Read, minXID, minYID, width, height, (uchar*)&rasterVecVec[0], width, height, GDT_Byte, 0, 0);

		return rasterVecVec;
	}

	//通过读取一部分.tif，创建栅格集合（8位）
	std::vector<uchar> getSegRasterVecVecFromTifAndLeftTop_8bit(const char* strImageName, int width, int height, double inputleftTopX, double inputleftTopY, 
		int& xRoi, int& yRoi, double &outputLeftTopX, double& outputLeftTopY, double& xResolu, double& yResolu)
	{
		std::vector<uchar> rasterVecVec;
		rasterVecVec.clear();

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "不能打开!" << std::endl;
			return rasterVecVec;
		}

		//输出图像的坐标和分辨率信息
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

		//不能超过阈值
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

		//返回起始id
		xRoi = minXID;
		yRoi = minYID;
		outputLeftTopX = topLeftX;
		outputLeftTopY = topLeftY;
		xResolu = xResolution;
		yResolu = yResolution;
		//读取图像
		GDALRasterBand * poBand = poDataSet->GetRasterBand(1);
		rasterVecVec.resize(width * height);
		poBand->RasterIO(GF_Read, minXID, minYID, width, height, (uchar*)&rasterVecVec[0], width, height, GDT_Byte, 0, 0);
		return rasterVecVec;
	}

	//通过读入tif,写点云
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName, bool bOrganized)
	{

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
		if (bOrganized == true)   //有序点云
		{
			cloudOutput.width = xSize1;
			cloudOutput.height = ySize1;
		}
		else  //无序点云
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

	//通过读入点云写tif
	//先根据点云大小创建大.tif,再分块处理每块，三角化插值后填充
	void writeTifFromPointCloud(const char* strInputPointCloudName, const char* strOutPutTifName, double xResolution, double yResolution, int bandSize)
	{

		//记录开始时的当前时间和结束时间
		time_t startTime, endTime;
		time_t readPointCloudTime;  //读取点云时间
		time_t triangulationTime;	//三角化时间
		time_t chazhiTime;			//插值栅格时间
		time_t createTifTime;		//创建.tiff时间
		time_t preUpdateTif;		//更新.tiff之前时间
		time_t afterUpdateTif;		//更新.tiff之后时间

		time(&startTime);
		PCLTif * thePCLTif1 = new PCLTif(strInputPointCloudName, xResolution, yResolution);
		time(&readPointCloudTime);
		double timeofreadPointCloudTime = difftime(readPointCloudTime, startTime);
		std::cout << "读点云耗时" << timeofreadPointCloudTime << std::endl;
		//创建网格文件
		//设置左上角为minx,maxY
		PCLDetail theDetail = thePCLTif1->getDetailOfthePointCloud();
		//求X,Y方向的像素个数,
		double xSizef = abs(theDetail.xDistance / xResolution);
		double ySizef = abs(theDetail.yDistance / yResolution);
		//取整
		int xSize = (int)xSizef;
		int ySize = (int)ySizef;
		double topLeftX = theDetail.leftTopX;
		double topLeftY = theDetail.leftTopY;
		//生成.tiff文件
		util::createRasterFile(strOutPutTifName, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
		time(&createTifTime);
		double timeofcreateTifTime = difftime(createTifTime, readPointCloudTime);
		std::cout << "创建.tif文件耗时" << timeofcreateTifTime << std::endl;
		//处理每块
		thePCLTif1->process();
		std::vector<std::vector<Pt3>> rastervec = thePCLTif1->getRasterVecVec3();

		//更新网格文件,
		time(&preUpdateTif);
		util::UpdateRasterFile(strOutPutTifName, rastervec);
		time(&afterUpdateTif);
		double timeofUpdateTif = difftime(afterUpdateTif, preUpdateTif);
		std::cout << "更新网格文件耗时" << timeofUpdateTif << std::endl;
		double allTime = difftime(afterUpdateTif, startTime);
		std::cout << "总共耗时" << allTime << std::endl;

	}

	//根据.tif文件名和区域范围，得到新的vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromTif(std::string strTifFileName, int xRoil, int yRoil, int width, int height)
	{
		//先求出一部分
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

		//赋值
		std::vector<std::vector<Pt3>> selectRaster = getSegRasterVecVecFromVecVec2(rastervecvec1, xRoil, yRoil, width, height);
		rastervecvec1.clear();
		return selectRaster;
	}
	//根据.tif的所有点的vector和区域范围，得到新的vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromVecVec2(std::vector<std::vector<Pt3>> inputVecVec, int xRoil, int yRoil, int width, int height)
	{
		//赋值
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
	//作差
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

		//先求出两部分的交集(minx,miny)的最大值,(max,maxy)的最小值
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

		//输出新图像
		util::createRasterFile(strOutPutTifName, 1, xSize2, ySize2, xResolution2, yResolution2, topLeftX2, topLeftY2);

		util::UpdateRasterFile(strOutPutTifName, outputVecVec);

		rastervecvec1.clear();
		rastervecvec2.clear();
		outputVecVec.clear();


	}

	//滤波TIF
	void filterTif(const char* strInputTifName, const char* strOutPutTifName)
	{
		//读取源图像并转化为灰度图像
		cv::Mat srcImage = cv::imread(strInputTifName, cv::ImreadModes::IMREAD_LOAD_GDAL);
		//定义结构元素
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

		//输出TIF

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
	//滤波点云
	void filterPCD(const char* strInputPCDName, const char* strOutPutPCDName)
	{
		//从点云获得点集
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;

		pcl::PCDReader reader;
		//读取点云数据
		std::cout << "读取点云数据:" << std::endl;
		reader.read(strInputPCDName, *inputCloud);

		filter.setInputCloud(inputCloud);
		filter.setRadiusSearch(10);
		filter.setMinNeighborsInRadius(2);
		filter.filter(*filteredCloud);

		pcl::PCDWriter writer;
		writer.write(strOutPutPCDName, *filteredCloud);
	}
	//读入.las，写点云
	void writePointCloudFromLas(const char* strInputLasName, const char* strOutPutPointCloudName)
	{
		//打开las文件
		std::ifstream ifs;
		ifs.open(strInputLasName, std::ios::in | std::ios::binary);

		liblas::ReaderFactory readerFactory;
		liblas::Reader reader = readerFactory.CreateWithStream(ifs);

		//写点云
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
	//读入点云，写.las
	void writeLasFromPointCloud(const char* strInputPointCloudName, const char* strOutLasName)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDReader pcdreader;
		pcdreader.read(strInputPointCloudName, *cloud);

		std::cout << "总数:" << cloud->points.size() << std::endl;
		//写liblas,

		std::ios::openmode m = std::ios::out | std::ios::in | std::ios::binary | std::ios::ate;

		std::ofstream ofs;
		if (!liblas::Create(ofs, strOutLasName))
		{
			std::cout << "无法创建.las" << std::endl;
			return;
		}
		ofs.close();

		std::ofstream * ofs2 = new std::ofstream(strOutLasName, m);
		if (!ofs2->is_open())
		{
			std::cout << "打不开.las" << std::endl;
			return;
		}
		else
		{
			std::cout << "能打开.las" << std::endl;

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

	//根据点云名称，求点云的中点及最大x,y,z距离
	void getPCLMidPointAndDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xResolution, double yResolution, Pt3& midPoint, double& maxDistance, double &minZ, double& maxZ, int &xSize, int &ySize)
	{
		std::cout << "总数:" << cloud->points.size() << std::endl;
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
		//取中值
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

		//求X,Y方向的像素个数,
		 xSize = (int) abs( distanceX / xResolution );
		 ySize = (int) abs(distanceY / yResolution);
	
	}
	//从第一幅图截取范围的左上角(xRoi,yRoi)计算出另一幅图像的截取范围的左上角(xRoi2,yRoi2)
	void getRoil2FromRoi1AndTif(int widthRoil, int heightRoil, double inputTopLeftX, double inputTopLeftY,
		double xResolution2, double yResolution2, double topLeftX2, double topLeftY2, int xSize2, int ySize2,
		int& xRoi2, int& yRoi2)
	{

		int xID = (inputTopLeftX - topLeftX2) / xResolution2;
		int yID = (inputTopLeftY - topLeftY2) / yResolution2;

		//不能超过阈值
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

		//返回起始id
		xRoi2 = minXID;
		yRoi2 = minYID;
	}

	//得到像素点序列
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

	//根据.tif名称得到该.tif的各要素
	void getDetailFromTifName(std::string strTifName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY)
	{

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strTifName.c_str(), GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strTifName << "不能打开!" << std::endl;
			return;
		}

		xSize = poDataSet->GetRasterXSize();
		ySize = poDataSet->GetRasterYSize();

		//输出图像的坐标和分辨率信息
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

	//通过读取一部分.tif，创建栅格集合（32位）
	std::vector<float> getSegRasterVecVecFromTif_32bit(const char* strImageName, int xID, int yID, int width, int height)
	{
		std::vector<float> rasterVecVec;
		rasterVecVec.clear();

		//设定支持中文路径
		CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
		//注册栅格驱动
		GDALAllRegister();
		//使用只读方式打开图像
		GDALDataset * poDataSet = (GDALDataset *)GDALOpen(strImageName, GA_ReadOnly);
		if (poDataSet == NULL)
		{
			std::cout << strImageName << "不能打开!" << std::endl;
			return rasterVecVec;
		}

		//读取图像
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
	//根据.tif名称得到该.tif的各要素
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

	//从第一幅图截取范围的参数计算出另一幅图像的截取范围的参数
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
		//不能超过阈值
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

		//返回区域范围
		zone2.xRoi = minXID;
		zone2.yRoi = minYID;
		zone2.widthRoi = maxXID - minXID;
		zone2.heightRoi = maxYID - minYID;
	}
	
	//按x从大到小排列
	bool greaterSortX(diffVec a, diffVec b)
	{
		return a.diffPt.x() > b.diffPt.x();
	}
	//按y从大到小排列
	bool greaterSortY(diffVec a, diffVec b)
	{
		return a.diffPt.y() > b.diffPt.y();
	}
	//按z从大到小排列
	bool greaterSortZ(diffVec a, diffVec b)
	{
		return a.diffPt.z() > b.diffPt.z();
	}
}