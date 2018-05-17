#include "AfxUtil.h"
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
		for ( int i = 0; i < iHeight; i++)
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
			yResolution = adfGeoTransform[5] * (-1);
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

	//通过读入tif,写点云
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName)
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