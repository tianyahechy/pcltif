#pragma once
#include "common.h"

namespace util
{

	//通过栅格创建TIF文件
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);

	//通过栅格创建TIF文件(8位)
	void createRasterFile_8bit(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);

	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet);

	//通过读取tif文件创建栅格集合
	rasterVec3Set getRasterSetFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);

	//用数组更合适
	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, std::vector<std::vector<Pt3>> theRasterVecVec);
	//通过更改栅格数据更新TIFF文件(8bit)
	void UpdateRasterFile_8bit(const char* strImageName, std::vector<std::vector<Pt8Bit>> theRasterVecVec);
	//通过读取tif文件创建栅格集合
	std::vector<std::vector<Pt3>> getRasterVecVecFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);
	//通过读取tif文件创建栅格集合(8位）
	std::vector<std::vector<Pt8Bit>> getRasterVecVecFromTif_8bit(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);

	//通过读取一部分.tif，创建栅格集合（8位）
	std::vector<uchar> getSegRasterVecVecFromTif_8bit(const char* strImageName, int xID, int yID, int width, int height,
		double &outputleftTopX, double &outputleftTopY, double &xResolu, double &yResolu);

	//通过读取一部分.tif，创建栅格集合（8位）
	std::vector<uchar> getSegRasterVecVecFromTifAndLeftTop_8bit(const char* strImageName, int width, int height, double inputleftTopX, double inputleftTopY,
		int& xRoi2, int& yRoi2, double &outputLeftTopX, double& outputLeftTopY, double& xResolu, double& yResolu);

	//通过读入tif,写点云
	//bOrganized：是否是有序点云，true为有序，false为无序点云
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName, float invalidValue, bool bOrganized = false);
	//通过读入点云写tif
	void writeTifFromPointCloud(const char* strInputPointCloudName, const char* strOutPutTifName, double xResolution, double yResolution, int bandSize = 1 );
	//通过读入点云分块写tif
	void writeTifFromPointCloudBySegment(const char* strInputPointCloudName, 
		const char* strOutPutTifName, 
		int sizeofSegment,
		double xResolution, 
		double yResolution, 
		int bandSize = 1);

	//作差
	void getDifTifBetweenTwoTifs(const char* strInputTifName1, const char* strInputTifName2, const char* strOutPutTifName);
	//滤波.TIF
	void filterTif(const char* strInputTifName, const char* strOutPutTifName);
	//根据阈值滤波.TIF
	void filterTifByThreshold(const char* strInputTifName, const char* strOutPutTifName, double lowThreshold, double highThreshold);
	//滤波点云
	void filterPCD(const char* strInputPCDName, const char* strOutPutPCDName);

	//读入.las，写点云
	void writePointCloudFromLas(const char* strInputLasName, const char* strOutPutPointCloudName);
	//读入点云，写.las
	void writeLasFromPointCloud(const char* strInputPointCloudName, const char* strOutLasName);

	//根据点云名称，求点云的中点及最大x,y,z距离
	void getPCLMidPointAndDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xResolution, double yResolution, Pt3& midPoint, double& maxDistance, double &minZ, double& maxZ, int &xSize, int &ySize);

	//根据.tif文件名和区域范围，得到新的vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromTif(std::string strTifFileName, int xRoil, int yRoil, int width, int height);
	//根据.tif的所有点的vector和区域范围，得到新的vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromVecVec2(std::vector<std::vector<Pt3>> inputVecVec, int xRoil, int yRoil, int width, int height);

	//从第一幅图截取范围的左上角(xRoi,yRoi)计算出另一幅图像的截取范围的左上角(xRoi2,yRoi2)
	void getRoil2FromRoi1AndTif(int widthRoil, int heightRoil, double inputTopLeftX, double inputTopLeftY,
		 double xResolution2, double yResolution2, double topLeftX2,double topLeftY2,int xSize2,int ySize2,
		 int& xRoi2, int& yRoi2);

	//从第一幅图截取范围的参数计算出另一幅图像的截取范围的参数
	void getZone2FromZone1(zone zone1, tifParameter tif1, tifParameter tif2, zone& zone2);
	//得到像素点序列
	std::vector<float> getPixel32bitFromTifVecVec(std::vector<std::vector<Pt3>> vecvec);

	//根据.tif名称得到该.tif的各要素
	void getDetailFromTifName(std::string strTifName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);	
	void getTifParameterFromTifName(std::string strTifName,
		tifParameter & theTifParameter);

	//通过读取一部分.tif，创建栅格集合（32位）
	std::vector <float> getSegRasterVecVecFromTif_32bit(const char* strImageName, int xID, int yID, int width, int height);
	std::vector <float> getSegRasterVecVecFromTif_32bit(const char* strImageName, zone theZone);

	//按x从大到小排列
	bool greaterSortX(diffVec a, diffVec b);
	//按y从大到小排列
	bool greaterSortY(diffVec a, diffVec b);
	//按z从大到小排列
	bool greaterSortZ(diffVec a, diffVec b);
}