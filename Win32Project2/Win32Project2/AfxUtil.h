#pragma once
#include "common.h"

namespace util
{

	//ͨ��դ�񴴽�TIF�ļ�
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);

	//ͨ��դ�񴴽�TIF�ļ�(8λ)
	void createRasterFile_8bit(const char* strImageName, int bandSize, int xSize, int ySize,
		double xResolution, double yResolution,
		double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);

	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet);

	//ͨ����ȡtif�ļ�����դ�񼯺�
	rasterVec3Set getRasterSetFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);

	//�����������
	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile(const char* strImageName, std::vector<std::vector<Pt3>> theRasterVecVec);
	//ͨ������դ�����ݸ���TIFF�ļ�(8bit)
	void UpdateRasterFile_8bit(const char* strImageName, std::vector<std::vector<Pt8Bit>> theRasterVecVec);
	//ͨ����ȡtif�ļ�����դ�񼯺�
	std::vector<std::vector<Pt3>> getRasterVecVecFromTif(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);
	//ͨ����ȡtif�ļ�����դ�񼯺�(8λ��
	std::vector<std::vector<Pt8Bit>> getRasterVecVecFromTif_8bit(const char* strImageName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�8λ��
	std::vector<uchar> getSegRasterVecVecFromTif_8bit(const char* strImageName, int xID, int yID, int width, int height,
		double &outputleftTopX, double &outputleftTopY, double &xResolu, double &yResolu);

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�8λ��
	std::vector<uchar> getSegRasterVecVecFromTifAndLeftTop_8bit(const char* strImageName, int width, int height, double inputleftTopX, double inputleftTopY,
		int& xRoi2, int& yRoi2, double &outputLeftTopX, double& outputLeftTopY, double& xResolu, double& yResolu);

	//ͨ������tif,д����
	//bOrganized���Ƿ���������ƣ�trueΪ����falseΪ�������
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName, float invalidValue, bool bOrganized = false);
	//ͨ���������дtif
	void writeTifFromPointCloud(const char* strInputPointCloudName, const char* strOutPutTifName, double xResolution, double yResolution, int bandSize = 1 );
	//ͨ��������Ʒֿ�дtif
	void writeTifFromPointCloudBySegment(const char* strInputPointCloudName, 
		const char* strOutPutTifName, 
		int sizeofSegment,
		double xResolution, 
		double yResolution, 
		int bandSize = 1);

	//����
	void getDifTifBetweenTwoTifs(const char* strInputTifName1, const char* strInputTifName2, const char* strOutPutTifName);
	//�˲�.TIF
	void filterTif(const char* strInputTifName, const char* strOutPutTifName);
	//������ֵ�˲�.TIF
	void filterTifByThreshold(const char* strInputTifName, const char* strOutPutTifName, double lowThreshold, double highThreshold);
	//�˲�����
	void filterPCD(const char* strInputPCDName, const char* strOutPutPCDName);

	//����.las��д����
	void writePointCloudFromLas(const char* strInputLasName, const char* strOutPutPointCloudName);
	//������ƣ�д.las
	void writeLasFromPointCloud(const char* strInputPointCloudName, const char* strOutLasName);

	//���ݵ������ƣ�����Ƶ��е㼰���x,y,z����
	void getPCLMidPointAndDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xResolution, double yResolution, Pt3& midPoint, double& maxDistance, double &minZ, double& maxZ, int &xSize, int &ySize);

	//����.tif�ļ���������Χ���õ��µ�vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromTif(std::string strTifFileName, int xRoil, int yRoil, int width, int height);
	//����.tif�����е��vector������Χ���õ��µ�vector
	std::vector<std::vector<Pt3>> getSegRasterVecVecFromVecVec2(std::vector<std::vector<Pt3>> inputVecVec, int xRoil, int yRoil, int width, int height);

	//�ӵ�һ��ͼ��ȡ��Χ�����Ͻ�(xRoi,yRoi)�������һ��ͼ��Ľ�ȡ��Χ�����Ͻ�(xRoi2,yRoi2)
	void getRoil2FromRoi1AndTif(int widthRoil, int heightRoil, double inputTopLeftX, double inputTopLeftY,
		 double xResolution2, double yResolution2, double topLeftX2,double topLeftY2,int xSize2,int ySize2,
		 int& xRoi2, int& yRoi2);

	//�ӵ�һ��ͼ��ȡ��Χ�Ĳ����������һ��ͼ��Ľ�ȡ��Χ�Ĳ���
	void getZone2FromZone1(zone zone1, tifParameter tif1, tifParameter tif2, zone& zone2);
	//�õ����ص�����
	std::vector<float> getPixel32bitFromTifVecVec(std::vector<std::vector<Pt3>> vecvec);

	//����.tif���Ƶõ���.tif�ĸ�Ҫ��
	void getDetailFromTifName(std::string strTifName,
		int& xSize, int& ySize,
		double& xResolution, double& yResolution,
		double& topLeftX, double& topLeftY);	
	void getTifParameterFromTifName(std::string strTifName,
		tifParameter & theTifParameter);

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�32λ��
	std::vector <float> getSegRasterVecVecFromTif_32bit(const char* strImageName, int xID, int yID, int width, int height);
	std::vector <float> getSegRasterVecVecFromTif_32bit(const char* strImageName, zone theZone);

	//��x�Ӵ�С����
	bool greaterSortX(diffVec a, diffVec b);
	//��y�Ӵ�С����
	bool greaterSortY(diffVec a, diffVec b);
	//��z�Ӵ�С����
	bool greaterSortZ(diffVec a, diffVec b);
}