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
	std::vector<uchar> getSegRasterVecVecFromTif_8bit(const char* strImageName, int xID, int yID, int width, int height, double &outputleftTopX, double &outputleftTopY);

	//ͨ����ȡһ����.tif������դ�񼯺ϣ�8λ��
	std::vector<uchar> getSegRasterVecVecFromTifAndLeftTop_8bit(const char* strImageName, int width, int height, double inputleftTopX, double inputleftTopY);

	//ͨ������tif,д����
	//bOrganized���Ƿ���������ƣ�trueΪ����falseΪ�������
	void writePointCloudFromTif(const char* strInputTifName, const char* strOutPutPointCloudName, bool bOrganized = false);
	//ͨ���������дtif
	void writeTifFromPointCloud(const char* strInputPointCloudName, const char* strOutPutTifName, double xResolution, double yResolution, int bandSize = 1 );
	//����
	void getDifTifBetweenTwoTifs(const char* strInputTifName1, const char* strInputTifName2, const char* strOutPutTifName);
	//�˲�.TIF
	void filterTif(const char* strInputTifName, const char* strOutPutTifName);
	//�˲�����
	void filterPCD(const char* strInputPCDName, const char* strOutPutPCDName);

	//����.las��д����
	void writePointCloudFromLas(const char* strInputLasName, const char* strOutPutPointCloudName);
	//������ƣ�д.las
	void writeLasFromPointCloud(const char* strInputPointCloudName, const char* strOutLasName);

	//���ݵ������ƣ�����Ƶ��е㼰���x,y,z����
	void getPCLMidPointAndDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xResolution, double yResolution, Pt3& midPoint, double& maxDistance, double &minZ, double& maxZ, int &xSize, int &ySize);

}