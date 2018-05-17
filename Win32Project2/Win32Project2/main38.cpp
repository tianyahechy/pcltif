

#pragma once
#include "PCLTif.h"

int main()
{
	//��¼��ʼʱ�ĵ�ǰʱ��ͽ���ʱ��
	time_t startTime, endTime;
	time_t readPointCloudTime;  //��ȡ����ʱ��
	time_t triangulationTime;	//���ǻ�ʱ��
	time_t chazhiTime;			//��ֵդ��ʱ��

	double diff;
	time(&startTime);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	//��ȡ��������
	std::cout << "��ȡ��������:" << std::endl;
	reader.read("e:\\pt666000001_del.pcd", *cloud);
	time(&readPointCloudTime);
	pt3Set vector3Set;
	vector3Set.clear();
	std::cout << "����:" << cloud->points.size() << std::endl;
	int sizeOfTotal = cloud->points.size();
	//for (size_t i = 0; i < cloud->points.size(); i++)
	for (size_t i = 0; i < sizeOfTotal; i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		Pt3 theVector(Pt3(x, y, z));
		vector3Set.insert(pt3Pair(i, theVector));
		//std::cout << "��" << i << "�����ݣ�(" <<theVector.x() << "," << theVector.y() << "," << theVector.z() << ")" << std::endl;
	}

	//���ݵ��Ƽ�����Ƶ�ϸ��
	PCLDetail theDetail = getPCLDetail(vector3Set);
	
	//����X��Y��������طֱ���
	double xResolution = 1.0;
	double yResolution = 1.0;

	//��X,Y��������ظ���,
	double distanceX = theDetail.xDistance;
	double distanceY = theDetail.yDistance;
	double xSizef = distanceX / xResolution;
	double ySizef = distanceY / yResolution;
	//ȡ��
	int xSize = (int)xSizef;
	int ySize = (int)ySizef;

	//�������Ͻ�Ϊminx,maxY
	double minX = theDetail.minX;
	double minY = theDetail.minY;

	//�ֿ�����
	int triangleNumberOfEachQuad = 100;
	int sizeOfQuadsColumn = sqrt(sizeOfTotal * 1.0 / triangleNumberOfEachQuad);
	int sizeOfQuadsRow = sqrt(sizeOfTotal * 1.0 / triangleNumberOfEachQuad);
	double distanceBetweenQuadX = distanceX / sizeOfQuadsColumn;
	double distanceBetweenQuadY = distanceY / sizeOfQuadsRow;
	//quadNetWork * theNetWork = new quadNetWork(sizeOfQuadsColumn, sizeOfQuadsRow, minX, minY, distanceBetweenQuadX, distanceBetweenQuadY);
	PCLTif * thePCLTif = new PCLTif(theDetail);

	//��������㼯�����ȷ�XYZ��࣬�õ�ͼ�����꼯��vector,����ʼ���Ҷ�ֵΪ0
	thePCLTif->getEqualXYZVectorFromDataSet(vector3Set, minX, minY, xSize, ySize, xResolution, yResolution);
	//���ǻ����ݼ����е�X,Y����,
	std::cout << "���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���" << std::endl;
	triangleSet myTriangleSet = thePCLTif->getTriangleSetFromDataSet(vector3Set);
	time(&triangulationTime);
	//���������μ��ϼ����դ�񼯺ϣ���ֵ
	thePCLTif->getRasterVec3SetFromTriangleSet(myTriangleSet, xResolution, yResolution, xSize, ySize);
	time(&chazhiTime);

	//rasterVec3Set imageSet = theNetWork->getRasterVec3Set();
	rasterVec3Set imageSet = thePCLTif->getRasterVec3Set();
	//���������ļ�
	const char * pszRasterFile = "E:\\pt666000001_del2.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//�趨ͼ�����Ͻ�
	float topLeftX = minX;
	float topLeftY = maxY;
	thePCLTif->createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	thePCLTif->UpdateRasterFile(pszRasterFile, imageSet);
	
	//��¼����ʱ��
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "�ܹ�����ʱ��" << diff << "��" << std::endl;

	return 0;
}