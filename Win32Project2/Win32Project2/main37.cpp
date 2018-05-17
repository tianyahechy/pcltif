

#pragma once
#include "quadNetWork.h"

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
	reader.read("d:\\1_xyz.pcd", *cloud);
	time(&readPointCloudTime);
	pt3Set vector3Set;
	vector3Set.clear();
	std::cout << "����:" << cloud->points.size() << std::endl;
	int sizeOfTotal = 20000;
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
	quadNetWork * theNetWork = new quadNetWork(sizeOfQuadsColumn, sizeOfQuadsRow, minX, minY, distanceBetweenQuadX, distanceBetweenQuadY);

	//��������㼯�����ȷ�XYZ��࣬�õ�ͼ�����꼯��vector,����ʼ���Ҷ�ֵΪ0
	theNetWork->getEqualXYZVectorFromDataSet(vector3Set, minX, minY, xSize, ySize, xResolution, yResolution);
	//���ǻ����ݼ����е�X,Y����,
	std::cout << "���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���" << std::endl;
	triangleSet myTriangleSet = theNetWork->getTriangleSetFromDataSet(vector3Set);
	time(&triangulationTime);
	//���������μ��ϼ����դ�񼯺ϣ���ֵ
	theNetWork->getRasterVec3SetFromTriangleSet(myTriangleSet, xResolution, yResolution, xSize, ySize );
	time(&chazhiTime);

	rasterVec3Set imageSet = theNetWork->getRasterVec3Set();

	//���������ļ�
	const char * pszRasterFile = "E:\\PCLOutPut_2w.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//�趨ͼ�����Ͻ�
	float topLeftX = minX;
	float topLeftY = maxY;
	theNetWork->createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	theNetWork->UpdateRasterFile(pszRasterFile, imageSet);
	
	//��¼����ʱ��
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "�ܹ�����ʱ��" << diff << "��" << std::endl;
	double diff_readPointCloudTime = difftime(readPointCloudTime, startTime);
	std::cout << "��ȡ���ƻ���ʱ��" << diff << "��" << std::endl;
	double diff_triangulationTime = difftime(triangulationTime, readPointCloudTime);
	std::cout << "���ǻ�����ʱ��" << diff << "��" << std::endl;
	double diff_chazhiTime = difftime(chazhiTime, triangulationTime);
	std::cout << "��ֵ����ʱ��" << diff << "��" << std::endl;
	double diff_writeTiff = difftime(endTime, chazhiTime);
	std::cout << "д��tif�ļ�����ʱ��" << diff << "��" << std::endl;
	
	/*
	double testZ = -1;
	//�õ������漯�ϡ���ȡ������
	//���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���
	std::cout << "���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);

	//�õ�ͼ��������XYZ
	std::cout << "�õ�ͼ��������XYZ" << std::endl;

	//������ļ�
	FILE * file = fopen("e:\\outputTriangleAndPoint", "w");

	for (int x = 118; x < 119; x++)
	{
		for (int y = 70; y < 71; y++ )
		{
			double theX = minX + xResolution * x;
			double theY = minY + yResolution * y;
			Pt2 thisPoint = Pt2(theX, theY);
			
			Pt3 thept3(0, 0, 0);
			std::vector<Pt3> pt3Vec;
			pt3Vec.clear();
			getPt3InsertAndPt3VecInOneTriangleFromDataSet(vector3Set, thisPoint, thept3, myTriangleSet, pt3Vec);
			std::cout << "��СΪ��" << pt3Vec.size() << std::endl;
	
			fprintf( file, "(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f)\n",
				pt3Vec[0].x(), pt3Vec[0].y(), pt3Vec[0].z(),
				pt3Vec[1].x(), pt3Vec[1].y(), pt3Vec[1].z(),
				pt3Vec[2].x(), pt3Vec[2].y(), pt3Vec[2].z(),
				thept3.x(), thept3.y(), thept3.z(),thisPoint.x(), thisPoint.y());
		}
	}

	fclose(file);
	*/
	return 0;
}