

#pragma once
#include "PCLTif.h"

int main()
{
	//��¼��ʼʱ�ĵ�ǰʱ��ͽ���ʱ��
	time_t startTime, endTime;
	double diff;
	time(&startTime);

	//����X��Y��������طֱ���
	double xResolution = 1.0;
	double yResolution = 1.0;

	PCLTif * thePCLTif = new PCLTif("e:\\pt666000001_del.pcd", xResolution, yResolution);
	//���ݵ��Ƽ�����Ƶ�ϸ��
	PCLDetail theDetail = thePCLTif->getPCLDetailFromDataSet();
	thePCLTif->setPointCloudDetail(theDetail);
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

	//��������㼯�����ȷ�XYZ��࣬�õ�ͼ�����꼯��vector,����ʼ���Ҷ�ֵΪ0
	thePCLTif->getEqualXYZVectorFromDataSet( xSize, ySize);
	//���ǻ����ݼ����е�X,Y����,
	std::cout << "���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���" << std::endl;
	thePCLTif->getTriangleSetFromDataSet();
	//���������μ��ϼ����դ�񼯺ϣ���ֵ
	thePCLTif->getRasterVec3SetFromTriangleSet(xSize, ySize);

	rasterVec3Set imageSet = thePCLTif->getRasterVec3Set();
	//���������ļ�
	const char * pszRasterFile = "E:\\pt666000001_del6.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//�趨ͼ�����Ͻ�
	float topLeftX = minX;
	float topLeftY = maxY;
	thePCLTif->createRasterFile(pszRasterFile, bandSize, xSize, ySize, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	thePCLTif->UpdateRasterFile(pszRasterFile, imageSet);
	
	//��¼����ʱ��
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "�ܹ�����ʱ��" << diff << "��" << std::endl;

	return 0;
}