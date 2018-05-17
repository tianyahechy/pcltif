

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

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
	thePCLTif->process();

	//���������ļ�
	const char * pszRasterFile = "E:\\pt666000001_del9.tif";
	int bandSize = 1;
	int xSize = thePCLTif->getXSize();
	int ySize = thePCLTif->getYSize();
	//�������Ͻ�Ϊminx,maxY
	PCLDetail theDetail = thePCLTif->getDetailOfthePointCloud();
	double topLeftX = theDetail.minX;
	double topLeftY = theDetail.maxY;
	util::createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	rasterVec3Set imageSet = thePCLTif->getRasterVec3Set();
	util::UpdateRasterFile(pszRasterFile, imageSet);

	//��¼����ʱ��
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "�ܹ�����ʱ��" << diff << "��" << std::endl;

	return 0;
}