

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"

int main(int argc, char **argv)
{
	//��¼��ʼʱ�ĵ�ǰʱ��ͽ���ʱ��
	time_t startTime, endTime;
	double diff;
	time(&startTime);

	//����X��Y��������طֱ���
	double xResolution = 1.0;
	double yResolution = -1.0;

	//const char * inputName = "e:\\CSite1origC.pcd";
	const char * inputName = argv[1];
	PCLTif * thePCLTif1 = new PCLTif(inputName, xResolution, yResolution);
	thePCLTif1->process();
	std::vector<std::vector<Pt3>> rastervec = thePCLTif1->getRasterVecVec3();
	
	//���������ļ�
	//const char * pszRasterFile = "E:\\CSite1origC.tif";
	const char * pszRasterFile = argv[2];
	int bandSize = 1;
	int xSize = thePCLTif1->getXSize();
	int ySize = thePCLTif1->getYSize();
	//�������Ͻ�Ϊminx,maxY
	PCLDetail theDetail = thePCLTif1->getDetailOfthePointCloud();
	double topLeftX = theDetail.leftTopX;
	double topLeftY = theDetail.leftTopY;
	util::createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	util::UpdateRasterFile(pszRasterFile, rastervec);

	//��¼����ʱ��
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "�ܹ�����ʱ��" << diff << "��" << std::endl;

	return 0;
	return 0;
}