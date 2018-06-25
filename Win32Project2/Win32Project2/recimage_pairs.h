#pragma once
#include "common.h"
#include "soe_envelope.h"
#include "ImgCore.h"

#define  BT_32F GDT_Float32
#define find_win_size 30

//���Ʒֿ�
struct epi_block        
{
	int block_index;			//���Ʒֿ�����
	std::string block_dem_path; //·������
};

class recimage_pairs
{
public:
	recimage_pairs();
	~recimage_pairs();
public:
	//�õ��㼯
	pt3Set getPointSet();
	//���õ㼯,��ʼ����յ�,�Լ����
	void setPointSet(pt3Set dataSet,int startXID, int startYID, int widthRoi, int heightRoi );
	//�ֿ鴦��
	void processBySegment(int sizeofSegment);
	//�����Ƽ��Ϸָ����Ӧ������
	void splitVectorFromPointCloud(pt3Set pointCloudDataSet,
		int segmentSize,
		std::vector<epi_block>& epi_blockVector,
		std::vector<std::vector<Pt3>>& cloudVector);
	//���÷ֱ���
	void setResolution(double xResolution, double yResolution );
	//������Ƶ���������֮��ľ��� ������ÿ������
	bool dtm_resample(SOE_64F dtm_cel_size, epi_block & block);
	//�����ܵ�դ������
	void setTotalRasterVec(std::vector<std::vector<Pt3>> rasterVector);
	//�����ܵ�դ������
	std::vector<std::vector<Pt3>> getRasterVecVec3();
	//��������դ������Ͻ�����
	void setTopLeft(double topLeftX, double topLeftY);
private:
	pt3Set _pointSet;							//�ܵĵ������ݼ���
	std::vector<std::vector<Pt3>> _totalRasterVec;//�ܵ�դ������
	std::vector<std::vector<Pt3>> _point_clound; //ÿ����Ƶļ���
	double _xResolution;						//���طֱ��ʴ�С		
	double _yResolution;
	double _topLeftX;							//���Ͻ�����
	double _topLeftY;
	int _null_value;
	std::string _wkt;
};

