#pragma once
#include "common.h"

class soe_envelope
{
public:
	soe_envelope();
	~soe_envelope();

	//��ʼ���ÿ���Ƶ�������Ϣ
	void init(std::vector<Pt3> pointCloudIntheSegment);
	//���������Сֵ��ʼ�������ʼ����Ϣ
	void init(float minX, float maxX, float minY, float maxY );
	//�õ����Ƶ������Сxy����
	float getMaxX();
	float getMaxY();
	float getMinX();
	float getMinY();

private:
	float _minX;
	float _maxX;
	float _minY;
	float _maxY;

};

