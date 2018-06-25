#include "soe_envelope.h"

soe_envelope::soe_envelope()
{
}

soe_envelope::~soe_envelope()
{
}

//��ʼ���ÿ����
void soe_envelope::init(std::vector<Pt3> pointCloudIntheSegment)
{
	//1��������Ƶ��Ƿ�Ϊ��,���򷵻�
	int theSize = pointCloudIntheSegment.size();
	if (theSize == 0)
	{
		return;
	}
	//2,�����㼯������xy����������Сֵ
	_minX = pointCloudIntheSegment[0].x();
	_maxX = pointCloudIntheSegment[0].x();
	_minY = pointCloudIntheSegment[0].y();
	_maxY = pointCloudIntheSegment[0].y();
	for (size_t i = 0; i < theSize; i++)
	{
		float x = pointCloudIntheSegment[i].x();
		float y = pointCloudIntheSegment[i].y();
		if (_minX > x)
		{
			_minX = x;
		}
		if (_maxX < x)
		{
			_maxX = x;
		}
		if ( _minY > y)
		{
			_minY = y;
		}
		if (_maxY < y)
		{
			_maxY = y;
		}
	}
}

//���������Сֵ��ʼ�������ʼ����Ϣ
void soe_envelope::init(float minX, float maxX, float minY, float maxY)
{
	_minX = minX;
	_maxX = maxX;
	_minY = minY;
	_maxY = maxY;
}
float soe_envelope::getMaxX()
{
	return _maxX;
}
float soe_envelope::getMaxY()
{
	return _maxY;
}
float soe_envelope::getMinX()
{
	return _minX;
}
float soe_envelope::getMinY()
{
	return _minY;
}