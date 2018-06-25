#pragma once
#include "common.h"

class soe_envelope
{
public:
	soe_envelope();
	~soe_envelope();

	//初始化该块点云的坐标信息
	void init(std::vector<Pt3> pointCloudIntheSegment);
	//根据最大最小值初始化坐标初始化信息
	void init(float minX, float maxX, float minY, float maxY );
	//得到点云的最大最小xy坐标
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

