#pragma once
#include <string>
#include "soeImgInfo.h"
#include "soe_envelope.h"

typedef __int64 SOE_64F;
typedef __int32 SOE_32S;
typedef float	SOE_32F;
typedef unsigned int SOE_16U;

class ImgCore
{
public:
	ImgCore();
	~ImgCore();

public:
	bool Create( std::string pathStr, SOE_IMG_INFO * theInfo );
	void SetExtent(soe_envelope * theEnvelop);
	void SetProjection(const char * charWKT);
	//Ð´ÎÄ¼þ
	void DataWriteBand(int, int, int, int width, int height, SOE_32F *result_data);
	void Close();
private:
	std::string _wkt;
};

