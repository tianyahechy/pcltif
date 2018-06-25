#pragma once
#include "common.h"
#include "soe_envelope.h"
#include "ImgCore.h"

#define  BT_32F GDT_Float32
#define find_win_size 30

//点云分块
struct epi_block        
{
	int block_index;			//点云分块的序号
	std::string block_dem_path; //路径名称
};

class recimage_pairs
{
public:
	recimage_pairs();
	~recimage_pairs();
public:
	//得到点集
	pt3Set getPointSet();
	//设置点集,起始点和终点,以及宽高
	void setPointSet(pt3Set dataSet,int startXID, int startYID, int widthRoi, int heightRoi );
	//分块处理
	void processBySegment(int sizeofSegment);
	//将点云集合分割成相应的数组
	void splitVectorFromPointCloud(pt3Set pointCloudDataSet,
		int segmentSize,
		std::vector<epi_block>& epi_blockVector,
		std::vector<std::vector<Pt3>>& cloudVector);
	//设置分辨率
	void setResolution(double xResolution, double yResolution );
	//输入点云的相邻两点之间的距离 ，输出该块的坐标
	bool dtm_resample(SOE_64F dtm_cel_size, epi_block & block);
	//设置总的栅格数组
	void setTotalRasterVec(std::vector<std::vector<Pt3>> rasterVector);
	//返回总的栅格数组
	std::vector<std::vector<Pt3>> getRasterVecVec3();
	//设置整个栅格的左上角坐标
	void setTopLeft(double topLeftX, double topLeftY);
private:
	pt3Set _pointSet;							//总的点云数据集合
	std::vector<std::vector<Pt3>> _totalRasterVec;//总的栅格数组
	std::vector<std::vector<Pt3>> _point_clound; //每块点云的集合
	double _xResolution;						//像素分辨率大小		
	double _yResolution;
	double _topLeftX;							//左上角坐标
	double _topLeftY;
	int _null_value;
	std::string _wkt;
};

