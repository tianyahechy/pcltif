#pragma once
#include "common.h"

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

class PCLTif
{
public:
	PCLTif( std::string strFileName, double xResolution, double yResolution, PCLType theType );
	~PCLTif();
public:
	//处理过程，将合并这几步
	void process();
	//根据输入集合求等分插值XYZ坐标
	void getEqualXYZVectorFromDataSet();
	//输入数据集合，三角化后输出三角形集合
	void getTriangleSetFromDataSet();
	//根据三角形集合计算出栅格集合，插值
	void getRasterVec3SetFromTriangleSet();	
	//根据三角形计算出在该三角形外接矩形的栅格集合，插值
	void getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3);
	//返回三维栅格二维数组
	std::vector<std::vector<Pt3>> getRasterVecVec3();
	//根据点云集，求点云的细节内容
	PCLDetail getPCLDetailFromDataSet();
	//设置点云细节
	void setPointCloudDetail(PCLDetail theDetail);
	//得到点云细节
	PCLDetail getDetailOfthePointCloud();
	//得到点云的X方向像素数
	int getXSize();
	//得到点云的Y方向像素数
	int getYSize();

private:
	//根据三角形坐标求外接矩形的坐标极值，4个点的坐标
	std::vector<Pt2> _getRectanglePt2VectorFromTriangle(std::vector<Pt3> theTrianglePoint);
	// Same side method
	// Determine whether point P in triangle ABC
	bool testPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	//从三角形面的VECTOR和XY，得到指定点的Z值
	bool getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ);
	//判断该点在三角形中是否有重合点
	bool bInTheTriangle(Pt2 thePt2, std::vector<Pt3> pt3Vector, double& pointZ);
	//判断两点是否重合
	bool bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3);	
	//测试射线与线段是否相交
	bool computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint);
	//从线段两端的坐标得到插入点的高度
	double getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint);

private:
	pt3Set							_pointCloudDataSet;//点云集合	
	int								_xSize;			//栅格的x方向像素个数
	int								_ySize;			//栅格的y方向像素个数
	std::vector<std::vector<Pt3>>	_rasterVecVec;	//栅格的二维数组
	PCLDetail						_theDetail;		//点云细节
	double							_xResolution;	//像素分辨率(X)
	double							_yResolution;	//像素分辨率(Y)
	triangleSet						_myTriangleSet;	//三角化后的三角形集合
};

