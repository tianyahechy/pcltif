#pragma once

#include "common.h"

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <thread>

class quadNetWork
{
public:
	quadNetWork(int sizeOfColumn, int sizeOfRow, double minX, double minY, double distanceBetweenQuadX, double distanceBetweenQuadY);
	~quadNetWork();

public:
	//输入数据集合，三角化后输出各分块的三角形集合
	void getTriangleSetOfEachQuadFromDataSet(pt3Set dataSet);
	//通过二维坐标，获得相应分块里的三角形集合
	mapTrianglesOfEachQuad getTriangleSetByCoordinateXY(Pt2 inputCoordinate);
	//将指定三角形加入相应分块的三角形集合中
	bool addTriangleToTrianglesSetOfAllQuads(myTriangle theTriangle);
	//得到所有三角形集合
	mapTrianglesSetOfAllQuads getTriangleSetOfAllQuads();

public:
	/*
	插值栅格灰度值，
	*/

private:	
	//通过分块序号获得该分块里的三角形集合
	mapTrianglesOfEachQuad _getTriangleSetByQuadID(int quadId);
	//根据二维坐标来计算所在块的序号
	void _getQuadIDFromD2(Pt2 inputCoordinate, int &quadID);
	//根据一位数字计算所在块的行列和ID
	void _getQuadIDFromD1(float inputNumber, int &outputRow, int& outputColumn, int &quadID);
	//指定一维坐标，计算该坐标的ID
	void _getdistanceFromD1(double inputCoordinate, double minNum, double distanceofEachQuad, int& outputID);
	//给定三角形和分块三角形集合，以及指定分块ID，在该块加上此三角形
	bool _addTriangleToQuadByQuadID(myTriangle theTriangle, int quadID);
	//根据三角形坐标求外接矩形的坐标极值，4个点的坐标
	std::vector<Pt2> _getRectanglePt2VectorFromTriangle(myTriangle theTriangle);	
	//根据三角形坐标求外接矩形的坐标极值，4个点的坐标
	std::vector<Pt2> _getRectanglePt2VectorFromTriangle(std::vector<Pt3> theTrianglePoint);
	//根据三角形外接矩形顶点坐标集合,计算出三角形所在的块ID(可能会多个块）
	std::vector<int> _getQuadIDSFromRectangle(std::vector<Pt2> rectVec);
	//给定三角形和分块三角形集合，以及指定分块ID集合，在该块加上此三角形
	bool _addTriangleToQuadsByQuadIDs(myTriangle theTriangle, std::vector<int> quadIDVector);
private:

	double _minX;					//最小x坐标
	double _minY;					//最小Y坐标
	double _distanceBetweenQuadX;	//各分块之间的X方向距离
	double _distanceBetweenQuadY;	//各分块之间的Y方向距离
	int _sizeOfColumn;				//列数总数
	int _sizeOfRow;					//行数总数

	mapTrianglesSetOfAllQuads	_triangleSetOfAllQuads;	//所有块中的三角形集合
	rasterVec3Set				_rasterVec3Set; //栅格的三维集合
	std::vector<std::vector<Pt3>> _rasterVecVec; //栅格的二维数组

public:	
	//暂时存放的函数
	//指定点XY集合得出
	//测试线段和指定点的朝向，(左旋，右旋还是共面)
	int testOrientationBetweenPointInSegment(Pt2 thePoint, Pt2 startPoint, Pt2 endPoint);
	// 测试坐标是否在三角形内
	bool testRange(Pt2 thePoint, std::vector<Pt2> pt);
	// Determine whether two vectors v1 and v2 point to the same direction
	// v1 = Cross(AB, AC)
	// v2 = Cross(AB, AP)
	bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	int getSignal(Vector3 A, Vector3 B, Vector3 P);
	bool SameSide2(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	// Same side method
	// Determine whether point P in triangle ABC
	bool testPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	//测试射线与线段是否相交
	bool computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint);
	//从线段两端的坐标得到插入点的高度
	double getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint);
	//判断两点是否重合
	bool bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3);
	//判断点是否在矩形内
	bool testPtInRect(Pt2 thePt, myRect theRect);
	//判断该点在三角形中是否有重合点
	bool bInTheTriangle(Pt2 thePt2, std::vector<Pt3> pt3Vector, double& pointZ);
	//从三角形面的VECTOR和XY，得到指定点的Z值
	bool getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ);
	// 从三角形面的VECTOR和XY，得到指定点的Z值
	bool getPointZAndTraingleVectorfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ);
	//根据指定点的X,Y和该分块的三角形集合，计算出该点所在的三角形顶点序列
	std::vector<Pt3> getPt3VectorOftheTrianglefromthePointAndTriangleSetOfQuad(Pt2 thePt, mapTrianglesOfEachQuad theTriangleSetOfThisQuad);
	//输入数据集合，三角化后输出三角形集合
	triangleSet getTriangleSetFromDataSet(pt3Set dataSet);
	//从数据集合和三角面集合中得出插入点的高度和插入的三角形点
	bool getPt3InsertAndPt3VecInOneTriangleFromDataSet(pt3Set dataSet, Pt2 thePoint, Pt3& outPutPoint3d, std::vector<Pt3>& trianglePointVector);
	//从点云数据集合和坐标XYvector和分块三角形集合中,
	rasterVec3Set getPt3SetFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set);

	//根据输入集合求等分插值XY坐标
	rasterVec2Set getEqualXYVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution);
	//根据输入集合求等分插值XYZ坐标
	void getEqualXYZVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution);
	//通过栅格创建TIF文件
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, double xResolution, double yResolution, double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);
	//通过更改栅格数据更新TIFF文件
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet);

	//返回三维栅格集合
	rasterVec3Set getRasterVec3Set();

	//根据三角形集合计算出栅格集合，插值
	void getRasterVec3SetFromTriangleSet(triangleSet theTriangleSet, double xResolution, double	 yResolution, int xSize, int ySize );
	//根据三角形计算出在该三角形外接矩形的栅格集合，插值
	void getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3, double xResolution, double yResolution,int xSize, int ySize);

public:
	//并行计算（暂时不用）

	//二维坐标将集合分为
	std::vector<rasterLine2> getVectorFromSet(rasterVec2Set theSet);
	//将原来的查找点的二维坐标数组分成几个数组
	std::vector<std::vector<Pt2>> getVectorOfEachPart(int partNumber, std::vector<Pt2> originalVector);
	//对行号为id的行的其中一部分进行输出处理
	void processOnePart(pt3Set dataSet, std::vector<Pt2> theVector, int id);
	//将行号为id的行的二维坐标数组并行计算
	void parrelProcessVector(pt3Set dataSet, std::vector<Pt2> originalVector, int id);
	//将原来的查找点的二维坐标集合并行计算
	void parrelProcessSet(pt3Set dataSet, rasterVec2Set originalSet);
	//从点云数据集合和坐标XYvector和三角形集合中,并行处理得到栅格的三维集合
	void getPt3SetParrelFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set);

public:

	//得到被三角形的外接矩形截取的map部分(minx,miny,max,maxy)
	std::map<int, std::vector<Pt3>> getSelectedRegion3(double minY, double maxY, double yResolution, std::map<int, std::vector<Pt3>> inputMap);
	//得到截取的带序号的vector部分
	std::vector<Pt3WithColumnID> getSelectedVector(double minX, double maxX, double xResolution, std::vector<Pt3> inputVector);
	//根据xy的范围，以及位置集合，来得到带序号的范围集合
	std::map<int, std::vector<Pt3WithColumnID>> getPt3WithColumnIDSetFromPt3SetAndXYRange(double minX, double minY,
		double maxX, double maxY, double xResolution, double yResolution, std::map<int, std::vector<Pt3>> inputMap );
	//处理带序号的点
	Pt3WithColumnID processPt3WithColumnID(Pt3WithColumnID inputPt, Vector3 vecA, Vector3 vecB, Vector3 vecC, std::vector<Pt3> theTrianglePt3);
	//处理带序号集合
	std::map<int, std::vector<Pt3WithColumnID>> processSetWithColumnID(std::map<int, std::vector<Pt3WithColumnID>> inputSet, Vector3 vecA, Vector3 vecB, Vector3 vecC, std::vector<Pt3> theTrianglePt3);
	//返回原来的集合
	void convertToOriginSet(std::map<int, std::vector<Pt3WithColumnID>> selectedSet);

};

