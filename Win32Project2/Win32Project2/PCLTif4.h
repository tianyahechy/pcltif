#pragma once
#include "common.h"

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

class PCLTif
{
public:
	PCLTif( std::string strFileName, double xResolution, double yResolution );
	~PCLTif();
public:
	//�������뼯����ȷֲ�ֵXYZ����
	void getEqualXYZVectorFromDataSet( int xSize, int ySize );
	//�������ݼ��ϣ����ǻ�����������μ���
	triangleSet getTriangleSetFromDataSet();
	//���������μ��ϼ����դ�񼯺ϣ���ֵ
	void getRasterVec3SetFromTriangleSet(triangleSet theTriangleSet, int xSize, int ySize);	
	//���������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
	void getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3, int xSize, int ySize);
	//������άդ�񼯺�
	rasterVec3Set getRasterVec3Set();
	//ͨ��դ�񴴽�TIF�ļ�
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);
	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet);
	//���ݵ��Ƽ�������Ƶ�ϸ������
	PCLDetail getPCLDetailFromDataSet();
	//���õ���ϸ��
	void setPointCloudDetail(PCLDetail theDetail);
private:
	//������������������Ӿ��ε����꼫ֵ��4���������
	std::vector<Pt2> _getRectanglePt2VectorFromTriangle(std::vector<Pt3> theTrianglePoint);
	// Same side method
	// Determine whether point P in triangle ABC
	bool testPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P);
	//�����������VECTOR��XY���õ�ָ�����Zֵ
	bool getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ);
	//�жϸõ������������Ƿ����غϵ�
	bool bInTheTriangle(Pt2 thePt2, std::vector<Pt3> pt3Vector, double& pointZ);
	//�ж������Ƿ��غ�
	bool bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3);	
	//�����������߶��Ƿ��ཻ
	bool computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint);
	//���߶����˵�����õ������ĸ߶�
	double getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint);

private:
	pt3Set							_pointCloudDataSet;//���Ƽ���	
	std::vector<std::vector<Pt3>>	_rasterVecVec;	//դ��Ķ�ά����
	rasterVec3Set					_rasterVec3Set; //դ�����ά����
	PCLDetail						_theDetail;
	double							_xResolution;	//���طֱ���(X)
	double							_yResolution;	//���طֱ���(Y)
};

