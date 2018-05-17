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
	//������̣����ϲ��⼸��
	void process();
	//�������뼯����ȷֲ�ֵXYZ����
	void getEqualXYZVectorFromDataSet();
	//�������ݼ��ϣ����ǻ�����������μ���
	void getTriangleSetFromDataSet();
	//���������μ��ϼ����դ�񼯺ϣ���ֵ
	void getRasterVec3SetFromTriangleSet();	
	//���������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
	void getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3);
	//������άդ���ά����
	std::vector<std::vector<Pt3>> getRasterVecVec3();
	//���ݵ��Ƽ�������Ƶ�ϸ������
	PCLDetail getPCLDetailFromDataSet();
	//���õ���ϸ��
	void setPointCloudDetail(PCLDetail theDetail);
	//�õ�����ϸ��
	PCLDetail getDetailOfthePointCloud();
	//�õ����Ƶ�X����������
	int getXSize();
	//�õ����Ƶ�Y����������
	int getYSize();

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
	int								_xSize;			//դ���x�������ظ���
	int								_ySize;			//դ���y�������ظ���
	std::vector<std::vector<Pt3>>	_rasterVecVec;	//դ��Ķ�ά����
	PCLDetail						_theDetail;		//����ϸ��
	double							_xResolution;	//���طֱ���(X)
	double							_yResolution;	//���طֱ���(Y)
	triangleSet						_myTriangleSet;	//���ǻ���������μ���
};

