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
	//�������ݼ��ϣ����ǻ���������ֿ�������μ���
	void getTriangleSetOfEachQuadFromDataSet(pt3Set dataSet);
	//ͨ����ά���꣬�����Ӧ�ֿ���������μ���
	mapTrianglesOfEachQuad getTriangleSetByCoordinateXY(Pt2 inputCoordinate);
	//��ָ�������μ�����Ӧ�ֿ�������μ�����
	bool addTriangleToTrianglesSetOfAllQuads(myTriangle theTriangle);
	//�õ����������μ���
	mapTrianglesSetOfAllQuads getTriangleSetOfAllQuads();

public:
	/*
	��ֵդ��Ҷ�ֵ��
	*/

private:	
	//ͨ���ֿ���Ż�ø÷ֿ���������μ���
	mapTrianglesOfEachQuad _getTriangleSetByQuadID(int quadId);
	//���ݶ�ά�������������ڿ�����
	void _getQuadIDFromD2(Pt2 inputCoordinate, int &quadID);
	//����һλ���ּ������ڿ�����к�ID
	void _getQuadIDFromD1(float inputNumber, int &outputRow, int& outputColumn, int &quadID);
	//ָ��һά���꣬����������ID
	void _getdistanceFromD1(double inputCoordinate, double minNum, double distanceofEachQuad, int& outputID);
	//���������κͷֿ������μ��ϣ��Լ�ָ���ֿ�ID���ڸÿ���ϴ�������
	bool _addTriangleToQuadByQuadID(myTriangle theTriangle, int quadID);
	//������������������Ӿ��ε����꼫ֵ��4���������
	std::vector<Pt2> _getRectanglePt2VectorFromTriangle(myTriangle theTriangle);	
	//������������������Ӿ��ε����꼫ֵ��4���������
	std::vector<Pt2> _getRectanglePt2VectorFromTriangle(std::vector<Pt3> theTrianglePoint);
	//������������Ӿ��ζ������꼯��,��������������ڵĿ�ID(���ܻ����飩
	std::vector<int> _getQuadIDSFromRectangle(std::vector<Pt2> rectVec);
	//���������κͷֿ������μ��ϣ��Լ�ָ���ֿ�ID���ϣ��ڸÿ���ϴ�������
	bool _addTriangleToQuadsByQuadIDs(myTriangle theTriangle, std::vector<int> quadIDVector);
private:

	double _minX;					//��Сx����
	double _minY;					//��СY����
	double _distanceBetweenQuadX;	//���ֿ�֮���X�������
	double _distanceBetweenQuadY;	//���ֿ�֮���Y�������
	int _sizeOfColumn;				//��������
	int _sizeOfRow;					//��������

	mapTrianglesSetOfAllQuads	_triangleSetOfAllQuads;	//���п��е������μ���
	rasterVec3Set				_rasterVec3Set; //դ�����ά����
	std::vector<std::vector<Pt3>> _rasterVecVec; //դ��Ķ�ά����

public:	
	//��ʱ��ŵĺ���
	//ָ����XY���ϵó�
	//�����߶κ�ָ����ĳ���(�������������ǹ���)
	int testOrientationBetweenPointInSegment(Pt2 thePoint, Pt2 startPoint, Pt2 endPoint);
	// ���������Ƿ�����������
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
	//�����������߶��Ƿ��ཻ
	bool computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint);
	//���߶����˵�����õ������ĸ߶�
	double getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint);
	//�ж������Ƿ��غ�
	bool bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3);
	//�жϵ��Ƿ��ھ�����
	bool testPtInRect(Pt2 thePt, myRect theRect);
	//�жϸõ������������Ƿ����غϵ�
	bool bInTheTriangle(Pt2 thePt2, std::vector<Pt3> pt3Vector, double& pointZ);
	//�����������VECTOR��XY���õ�ָ�����Zֵ
	bool getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ);
	// �����������VECTOR��XY���õ�ָ�����Zֵ
	bool getPointZAndTraingleVectorfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ);
	//����ָ�����X,Y�͸÷ֿ�������μ��ϣ�������õ����ڵ������ζ�������
	std::vector<Pt3> getPt3VectorOftheTrianglefromthePointAndTriangleSetOfQuad(Pt2 thePt, mapTrianglesOfEachQuad theTriangleSetOfThisQuad);
	//�������ݼ��ϣ����ǻ�����������μ���
	triangleSet getTriangleSetFromDataSet(pt3Set dataSet);
	//�����ݼ��Ϻ������漯���еó������ĸ߶ȺͲ���������ε�
	bool getPt3InsertAndPt3VecInOneTriangleFromDataSet(pt3Set dataSet, Pt2 thePoint, Pt3& outPutPoint3d, std::vector<Pt3>& trianglePointVector);
	//�ӵ������ݼ��Ϻ�����XYvector�ͷֿ������μ�����,
	rasterVec3Set getPt3SetFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set);

	//�������뼯����ȷֲ�ֵXY����
	rasterVec2Set getEqualXYVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution);
	//�������뼯����ȷֲ�ֵXYZ����
	void getEqualXYZVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution);
	//ͨ��դ�񴴽�TIF�ļ�
	void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, double xResolution, double yResolution, double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0);
	//ͨ������դ�����ݸ���TIFF�ļ�
	void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet);

	//������άդ�񼯺�
	rasterVec3Set getRasterVec3Set();

	//���������μ��ϼ����դ�񼯺ϣ���ֵ
	void getRasterVec3SetFromTriangleSet(triangleSet theTriangleSet, double xResolution, double	 yResolution, int xSize, int ySize );
	//���������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
	void getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3, double xResolution, double yResolution,int xSize, int ySize);

public:
	//���м��㣨��ʱ���ã�

	//��ά���꽫���Ϸ�Ϊ
	std::vector<rasterLine2> getVectorFromSet(rasterVec2Set theSet);
	//��ԭ���Ĳ��ҵ�Ķ�ά��������ֳɼ�������
	std::vector<std::vector<Pt2>> getVectorOfEachPart(int partNumber, std::vector<Pt2> originalVector);
	//���к�Ϊid���е�����һ���ֽ����������
	void processOnePart(pt3Set dataSet, std::vector<Pt2> theVector, int id);
	//���к�Ϊid���еĶ�ά�������鲢�м���
	void parrelProcessVector(pt3Set dataSet, std::vector<Pt2> originalVector, int id);
	//��ԭ���Ĳ��ҵ�Ķ�ά���꼯�ϲ��м���
	void parrelProcessSet(pt3Set dataSet, rasterVec2Set originalSet);
	//�ӵ������ݼ��Ϻ�����XYvector�������μ�����,���д���õ�դ�����ά����
	void getPt3SetParrelFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set);

public:

	//�õ��������ε���Ӿ��ν�ȡ��map����(minx,miny,max,maxy)
	std::map<int, std::vector<Pt3>> getSelectedRegion3(double minY, double maxY, double yResolution, std::map<int, std::vector<Pt3>> inputMap);
	//�õ���ȡ�Ĵ���ŵ�vector����
	std::vector<Pt3WithColumnID> getSelectedVector(double minX, double maxX, double xResolution, std::vector<Pt3> inputVector);
	//����xy�ķ�Χ���Լ�λ�ü��ϣ����õ�����ŵķ�Χ����
	std::map<int, std::vector<Pt3WithColumnID>> getPt3WithColumnIDSetFromPt3SetAndXYRange(double minX, double minY,
		double maxX, double maxY, double xResolution, double yResolution, std::map<int, std::vector<Pt3>> inputMap );
	//�������ŵĵ�
	Pt3WithColumnID processPt3WithColumnID(Pt3WithColumnID inputPt, Vector3 vecA, Vector3 vecB, Vector3 vecC, std::vector<Pt3> theTrianglePt3);
	//�������ż���
	std::map<int, std::vector<Pt3WithColumnID>> processSetWithColumnID(std::map<int, std::vector<Pt3WithColumnID>> inputSet, Vector3 vecA, Vector3 vecB, Vector3 vecC, std::vector<Pt3> theTrianglePt3);
	//����ԭ���ļ���
	void convertToOriginSet(std::map<int, std::vector<Pt3WithColumnID>> selectedSet);

};

