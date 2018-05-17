#include "quadNetWork.h"

quadNetWork::quadNetWork(int sizeOfColumn, int sizeOfRow, double minX, double minY, double distanceBetweenQuadX, double distanceBetweenQuadY)
{
	_sizeOfColumn = sizeOfColumn;
	_sizeOfRow = sizeOfRow;
	_minX = minX;
	_minY = minY;
	_distanceBetweenQuadX = distanceBetweenQuadX;
	_distanceBetweenQuadY = distanceBetweenQuadY;

	int totalQuadNumber = _sizeOfRow * _sizeOfColumn;

	//�ֿ������μ���
	_triangleSetOfAllQuads.clear();
	for (int i = 0; i < totalQuadNumber; i++)
	{
		mapTrianglesOfEachQuad triangleSet;
		triangleSet.clear();
		_triangleSetOfAllQuads.insert(pairTrianglesSetOfAllQuads(i, triangleSet));
	}

	_rasterVec3Set.clear();
}

quadNetWork::~quadNetWork()
{
}

//ͨ����ά���꣬�����Ӧ�ֿ���������μ���
mapTrianglesOfEachQuad quadNetWork::getTriangleSetByCoordinateXY(Pt2 inputCoordinate)
{	//���ݶ�ά�������������ڿ�����
	int quadID = -1;
	this->_getQuadIDFromD2(inputCoordinate, quadID);
	//ͨ���ֿ���Ż�ø÷ֿ���������μ���
	mapTrianglesOfEachQuad trianglesOfThisQuad = this->_getTriangleSetByQuadID(quadID);
	return trianglesOfThisQuad;

}

//��ָ�������μ�����Ӧ�ֿ�������μ�����
bool quadNetWork::addTriangleToTrianglesSetOfAllQuads(myTriangle theTriangle)
{
	//1����ø������ε���Ӿ��ζ������꼯��
	std::vector<Pt2> coordinateOfRectVector = this->_getRectanglePt2VectorFromTriangle(theTriangle);

	//2��������������Ӿ��ζ������꼯�ϼ�������������ڵĿ�ID(���ܻ����飩
	std::vector<int> quadIDVector = this->_getQuadIDSFromRectangle(coordinateOfRectVector);

	//3��������Ӿ���
	myTriangle tri;
	tri.pt3sInTriangle = theTriangle.pt3sInTriangle;
	tri.theRect.minX = coordinateOfRectVector[0].x();
	tri.theRect.minY = coordinateOfRectVector[0].y();
	tri.theRect.maxX = coordinateOfRectVector[3].x();
	tri.theRect.maxY = coordinateOfRectVector[3].y();

	//4,���ݷֿ�ID���ϣ��ڸ�����ϴ�������
	this->_addTriangleToQuadsByQuadIDs(tri, quadIDVector);

	return true;
}

//�õ����������μ���
mapTrianglesSetOfAllQuads quadNetWork::getTriangleSetOfAllQuads()
{
	return _triangleSetOfAllQuads;
}

//����һλ�����ж����ڿ�����к�ID
void quadNetWork::_getQuadIDFromD1(float inputNumber, int &outputRow, int& outputColumn, int &quadID)
{
	int numb = (int)inputNumber;
	outputColumn = numb % _sizeOfColumn;
	outputRow = numb / _sizeOfColumn;
	quadID = outputRow * _sizeOfColumn + outputColumn;
}

//�ж�һά�����ϵĿ���
void quadNetWork::_getdistanceFromD1(double inputCoordinate, double minNum, double distanceofEachQuad, int& outputID)
{
	int numb = (int)(inputCoordinate - minNum);
	outputID = numb / distanceofEachQuad;
}

//���ݶ�ά����������ڷֿ�����
void quadNetWork::_getQuadIDFromD2(Pt2 inputCoordinate,int &quadID)
{
	double inputX = inputCoordinate.x();
	double inputY = inputCoordinate.y();
	int outputColumn = -1;
	int outputRow = -1;
	this->_getdistanceFromD1(inputX, _minX, _distanceBetweenQuadX, outputColumn);
	this->_getdistanceFromD1(inputY, _minY, _distanceBetweenQuadY, outputRow);

	quadID = outputRow * _sizeOfColumn + outputColumn;

}

//ͨ��ID���ҷֿ鼯�����ID�������μ���
mapTrianglesOfEachQuad quadNetWork::_getTriangleSetByQuadID(int quadId )
{
	mapTrianglesOfEachQuad theTrianglesSet;
	theTrianglesSet.clear();
	//�鿴�ֿ������Ƿ��и�ID
	mapTrianglesSetOfAllQuads::iterator
		iterCurQuad = _triangleSetOfAllQuads.find(quadId),
		iterEndQuad = _triangleSetOfAllQuads.end();
	if (iterCurQuad == iterEndQuad)
	{
		return theTrianglesSet;
	}

	theTrianglesSet = iterCurQuad->second;
	return theTrianglesSet;
}

//���������κ�ָ���ֿ�ID���ڸÿ���ϴ�������
bool quadNetWork::_addTriangleToQuadByQuadID(myTriangle theTriangle, int quadID)
{
	bool bFindQuad = false;
	//���Ҹ�ID�Ƿ��ڼ����д���
	mapTrianglesSetOfAllQuads::iterator
		iterFindQuadID = _triangleSetOfAllQuads.find(quadID),
		iterEndQuadID = _triangleSetOfAllQuads.end();
	if (iterFindQuadID == iterEndQuadID)
	{
		bFindQuad = false;
		return bFindQuad;
	}

	mapTrianglesOfEachQuad theQuadTriangles = iterFindQuadID->second;
	int id = theQuadTriangles.size();
	theQuadTriangles.insert(pairTrianglesOfEachQuad(id, theTriangle));
	iterFindQuadID->second.clear();
	iterFindQuadID->second = theQuadTriangles;
	return bFindQuad;
}

//������������������Ӿ��ε����꼫ֵ��4���������
std::vector<Pt2> quadNetWork::_getRectanglePt2VectorFromTriangle(myTriangle theTriangle)
{
	std::vector<Pt2> rectangleVector;
	rectangleVector.clear();
	rectangleVector.resize(4);

	std::vector<Pt3> trianglePt = theTriangle.pt3sInTriangle;

	//�������Сֵ
	double minX = trianglePt[0].x();
	double maxX = trianglePt[0].x();
	double minY = trianglePt[0].y();
	double maxY = trianglePt[0].y();

	std::vector<Pt3>::iterator
		iterCurPt = trianglePt.begin(),
		iterEndPt = trianglePt.end();
	for (; iterCurPt != iterEndPt; iterCurPt++)
	{
		Pt3 thePt = *iterCurPt;
		double x = thePt.x();
		double y = thePt.y();
		if (x < minX)
		{
			minX = x;
		}
		if (x > maxX)
		{
			maxX = x;
		}

		if (y > maxY)
		{
			maxY = y;
		}
		if (y < minY)
		{
			minY = y;
		}
	}

	//�����θ�ֵ
	Pt2 p0 = Pt2(minX, minY);
	rectangleVector[0] = p0;
	Pt2 p1 = Pt2(minX, maxY);
	rectangleVector[1] = p1;
	Pt2 p2 = Pt2(maxX, minY);
	rectangleVector[2] = p2;
	Pt2 p3 = Pt2(maxX, maxY);
	rectangleVector[3] = p3;

	return rectangleVector;
}

//������������������Ӿ��ε����꼫ֵ��4���������
std::vector<Pt2> quadNetWork::_getRectanglePt2VectorFromTriangle(std::vector<Pt3> trianglePt)
{
	std::vector<Pt2> rectangleVector;
	rectangleVector.clear();
	rectangleVector.resize(4);

	//�������Сֵ
	double minX = trianglePt[0].x();
	double maxX = trianglePt[0].x();
	double minY = trianglePt[0].y();
	double maxY = trianglePt[0].y();

	std::vector<Pt3>::iterator
		iterCurPt = trianglePt.begin(),
		iterEndPt = trianglePt.end();
	for (; iterCurPt != iterEndPt; iterCurPt++)
	{
		Pt3 thePt = *iterCurPt;
		double x = thePt.x();
		double y = thePt.y();
		if (x < minX)
		{
			minX = x;
		}
		if (x > maxX)
		{
			maxX = x;
		}

		if (y > maxY)
		{
			maxY = y;
		}
		if (y < minY)
		{
			minY = y;
		}
	}

	//�����θ�ֵ
	Pt2 p0 = Pt2(minX, minY);
	rectangleVector[0] = p0;
	Pt2 p1 = Pt2(minX, maxY);
	rectangleVector[1] = p1;
	Pt2 p2 = Pt2(maxX, minY);
	rectangleVector[2] = p2;
	Pt2 p3 = Pt2(maxX, maxY);
	rectangleVector[3] = p3;

	return rectangleVector;
}

//������������Ӿ��ζ������꼯�Ϻͷֿ�ϸ������������������ڵĿ�ID(���ܻ����飩
std::vector<int> quadNetWork::_getQuadIDSFromRectangle(std::vector<Pt2> rectVec)
{
	std::vector<int> idsVector;
	idsVector.clear();
	//�ֱ�ó����ε��ĸ������������ڵĵķֿ�ID
	std::vector<Pt2>::iterator
		iterCurVec = rectVec.begin(),
		iterEndVec = rectVec.end();
	for (; iterCurVec != iterEndVec; iterCurVec++)
	{
		int outputRow = -1;
		int outputColumn = -1;
		int quadID = -1;

		Pt2 thePos = *iterCurVec;
		this->_getQuadIDFromD2(thePos, quadID);
		idsVector.push_back(quadID);
	}

	//ȥ���ظ���
	sort(idsVector.begin(), idsVector.end());
	idsVector.erase(unique(idsVector.begin(), idsVector.end()), idsVector.end());

	return idsVector;
}

//���������κͷֿ������μ��ϣ��Լ�ָ���ֿ�ID���ϣ��ڸÿ���ϴ�������
bool quadNetWork::_addTriangleToQuadsByQuadIDs(myTriangle theTriangle, std::vector<int> quadIDVector)
{
	std::vector<int>::iterator
		iterCurQuadID = quadIDVector.begin(),
		iterEndQuadID = quadIDVector.end();
	for (; iterCurQuadID != iterEndQuadID; iterCurQuadID++)
	{
		int quadID = *iterCurQuadID;
		this->_addTriangleToQuadByQuadID(theTriangle, quadID);
	}
	return true;
}



//ָ����XY���ϵó�
//�����߶κ�ָ����ĳ���(�������������ǹ���)
int quadNetWork::testOrientationBetweenPointInSegment(Pt2 thePoint, Pt2 startPoint, Pt2 endPoint)
{
	int result = -100;
	switch (CGAL::orientation(startPoint, thePoint, endPoint))
	{
	case  CGAL::COLLINEAR:
		//����ʱ������Ҫ���ǵ����Ե���û���ڸ��߶ε��ӳ����ϣ����ӳ����ϣ�������Ϊ������������
	{

		double minX = startPoint.x();
		double minY = startPoint.y();
		double maxX = startPoint.x();
		double maxY = startPoint.y();

		if (endPoint.x() < minX)
		{
			minX = endPoint.x();
			maxX = startPoint.x();
		}
		if (endPoint.y() < minY)
		{
			minY = endPoint.y();
			maxY = startPoint.y();
		}

		if (endPoint.x() > maxX)
		{
			maxX = endPoint.x();
			minX = startPoint.x();
		}
		if (endPoint.y() > maxY)
		{
			maxY = endPoint.y();
			minY = startPoint.y();
		}

		if ((thePoint.x() <= maxX) &&
			(thePoint.x() >= minX) &&
			(thePoint.y() <= maxY) &&
			(thePoint.y() >= minY))
		{
			result = 0;
		}
	}

	break;

	case  CGAL::LEFT_TURN:
		result = 1;
		break;

	case CGAL::RIGHT_TURN:
		result = -1;
		break;

	default:
		break;
	}
	return result;

}

// ���������Ƿ�����������
bool quadNetWork::testRange(Pt2 thePoint, std::vector<Pt2> pt)
{
	bool bRanged = false;

	double maxX = pt[0].x();
	double maxY = pt[0].y();
	double minX = pt[0].x();
	double minY = pt[0].y();

	//��������ε����XYֵ
	std::vector<Pt2>::iterator
		iterCur = pt.begin(),
		iterEnd = pt.end();
	for (; iterCur != iterEnd; iterCur++)
	{
		Pt2 thePt = *iterCur;
		if (thePt.x() > maxX)
		{
			maxX = thePt.x();
		}
		if (thePt.y() > maxY)
		{
			maxY = thePt.y();
		}
		if (thePt.x() < minX)
		{
			minX = thePt.x();
		}
		if (thePt.y() < minY)
		{
			minY = thePt.y();
		}
	}

	//�жϸõ��Ƿ��ڷ�Χ��
	if ((thePoint.x() <= maxX) &&
		(thePoint.y() <= maxY) &&
		(thePoint.x() >= minX) &&
		(thePoint.y() >= minY)
		)
	{
		bRanged = true;
	}

	return bRanged;

}

// Determine whether two vectors v1 and v2 point to the same direction
// v1 = Cross(AB, AC)
// v2 = Cross(AB, AP)
bool quadNetWork::SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	Vector3 AB = B - A;
	Vector3 AC = C - A;
	Vector3 AP = P - A;

	Vector3 v1 = AB.Cross(AC);
	Vector3 v2 = AB.Cross(AP);

	// v1 and v2 should point to the same direction
	return v1.Dot(v2) >= 0;
}
int quadNetWork::getSignal(Vector3 p, Vector3 q, Vector3 r)
{
	double theValue = -1;

	Vector3 pq = q - p;
	double qX = q.x;
	double qY = q.y;
	double pX = p.x;
	double pY = p.y;
	double x = r.x;
	double y = r.y;
	double deltaX = abs(pX - qX);
	if ( deltaX <= 0.001 )
	{
		theValue = pX - x;
	}
	else
	{
		theValue = y - pq.y * 1.0 * x / pq.x - (qX * pY - pX * qY) * 1.0 / pq.x;
	}

	return theValue;
}
bool quadNetWork::SameSide2(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	bool bOneSide = false;

	double AB_C = this->getSignal(A, B, C);
	double AB_P = this->getSignal(A, B, P);
	double CP = AB_C * AB_P;

	double BC_A = this->getSignal(B, C, A);
	double BC_P = this->getSignal(B, C, P);
	double AP = BC_A * BC_P;

	double AC_B = this->getSignal(A, C, B);
	double AC_P = this->getSignal(A, C, P);
	double BP = AC_B * AC_P;

	
	if ( CP<0 || AP < 0 || BP <0 )
	{
		bOneSide = false;
	}
	else
	{
		bOneSide = true;
	}
	return bOneSide;

}

// Same side method
// Determine whether point P in triangle ABC
bool quadNetWork::testPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	return SameSide(A, B, C, P) &&
		SameSide(B, C, A, P) &&
		SameSide(C, A, B, P);
}
//�����������߶��Ƿ��ཻ
bool quadNetWork::computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint)
{
	bool bInsect = false;
	if (!CGAL::do_intersect(theSeg, theRay))
	{
		//std::cout << "do not intersect" << std::endl;
		bInsect = false;
		return bInsect;
	}

	CGAL::Object result = CGAL::intersection(theSeg, theRay);
	if (!CGAL::assign(intersectionPoint, result))
	{
		//std::cout << "���ǽ���" << std::endl;
		bInsect = false;
		return bInsect;
	}

	bInsect = true;
	return bInsect;
}

//���߶����˵�����õ������ĸ߶�
double quadNetWork::getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint)
{
	double startX = theSeg.start().x();
	double startY = theSeg.start().y();
	double startZ = theSeg.start().z();
	double endX = theSeg.end().x();
	double endY = theSeg.end().y();
	double endZ = theSeg.end().z();
	double insectPointX = inSectPoint.x();
	double insectPointY = inSectPoint.y();
	//Ҫ��ĸ߶�
	double inSectPointZ = 0;

	//��ֵ��������ĳ�˵㣬��ֱ��ȡֵ��
	double distanceStartPoint_Insection = sqrt((startX - insectPointX) * (startX - insectPointX) + (startY - insectPointY) * (startY - insectPointY));
	double distanceEndPoint_Insection = sqrt((endX - insectPointX) * (endX - insectPointX) + (endY - insectPointY) * (endY - insectPointY));

	double deltaAP = distanceStartPoint_Insection / (distanceStartPoint_Insection + distanceEndPoint_Insection);
	inSectPointZ = startZ + deltaAP * (endZ - startZ);

	return inSectPointZ;
}
//�ж������Ƿ��غ�
bool quadNetWork::bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3)
{
	bool bSame = false;

	double x1 = firstPt2.x();
	double y1 = firstPt2.y();
	double x2 = theComparedPt3.x();
	double y2 = theComparedPt3.y();

	//��ֵ��������ĳ�˵㣬��ֱ��ȡֵ��
	double distance = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	if (distance < 0.0001)   //���������A��
	{
		bSame = true;
		return bSame;
	}
	return bSame;
}

//�жϸõ������������Ƿ����غϵ�
bool quadNetWork::bInTheTriangle(Pt2 thePt2, std::vector<Pt3> pt3Vector, double& pointZ)
{
	bool bInTheTriangle = false;
	std::vector<Pt3>::iterator
		iterCur = pt3Vector.begin(),
		iterEnd = pt3Vector.end();
	for (; iterCur != iterEnd; iterCur++)
	{
		Pt3 thePt3 = *iterCur;
		if (bTheSamePoint(thePt2, thePt3))
		{
			break;
		}
	}
	if (iterCur == iterEnd)
	{
		bInTheTriangle = false;
		return bInTheTriangle;
	}

	Pt3 thePoint = *iterCur;
	pointZ = thePoint.z();
	bInTheTriangle = true;
	return bInTheTriangle;
}
//�����������VECTOR��XY���õ�ָ�����Zֵ
bool quadNetWork::getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ)
{
	bool bIntersect = false;

	Pt2 pA = Pt2(triangleVector[0].x(), triangleVector[0].y());
	Pt2 pB = Pt2(triangleVector[1].x(), triangleVector[1].y());
	Pt2 pC = Pt2(triangleVector[2].x(), triangleVector[2].y());
	Pt2 interSectionPoint(0, 0);
	Segment segAB(pA, pB);
	Ray2 rayCP(pC, thePt);

	bool bInTri = bInTheTriangle(thePt, triangleVector, pointZ);
	//�ȿ���������Ƿ������������ϣ�����ڣ����Ϸ�����
	if (bInTri)
	{
		bIntersect = true;
		return bIntersect;
	}

	bool bHasIntersectionPoint = computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
	if (!bHasIntersectionPoint)
	{
		//std::cout << "û�н���" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	//std::cout << "�������꣺��" << interSectionPoint << ")" << std::endl;

	//���β�ֵ��

	Pt3 vecA = triangleVector[0];
	Pt3 vecB = triangleVector[1];
	Pt3 vecC = triangleVector[2];

	Pt3 pA3(vecA.x(), vecA.y(), vecA.z());
	Pt3 pB3(vecB.x(), vecB.y(), vecB.z());
	Pt3 pC3(vecC.x(), vecC.y(), vecC.z());

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //�ó������X,Y,Z����
	//std::cout << "��ֵ��߶�Ϊ:" << intersectZ << std::endl;

	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = getPointZfromSeg(seg3_CInterSection, thePt);
	//std::cout << "�趨��߶�Ϊ:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}

// �����������VECTOR��XY���õ�ָ�����Zֵ
bool quadNetWork::getPointZAndTraingleVectorfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ)
{

	bool bIntersect = false;

	Pt2 pA = Pt2(triangleVector[0].x(), triangleVector[0].y());
	Pt2 pB = Pt2(triangleVector[1].x(), triangleVector[1].y());
	Pt2 pC = Pt2(triangleVector[2].x(), triangleVector[2].y());
	Pt2 interSectionPoint(0, 0);
	Segment segAB(pA, pB);
	Ray2 rayCP(pC, thePt);

	//�ȿ���������Ƿ������������ϣ�����ڣ����Ϸ�����
	if (bInTheTriangle(thePt, triangleVector, pointZ))
	{
		bIntersect = true;
		return bIntersect;
	}

	bool bHasIntersectionPoint = computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
	if (!bHasIntersectionPoint)
	{
		//std::cout << "û�н���" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	//std::cout << "�������꣺��" << interSectionPoint << ")" << std::endl;

	//���β�ֵ��������ĳ�˵㣬��ֱ��ȡֵ

	Pt3 vecA = triangleVector[0];
	Pt3 vecB = triangleVector[1];
	Pt3 vecC = triangleVector[2];

	Pt3 pA3(vecA.x(), vecA.y(), vecA.z());
	Pt3 pB3(vecB.x(), vecB.y(), vecB.z());
	Pt3 pC3(vecC.x(), vecC.y(), vecC.z());

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //�ó������X,Y,Z����
	//std::cout << "��ֵ��߶�Ϊ:" << intersectZ << std::endl;

	//ָ��������ĳ�˵㣬��ֱ��ȡֵ��
	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = getPointZfromSeg(seg3_CInterSection, thePt);
	Pt3 thePt3(thePt.x(), thePt.y(), pointZ);
	//std::cout << "�趨��߶�Ϊ:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}

//����ָ�����X,Y�͸÷ֿ�������μ��ϣ�������õ����ڵ������ζ�������
std::vector<Pt3> quadNetWork::getPt3VectorOftheTrianglefromthePointAndTriangleSetOfQuad(Pt2 thePt, mapTrianglesOfEachQuad theTriangleSetOfThisQuad)
{
	std::vector<Pt3> ptsInTheTriangle;
	ptsInTheTriangle.clear();

	Vector3 vecA(0, 0, 0);
	Vector3 vecB(0, 0, 0);
	Vector3 vecC(0, 0, 0);
	Vector3 vecPt(thePt.x(), thePt.y(), 0);
	//�жϵ����ĸ���������

	//1,���ж��Ƿ�����Ӿ�����
	std::vector<myTriangle> triVectorIncludePt;
	triVectorIncludePt.clear();
	
	mapTrianglesOfEachQuad::iterator
		iterCurTri = theTriangleSetOfThisQuad.begin(),
		iterEndTri = theTriangleSetOfThisQuad.end();
	for (; iterCurTri != iterEndTri; iterCurTri++)
	{
		myTriangle theTri = iterCurTri->second;
		myRect theRect = theTri.theRect;
		bool bPtInRect = testPtInRect(thePt, theRect);
		if ( bPtInRect)
		{
			triVectorIncludePt.push_back(theTri);
		}
	}

	//2������Ӿ����ڵ����ж��Ƿ�����������
	std::vector<myTriangle>::iterator
		iterTriangleCur = triVectorIncludePt.begin(),
		iterTriangleEnd = triVectorIncludePt.end();
	for (; iterTriangleCur != iterTriangleEnd; iterTriangleCur++)
	{
		myTriangle theTr = *iterTriangleCur;
		std::vector<Pt3> thevec = theTr.pt3sInTriangle;

		vecA.x = thevec[0].x();
		vecA.y = thevec[0].y();
		vecB.x = thevec[1].x();
		vecB.y = thevec[1].y();
		vecC.x = thevec[2].x();
		vecC.y = thevec[2].y();

		Pt2 p0 = Pt2(thevec[0].x(), thevec[0].y());
		Pt2 p1 = Pt2(thevec[1].x(), thevec[1].y());
		Pt2 p2 = Pt2(thevec[2].x(), thevec[2].y());

		bool bTestInOrNot = testPointInTriangle(vecA, vecB, vecC, vecPt);
	
		if (bTestInOrNot)
		{
			std::cout << "��(" << thePt << ")��������" <<"(" << thevec[0] <<")��" << std::endl;
			break;
		}
	}

	//������ID
	if (iterTriangleCur == iterTriangleEnd)
	{
		//std::cout << "��" << thePt << "û������������" << std::endl;
		return ptsInTheTriangle;
	}

	ptsInTheTriangle = (*iterTriangleCur).pt3sInTriangle;

	return ptsInTheTriangle;
}

//�������ݼ��ϣ����ǻ�����������μ���
triangleSet quadNetWork::getTriangleSetFromDataSet(pt3Set dataSet)
{
	//���ǻ����ݼ����е�X,Y����

	typedef std::map<Delaunay::Vertex_handle, Pt3> mapVertexHandleAndPt3;
	typedef std::pair<Delaunay::Vertex_handle, Pt3> pairVertexHandleAndPt3;
	mapVertexHandleAndPt3 vertexHandleAndPt3Set;
	vertexHandleAndPt3Set.clear();

	Delaunay tr;
	pt3Set::iterator iterCurVector3 = dataSet.begin(),
		iterEndVector3 = dataSet.end();
	for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
	{
		int id = iterCurVector3->first;
		Pt3 thePt3 = iterCurVector3->second;
		double x = thePt3.x();
		double y = thePt3.y();

		Pt2 xy = Pt2(x, y);
		Delaunay::Vertex_handle theVertexHandle = tr.insert(xy);
		vertexHandleAndPt3Set.insert(pairVertexHandleAndPt3(theVertexHandle, thePt3));
	}

	//���ǻ����XY�������Zֵ�������μ���
	int faceID = 0;	//���ID
	triangleSet myTriangleSet;
	myTriangleSet.clear();

	//std::cout << "���������Ķ���:" << std::endl;
	Delaunay::Finite_faces_iterator faceCur = tr.finite_faces_begin(),
		faceEnd = tr.finite_faces_end();
	for (; faceCur != faceEnd; faceCur++)
	{
		//���ݸ���������������Zֵ
		std::vector<Pt3> theVector;
		theVector.clear();
		theVector.resize(3);
		for (int i = 0; i < 3; i++)
		{
			Delaunay::Vertex_handle theVertexHandle = faceCur->vertex(i);
			Pt3 thePt3 = vertexHandleAndPt3Set[theVertexHandle];
			theVector[i] = thePt3;

		}

		myTriangleSet.insert(trianglePair(faceID, theVector));
		faceID++;
	}

	return myTriangleSet;
}

//�������ݼ��ϣ����ǻ���������ֿ�������μ���
void quadNetWork::getTriangleSetOfEachQuadFromDataSet(pt3Set dataSet)
{
	//���ǻ����ݼ����е�X,Y����
	Delaunay tr;
	pt3Set::iterator iterCurVector3 = dataSet.begin(),
		iterEndVector3 = dataSet.end();
	for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
	{
		double x = iterCurVector3->second.x();
		double y = iterCurVector3->second.y();

		Pt2 xy = Pt2(x, y);
		tr.insert(xy);
	}
	//���ǻ����XY�������Zֵ�������μ���
	//std::cout << "���������Ķ���:" << std::endl;
	Delaunay::Finite_faces_iterator faceCur = tr.finite_faces_begin(),
		faceEnd = tr.finite_faces_end();
	for (; faceCur != faceEnd; faceCur++)
	{
		//���ݸ���������������Zֵ
		std::vector<Pt3> theVector;
		theVector.clear();
		theVector.resize(3);
		for (int i = 0; i < 3; i++)
		{
			double x = faceCur->vertex(i)->point().hx();
			double y = faceCur->vertex(i)->point().hy();
			double z = 0;
			//����XY����Zֵ
			pt3Set::iterator iterCurVector3 = dataSet.begin(),
				iterEndVector3 = dataSet.end();
			for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
			{
				double xRef = iterCurVector3->second.x();
				double yRef = iterCurVector3->second.y();
				double deltaDistance = sqrt((x - xRef) * (x - xRef) + (y - yRef) * (y - yRef));
				if (deltaDistance < 0.001)
				{
					z = iterCurVector3->second.z();
				}
			}

			double theX = faceCur->vertex(i)->point().hx();
			double theY = faceCur->vertex(i)->point().hy();
			double theZ = z;
			Pt3 vec(Pt3(theX, theY, theZ));
			theVector[i] = vec;

			//std::cout << "���ǻ�����������������,��" << faceID << ",����:" << theVector[i] << std::endl;
		}

		//���������η�����Ӧ�ķֿ���
		myTriangle theTriangle;
		theTriangle.pt3sInTriangle = theVector;
		this->addTriangleToTrianglesSetOfAllQuads(theTriangle);

	}

}
//�����ݼ��Ϻ������漯���еó������ĸ߶ȺͲ���������ε�
bool quadNetWork::getPt3InsertAndPt3VecInOneTriangleFromDataSet(pt3Set dataSet, Pt2 thePoint, Pt3& outPutPoint3d, std::vector<Pt3>& trianglePointVector)
{
	bool bPointInPointCloud = false;

	//ͨ����ά���꣬�����Ӧ�ֿ���������μ���
	mapTrianglesOfEachQuad triangleSetOfThisQuad = this->getTriangleSetByCoordinateXY(thePoint);
	//����������������е�����
	trianglePointVector = this->getPt3VectorOftheTrianglefromthePointAndTriangleSetOfQuad(thePoint, triangleSetOfThisQuad);
	//��û�õ�����˵��û���ڵ�����
	int sizeofTrianlgePts = trianglePointVector.size();
	if (sizeofTrianlgePts == 0)
	{
		//std::cout << "��(" << thePoint << ")û������������" << std::endl;
		bPointInPointCloud = false;
		return bPointInPointCloud;
	}
	
	/*
	printf("(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f)\n",
	trianglePointVector[0].x(), trianglePointVector[0].y(), trianglePointVector[0].z(),
	trianglePointVector[1].x(), trianglePointVector[1].y(), trianglePointVector[1].z(),
	trianglePointVector[2].x(), trianglePointVector[2].y(), trianglePointVector[2].z());
	*/
	double pointZ = -1;
	bool bInterSect = getPointZfromTriangleVector(trianglePointVector, thePoint, pointZ);
	if (bInterSect)
	{
		//printf("��ֵ������Ϊ(%0.6f,%0.6f,%0.6f)\n", thePoint.x(), thePoint.y(), pointZ);
		outPutPoint3d = Pt3(thePoint.x(), thePoint.y(), pointZ);
		bPointInPointCloud = true;
		return bPointInPointCloud;
	}

	return bPointInPointCloud;
}

//�ӵ������ݼ��Ϻ�����XYvector�������μ�����,�õ�դ�����ά����
rasterVec3Set quadNetWork::getPt3SetFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set)
{
	rasterVec3Set theSet;
	theSet.clear();

	rasterVec2Set::iterator
		iterPt2Cur = Pt2Set.begin(),
		iterPt2End = Pt2Set.end();
	for (; iterPt2Cur != iterPt2End; iterPt2Cur++)
	{
		int id = iterPt2Cur->first;
		std::vector<Pt2> theVec2 = iterPt2Cur->second;
		std::vector<Pt3> theVec3;
		theVec3.clear();
		std::vector<Pt2>::iterator
			thePt2Cur = theVec2.begin(),
			thePt2End = theVec2.end();
		for (; thePt2Cur != thePt2End; thePt2Cur++)
		{
			Pt2 thePoint = *thePt2Cur;
			Pt3 pointXYZ(0, 0, 0);
			std::vector<Pt3> triangleVec;
			triangleVec.clear();
			bool bPtInCloud = getPt3InsertAndPt3VecInOneTriangleFromDataSet(dataSet, thePoint, pointXYZ, triangleVec);
			if (bPtInCloud)
			{
				theVec3.push_back(pointXYZ);
			}
			else
			{
				//û�ڵ����ھ͸�ֵΪ0
				double x = thePoint.x();
				double y = thePoint.y();
				double z = 0;
				theVec3.push_back(Pt3(x, y, z));
			}
		}
		theSet.insert(rasterVec3Pair(id, theVec3));
	}

	return theSet;
}

//�������뼯����ȷֲ�ֵXY����
rasterVec2Set quadNetWork::getEqualXYVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution)
{
	//����������������ά��������
	rasterVec2Set myRaster;
	myRaster.clear();

	for (int i = 0; i < ySize; i++)
	{
		std::vector<Pt2> imagePointVector2;
		imagePointVector2.clear();
		for (int j = 0; j < xSize; j++)
		{
			double x = minX + xResolution * j;
			double y = minY + yResolution * i;
			Pt2 thisPoint = Pt2(x, y);
			imagePointVector2.push_back(thisPoint);
			//std::cout << "�ӵ������룬�����ά����:(" << i << "," << j << ")=" << thisPoint << std::endl;
		}
		myRaster.insert(rasterVec2Pair(i, imagePointVector2));
	}

	return myRaster;
}
//�������뼯����ȷֲ�ֵXYZ����
void quadNetWork::getEqualXYZVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution)
{
	_rasterVecVec.resize(ySize);
	//�����������������ά��������

	for (int i = 0; i < ySize; i++)
	{
		//_rasterVecVec[i].resize(xSize);
		std::vector<Pt3> imagePointVector3;
		imagePointVector3.clear();
		for (int j = 0; j < xSize; j++)
		{
			double x = minX + xResolution * j;
			double y = minY + yResolution * i;
			double z = -9999;			//�Ҷ�ֵ��ֵ��Ϊ0
			Pt3 thisPoint = Pt3(x, y,z);
			imagePointVector3.push_back(thisPoint);
			//std::cout << "�ӵ������룬�����ά����:(" << i << "," << j << ")=" << thisPoint << std::endl;
		}
		//_rasterVec3Set.insert(rasterVec3Pair(i, imagePointVector3));

		_rasterVecVec[i] = imagePointVector3;
	}

}
//ͨ��դ�񴴽�TIF�ļ�
void quadNetWork::createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, double xResolution, double yResolution, double topLeftX, double topLeftY, double rotationX, double rotationY)
{
	//�趨֧������·��
	CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
	//ע��դ������
	GDALAllRegister();
	//�����ȡָ����ʽ�����������ڴ���ͼ��
	const char * pszFormat = "GTiff";
	GDALDriver * poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if (poDriver == NULL)
	{
		std::cout << "��ʽ" << pszFormat << "��֧��create()����" << std::endl;
	}
	else
	{
		std::cout << "����OK��" << std::endl;
	}

	//��ȡ��������һЩԪ������Ϣ
	char ** papszMetadata = poDriver->GetMetadata();
	if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
	{
		std::cout << pszFormat << "֧��Create()����" << std::endl;
	}
	if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
	{
		std::cout << pszFormat << "֧��CreateCopy()����" << std::endl;
	}

	//��������ļ�����СΪ512*512*3����������8bit������
	GDALDataset * poDS = poDriver->Create(strImageName, xSize, ySize, bandSize, GDT_Float32, NULL);
	if (poDS == NULL)
	{
		std::cout << "����ͼ��" << strImageName << "ʧ��" << std::endl;
		return;
	}
	//��������������һ���͵��ĸ���ͼ�����Ͻǵ����꣬��2���͵�6��Ϊ���������ֱ��ʣ���������Ϊ��ת�Ƕ�
	double dGeotransform[6] = { 0, 0, 0, 0, 0, 0 };
	dGeotransform[0] = topLeftX;   //��������X
	dGeotransform[1] = xResolution;	//����ֱ���
	dGeotransform[2] = rotationX;	//��ת�Ƕ�
	dGeotransform[3] = topLeftY;	//��������Y
	dGeotransform[4] = rotationY;	//��ת�Ƕ�
	dGeotransform[5] = yResolution * (-1.0);	//y�ֱ��ʣ���ֵ��

	poDS->SetGeoTransform(dGeotransform);

	GDALClose((GDALDatasetH)poDS);
	std::cout << "���ݼ��رգ�" << std::endl;


}
//ͨ������դ�����ݸ���TIFF�ļ�
void quadNetWork::UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
{
	//�趨֧������·��
	CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
	//ע��դ������
	GDALAllRegister();
	//��Ҫ���µ����ݣ�ע��ڶ�������ʹ��GA_UPDATE
	GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
	if (poDS == NULL)
	{
		std::cout << "��ͼ��" << strImageName << "ʧ��" << std::endl;
	}

	//��ȡͼ���С
	int iWidth = poDS->GetRasterXSize();
	int iHeight = poDS->GetRasterYSize();

	//����һ��ͼ���С�Ŀռ�
	float * pBuf = new float[iWidth];
	memset(pBuf, 0, sizeof(float) * iWidth);
	//����һ������ֵ��Vector,���Ը�TIF�ļ�ÿ�и�ֵ
	std::vector<float> imagePerLineVector;
	imagePerLineVector.clear();

	//��ȡ��һ����
	GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
	//ѭ��ͼ��ߣ�����ͼ�����������ֵ
	rasterVec3Set::iterator
		iterRasterCur = theRasterSet.begin(),
		iterRasterEnd = theRasterSet.end();
	for (; iterRasterCur != iterRasterEnd; iterRasterCur++)
	{
		//���µߵ���
		int id = iHeight - 1 - iterRasterCur->first;
		std::vector<Pt3> theLine = iterRasterCur->second;

		//��������ڴ�
		imagePerLineVector.clear();
		memset(pBuf, 0, sizeof(float) * iWidth);

		std::vector<Pt3>::iterator
			iterCurLine = theLine.begin(),
			iterEndLine = theLine.end();
		for (; iterCurLine != iterEndLine; iterCurLine++)
		{
			Pt3 theRasterPoint = *iterCurLine;
			double theValue = theRasterPoint.z();
			//float theValue = 255;
			//���������ظ��Ҷ�ֵ��VECTOR
			imagePerLineVector.push_back(theValue);
		}

		//����������ֵ���ڴ�
		memcpy(pBuf, (float*)&imagePerLineVector[0], sizeof(float) * iWidth);
		//memset(pBuf, 255, sizeof(float) * iWidth);
		//����Ԫֵд��ͼ��
		//pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, pBuf, iWidth, 1, GDT_Byte, 0, 0);
		pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, pBuf, iWidth, 1, GDT_Float32, 0, 0);
	}
	//��������ڴ�
	imagePerLineVector.clear();
	memset(pBuf, 0, sizeof(float) * iWidth);

	GDALClose((GDALDatasetH)poDS);
}

//�жϵ��Ƿ��ھ�����
bool quadNetWork::testPtInRect(Pt2 thePt, myRect theRect)
{
	bool bInRect = false;

	double ptX = thePt.x();
	double ptY = thePt.y();

	double minX = theRect.minX;
	double minY = theRect.minY;
	double maxX = theRect.maxX;
	double maxY = theRect.maxY;
	
	if( ( ptX <= maxX) &&
		(ptX >= minX) && 
		(ptY <= maxY) && 
		(ptY >= minY) )
	{
		bInRect = true;
	}

	return bInRect;
}

//��ԭ��������ֳɼ�������
std::vector<std::vector<Pt2>> quadNetWork::getVectorOfEachPart(int partNumber, std::vector<Pt2> originalVector)
{
	int numberOfPartition = partNumber;
	int totalNumber = originalVector.size();
	int numberOfEachQuad = totalNumber / numberOfPartition;
	typedef std::vector<Pt2> pt2Vector;
	std::vector<pt2Vector> pt2VectorSet;
	pt2VectorSet.clear();
	pt2VectorSet.resize(numberOfPartition);
	for (int i = 0; i < numberOfPartition - 1; i++)
	{
		std::vector<Pt2> theVector(originalVector.begin() + i * numberOfEachQuad, originalVector.begin() + numberOfEachQuad * (i + 1));
		pt2VectorSet[i] = theVector;

	}
	std::vector<Pt2> lastV(originalVector.begin() + numberOfEachQuad * (numberOfPartition - 1), originalVector.end());
	pt2VectorSet[numberOfPartition - 1] = lastV;

	return pt2VectorSet;

}
//���к�Ϊid���е�����һ���ֽ����������
void quadNetWork::processOnePart(pt3Set dataSet, std::vector<Pt2> theVec2, int id)
{
	
	std::vector<Pt2>::iterator
		thePt2Cur = theVec2.begin(),
		thePt2End = theVec2.end();
	for (; thePt2Cur != thePt2End; thePt2Cur++)
	{
		Pt2 thePoint = *thePt2Cur;
		Pt3 pointXYZ(0, 0, 0);
		std::vector<Pt3> triangleVec;
		triangleVec.clear();
		bool bPtInCloud = getPt3InsertAndPt3VecInOneTriangleFromDataSet(dataSet, thePoint, pointXYZ, triangleVec);
		if (bPtInCloud)
		{
			_rasterVec3Set[id].push_back(pointXYZ);
		}
		else
		{
			//û�ڵ����ھ͸�ֵΪ0
			double x = thePoint.x();
			double y = thePoint.y();
			double z = 0;
			_rasterVec3Set[id].push_back(Pt3(x, y, z));
		}
	}
	
}

//���м���
void quadNetWork::parrelProcessVector(pt3Set dataSet, std::vector<Pt2> originalVector, int id)
{
	_rasterVec3Set[id].clear();
	const int numberOfPartition = 10;
	std::vector<std::vector<Pt2> > partVectorSet = getVectorOfEachPart(numberOfPartition, originalVector);
	std::thread threads[numberOfPartition];
	for (int i = 0; i < numberOfPartition; i++)
	{
		//threads[i] = std::thread(&this->processOnePart,partVectorSet[i] );
		threads[i] = std::thread(&quadNetWork::processOnePart,this, dataSet,partVectorSet[i], id);
		threads[i].join();
	}
}
//�����Ϸ�Ϊ��������
std::vector<std::vector<Pt2>> quadNetWork::getVectorFromSet(rasterVec2Set theSet)
{
	typedef std::vector<Pt2> quadVector;
	std::vector<quadVector> theVector;
	theVector.clear();
	rasterVec2Set::iterator
		iterCur = theSet.begin(),
		iterEnd = theSet.end();
	for (; iterCur != iterEnd; iterCur++)
	{
		quadVector curVec = iterCur->second;
		theVector.push_back(curVec);
		
	}
	return theVector;
}	
//��ԭ���Ĳ��ҵ�Ķ�ά���꼯�ϲ��м���
void quadNetWork::parrelProcessSet(pt3Set dataSet, rasterVec2Set originalSet)
{
	_rasterVec3Set.clear();
	//����ÿ��
	std::map<int, std::vector<Pt2> >::iterator
		iterCurLine = originalSet.begin(),
		iterEndLine = originalSet.end();
	for (; iterCurLine != iterEndLine; iterCurLine++)
	{
		int id = iterCurLine->first;
		std::vector<Pt2> originalVector = iterCurLine->second;
		this->parrelProcessVector( dataSet,originalVector, id);
	}
}

//�ӵ������ݼ��Ϻ�����XYvector�������μ�����,���д���õ�դ�����ά����
void quadNetWork::getPt3SetParrelFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set)
{
	_rasterVec3Set.clear();

	rasterVec2Set::iterator
		iterPt2Cur = Pt2Set.begin(),
		iterPt2End = Pt2Set.end();
	for (; iterPt2Cur != iterPt2End; iterPt2Cur++)
	{
		int id = iterPt2Cur->first;

		//���д������
		std::vector<Pt2> theVec2 = iterPt2Cur->second;
		this->parrelProcessVector( dataSet,theVec2, id);

	}

}

//������άդ�񼯺�
rasterVec3Set quadNetWork::getRasterVec3Set()
{
	return _rasterVec3Set;
}

std::vector<std::vector<Pt3>> quadNetWork::getrasterVecVec()
{
	return _rasterVecVec;
}
//���������μ��ϼ����դ�񼯺ϣ���ֵ
void quadNetWork::getRasterVec3SetFromTriangleSet(triangleSet theTriangleSet, double xResolution, double yResolution, int xSize, int ySize)
{
	//���ݸ��������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
	triangleSet::iterator
		iterCurTriangle = theTriangleSet.begin(),
		iterEndTrianlge = theTriangleSet.end();
	for (; iterCurTriangle != iterEndTrianlge; iterCurTriangle++)
	{
		std::vector<Pt3> theTrianglePoint3 = iterCurTriangle->second;
		this->getRasterVec3SetFromTriangle(theTrianglePoint3, xResolution, yResolution,xSize, ySize);
	}

	//���դ�񼯺�
	_rasterVec3Set.clear();
	for (int j = 0; j < ySize; j++)
	{
		std::vector<Pt3> thePt3Vec;
		thePt3Vec.clear();
		for (int i = 0; i < xSize; i++)
		{
			Pt3 thePt = _rasterVecVec[j][i];
			thePt3Vec.push_back(thePt);
		}
		_rasterVec3Set.insert(rasterVec3Pair(j, thePt3Vec));
	}
}

//�õ���ȡ��map����(minx,miny,max,maxy)
std::map<int, std::vector<Pt3>> quadNetWork::getSelectedRegion3(double minY, double maxY, double yResolution, std::map<int, std::vector<Pt3>> inputMap)
{
	std::map<int, std::vector<Pt3>> outPutSet;
	outPutSet.clear();
	std::map<int, std::vector<Pt3>>::iterator
		iterCur = inputMap.begin(),
		iterEnd = inputMap.end(),
		iterMin = inputMap.begin(),
		iterMax = inputMap.begin();

	int minRow = (minY - _minY) / yResolution;
	int maxRow = (maxY - _minY) / yResolution;

	for (int i = 0; i < minRow; i++)
	{
		iterMin++;
	}

	for (int i = 0; i < maxRow; i++)
	{
		iterMax++;
	}

	iterCur = iterMin;
	iterEnd = iterMax;
	iterEnd++;
	for (; iterCur != iterEnd; iterCur++)
	{
		int first = iterCur->first;
		std::vector<Pt3> second = iterCur->second;
		outPutSet.insert(std::pair<int, std::vector<Pt3>>(first, second));
	}
	return outPutSet;
}

//�õ���ȡ�Ĵ���ŵ�vector����
std::vector<Pt3WithColumnID> quadNetWork::getSelectedVector(double minX, double maxX, double xResolution, std::vector<Pt3> inputVector)
{
	std::vector<Pt3WithColumnID> outPutVec;
	outPutVec.clear();
	std::vector<Pt3>::iterator
		iterCur = inputVector.begin(),
		iterEnd = inputVector.end(),
		iterMin = inputVector.begin(),
		iterMax = inputVector.begin();

	int minCol = (minX - _minX) / xResolution;
	int maxCol = (maxX - _minX) / xResolution;

	for (int i = 0; i < minCol; i++)
	{
		iterMin++;
	}

	for (int i = 0; i < maxCol; i++)
	{
		iterMax++;
	}

	iterCur = iterMin;
	iterEnd = iterMax;
	//iterEnd++;
	for (int col = minCol; iterCur != iterEnd; iterCur++, col++)
	{
		Pt3 first = *iterCur;
		Pt3WithColumnID thePt;
		thePt.col = col;
		thePt.pos = first;

		outPutVec.push_back(thePt);
	}
	return outPutVec;
}
//����xy�ķ�Χ���Լ�λ�ü��ϣ����õ�����ŵķ�Χ����
std::map<int, std::vector<Pt3WithColumnID>> quadNetWork::getPt3WithColumnIDSetFromPt3SetAndXYRange(double minX, double minY,
	double maxX, double maxY, double xResolution, double yResolution, std::map<int, std::vector<Pt3>> inputMap
	)
{
	std::map<int, std::vector<Pt3WithColumnID>> ptWithColumnSet;
	ptWithColumnSet.clear();

	std::map<int, std::vector<Pt3>> selectedSetByRow = this->getSelectedRegion3(minY, maxY, yResolution, inputMap);
	std::map<int, std::vector<Pt3>>::iterator
		iterCur = selectedSetByRow.begin(),
		iterEnd = selectedSetByRow.end();
	for (; iterCur != iterEnd; iterCur++)
	{
		int id = iterCur->first;
		std::vector<Pt3> curPt3ColVec = iterCur->second;
		//�����к����жϽ�ȡ��һ����
		//std::vector<pt3>�õ�����ŵļ���
		std::vector<Pt3WithColumnID> ptWithColumnIDVec = this->getSelectedVector(minX, maxX, xResolution, curPt3ColVec);
		ptWithColumnSet.insert(std::pair<int, std::vector<Pt3WithColumnID>>(id, ptWithColumnIDVec));

	}
	return ptWithColumnSet;

}

//�������ŵĵ�
Pt3WithColumnID quadNetWork::processPt3WithColumnID(Pt3WithColumnID inputPt, Vector3 vecA, Vector3 vecB, Vector3 vecC, std::vector<Pt3> theTrianglePt3 )
{
	int col = inputPt.col;
	Pt3 thePt3 = inputPt.pos;
	double x = thePt3.x();
	double y = thePt3.y();
	Pt2 thePt2 = Pt2(x, y);
	//�ж��Ƿ�����������
	Vector3 vecPt(x, y, 0);
	bool bTestInOrNot = testPointInTriangle(vecA, vecB, vecC, vecPt);
	if (bTestInOrNot)
	{
		//�������������ֵ
		double pointZ = -1;
		bool bInterSect = getPointZfromTriangleVector(theTrianglePt3, thePt2, pointZ);
		if (bInterSect)
		{
			//printf("��ֵ������Ϊ(%0.6f,%0.6f,%0.6f)\n", thePoint.x(), thePoint.y(), pointZ);
			thePt3 = Pt3(x, y, pointZ);

		}

	}
	inputPt.pos = thePt3;
	return inputPt;
}
//�������ż���
std::map<int, std::vector<Pt3WithColumnID>> quadNetWork::processSetWithColumnID(std::map<int, std::vector<Pt3WithColumnID>> inputSet, Vector3 vecA, Vector3 vecB, Vector3 vecC, std::vector<Pt3> theTrianglePt3)
{
	//�������ŵĶ�����
	std::map<int, std::vector<Pt3WithColumnID>>::iterator
		iterCurPtWithID = inputSet.begin(),
		iterEndPtWithID = inputSet.end();
	for (; iterCurPtWithID != iterEndPtWithID; iterCurPtWithID++)
	{
		std::vector<Pt3WithColumnID> outVec;
		outVec.clear();
		int id = iterCurPtWithID->first;
		std::vector<Pt3WithColumnID> pt3Vec = iterCurPtWithID->second;
		std::vector<Pt3WithColumnID>::iterator
			iterCurPt = pt3Vec.begin(),
			iterEndPt = pt3Vec.end();
		for (; iterCurPt != iterEndPt; iterCurPt++)
		{
			Pt3WithColumnID thePt = *iterCurPt;
			//����ô���ŵĵ�
			Pt3WithColumnID newPt = this->processPt3WithColumnID(thePt, vecA, vecB, vecC, theTrianglePt3);
			outVec.push_back(newPt);

		}
		inputSet[id] = outVec;
	}
	return inputSet;

}
//����ԭ���ļ���
void quadNetWork::convertToOriginSet( std::map<int, std::vector<Pt3WithColumnID>> selectedSet)
{
	std::map<int, std::vector<Pt3WithColumnID>>::iterator
		iterCurPtWithID = selectedSet.begin(),
		iterEndPtWithID = selectedSet.end();
	for (; iterCurPtWithID != iterEndPtWithID; iterCurPtWithID++)
	{
		int id = iterCurPtWithID->first;
		std::vector<Pt3WithColumnID> pt3Vec = iterCurPtWithID->second;

		std::map<int, std::vector<Pt3>>::iterator
			iterFindPt3Vec = _rasterVec3Set.find(id),
			iterEndPt3Vec = _rasterVec3Set.end();
		if (iterFindPt3Vec != iterEndPt3Vec)
		{
			std::vector<Pt3> outVec = iterFindPt3Vec->second;

			std::vector<Pt3WithColumnID>::iterator
				iterCurPt = pt3Vec.begin(),
				iterEndPt = pt3Vec.end();
			for (; iterCurPt != iterEndPt; iterCurPt++)
			{
				Pt3WithColumnID thePt = *iterCurPt;
				int col = thePt.col;
				Pt3 pos = thePt.pos;
				outVec[col] = pos;


			}
			_rasterVec3Set[id] = outVec;
		}

	}

}
//���ݸ��������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
void quadNetWork::getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3, double xResolution, double yResolution, int xSize, int ySize)
{
	//1,��������ε���Ӿ���
	std::vector<Pt2> rectVec2 = this->_getRectanglePt2VectorFromTriangle(theTrianglePt3);
	double minX = rectVec2[0].x();
	double minY = rectVec2[0].y();
	double maxX = rectVec2[3].x();
	double maxY = rectVec2[3].y();
	myRect theRect;
	theRect.minX = minX;
	theRect.minY = minY;
	theRect.maxX = maxX;
	theRect.maxY = maxY;

	Vector3 vecA(theTrianglePt3[0].x(), theTrianglePt3[0].y(), 0);
	Vector3 vecB(theTrianglePt3[1].x(), theTrianglePt3[1].y(), 0);
	Vector3 vecC(theTrianglePt3[2].x(), theTrianglePt3[2].y(), 0);

	//�жϵ����ĸ���������

	//2������դ�񼯺ϣ����դ����ĵ��ڴ���Ӿ��η�Χ�ڣ����ж��Ƿ����������ȷ���Ƿ��ֵ

	//������С��Χ

	int minRow = (minY - _minY) / yResolution;
	int maxRow = (maxY - _minY) / yResolution;		
	int minCol = (minX - _minX) / xResolution;
	int maxCol = (maxX - _minX) / xResolution;
	if (maxRow >= ySize)
	{
		maxRow = ySize - 1;
	}
	if (maxCol >= xSize)
	{
		maxCol = xSize - 1;
	}
	if (minRow >= ySize)
	{
		minRow = ySize - 1;
	}
	if (minCol >= xSize)
	{
		minCol = xSize - 1;
	}

	for (int j = minRow; j <= maxRow; j++)
	{
		for (int i = minCol; i <= maxCol; i++ )
		{
			Pt3 thePt3 = _rasterVecVec[j][i];
			
			double x = thePt3.x();
			double y = thePt3.y();
			Pt2 thePt2 = Pt2(x, y);
			//�ж��Ƿ�����������
			Vector3 vecPt(x, y, 0);
			bool bTestInOrNot = testPointInTriangle(vecA, vecB, vecC, vecPt);
			if (bTestInOrNot)
			{
				//�������������ֵ
				double pointZ = -1;
				bool bInterSect = getPointZfromTriangleVector(theTrianglePt3, thePt2, pointZ);
				if (bInterSect)
				{
					//printf("��ֵ������Ϊ(%0.6f,%0.6f,%0.6f)\n", thePoint.x(), thePoint.y(), pointZ);
					thePt3 = Pt3(x, y, pointZ);
					
				}

			}
			_rasterVecVec[j][i] = thePt3;

		}
	}

}