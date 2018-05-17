#include "PCLTif.h"

PCLTif::PCLTif(double xResolution, double yResolution)
{
	_rasterVecVec.clear();
	_rasterVec3Set.clear();

	_xResolution = xResolution;
	_yResolution = yResolution;


}

PCLTif::~PCLTif()
{
	_rasterVecVec.clear();
	_rasterVec3Set.clear();
}
//���õ���ϸ��
void PCLTif::setPointCloudDetail(PCLDetail theDetail)
{
	_theDetail = theDetail;
}

//���ݵ��Ƽ�������Ƶ�ϸ������
PCLDetail PCLTif::getPCLDetail(pt3Set dataSet)
{
	//�ж����뼯�ϵķ�Χ
	//���ȸ������뼯�ϵ�X,Y����
	pt3Set::iterator
		iterInputExtentCur = dataSet.begin(),
		iterInputExtentEnd = dataSet.end();

	//�ж�XY����С���ֵ���Դ��ж���Χ
	iterInputExtentCur = dataSet.begin();
	double minX = iterInputExtentCur->second.x();
	double minY = iterInputExtentCur->second.y();
	double maxX = iterInputExtentCur->second.x();
	double maxY = iterInputExtentCur->second.y();

	for (; iterInputExtentCur != iterInputExtentEnd; iterInputExtentCur++)
	{
		double x = iterInputExtentCur->second.x();
		double y = iterInputExtentCur->second.y();

		if (x < minX)
		{
			minX = x;
		}
		if (x > maxX)
		{
			maxX = x;
		}
		if (y < minY)
		{
			minY = y;
		}
		if (y > maxY)
		{
			maxY = y;
		}
	}

	std::cout << "X�ķ�Χ:(" << minX << "," << maxX << ")" << std::endl;
	std::cout << "Y�ķ�Χ:(" << minY << "," << maxY << ")" << std::endl;

	//�ж����ݼ��Ĵ�С��
	double distanceX = maxX - minX;
	double distanceY = maxY - minY;

	PCLDetail theDetail;
	theDetail.minX = minX;
	theDetail.minY = minY;
	theDetail.maxX = maxX;
	theDetail.maxY = maxY;
	theDetail.xDistance = distanceX;
	theDetail.yDistance = distanceY;

	return theDetail;

}
//�������뼯����ȷֲ�ֵXYZ����
void PCLTif::getEqualXYZVectorFromDataSet(pt3Set dataSet, int xSize, int ySize )
{
	_rasterVecVec.resize(ySize);

	//�����������������ά��������
	for (int i = 0; i < ySize; i++)
	{
		std::vector<Pt3> imagePointVector3;
		imagePointVector3.clear();
		for (int j = 0; j < xSize; j++)
		{
			double x = _theDetail.minX + _xResolution * j;
			double y = _theDetail.minY + _yResolution * i;
			double z = -9999;			//�Ҷ�ֵ��ֵ��Ϊ0
			Pt3 thisPoint = Pt3(x, y, z);
			imagePointVector3.push_back(thisPoint);
		}
		_rasterVecVec[i] = imagePointVector3;
	}

}

//�������ݼ��ϣ����ǻ�����������μ���
triangleSet PCLTif::getTriangleSetFromDataSet(pt3Set dataSet)
{
	//���ǻ����ݼ����е�X,Y����

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

//���������μ��ϼ����դ�񼯺ϣ���ֵ
void PCLTif::getRasterVec3SetFromTriangleSet(triangleSet theTriangleSet, int xSize, int ySize)
{
	//���ݸ��������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
	triangleSet::iterator
		iterCurTriangle = theTriangleSet.begin(),
		iterEndTrianlge = theTriangleSet.end();
	for (; iterCurTriangle != iterEndTrianlge; iterCurTriangle++)
	{
		std::vector<Pt3> theTrianglePoint3 = iterCurTriangle->second;
		this->getRasterVec3SetFromTriangle(theTrianglePoint3, xSize, ySize);
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
//������������������Ӿ��ε����꼫ֵ��4���������
std::vector<Pt2> PCLTif::_getRectanglePt2VectorFromTriangle(std::vector<Pt3> trianglePt)
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
//������άդ�񼯺�
rasterVec3Set PCLTif::getRasterVec3Set()
{
	return _rasterVec3Set;
}
// Same side method
// Determine whether point P in triangle ABC
bool PCLTif::testPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	return this-> SameSide(A, B, C, P) &&
		this->SameSide(B, C, A, P) &&
		this->SameSide(C, A, B, P);
}
// Determine whether two vectors v1 and v2 point to the same direction
// v1 = Cross(AB, AC)
// v2 = Cross(AB, AP)
bool PCLTif::SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	Vector3 AB = B - A;
	Vector3 AC = C - A;
	Vector3 AP = P - A;

	Vector3 v1 = AB.Cross(AC);
	Vector3 v2 = AB.Cross(AP);

	// v1 and v2 should point to the same direction
	return v1.Dot(v2) >= 0;
}
//�ж������Ƿ��غ�
bool PCLTif::bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3)
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
bool PCLTif::bInTheTriangle(Pt2 thePt2, std::vector<Pt3> pt3Vector, double& pointZ)
{
	bool bInTheTriangle = false;
	std::vector<Pt3>::iterator
		iterCur = pt3Vector.begin(),
		iterEnd = pt3Vector.end();
	for (; iterCur != iterEnd; iterCur++)
	{
		Pt3 thePt3 = *iterCur;
		if (this-> bTheSamePoint(thePt2, thePt3))
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
//�����������߶��Ƿ��ཻ
bool PCLTif::computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint)
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
//�����������VECTOR��XY���õ�ָ�����Zֵ
bool PCLTif::getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ)
{
	bool bIntersect = false;

	Pt2 pA = Pt2(triangleVector[0].x(), triangleVector[0].y());
	Pt2 pB = Pt2(triangleVector[1].x(), triangleVector[1].y());
	Pt2 pC = Pt2(triangleVector[2].x(), triangleVector[2].y());
	Pt2 interSectionPoint(0, 0);
	Segment segAB(pA, pB);
	Ray2 rayCP(pC, thePt);

	bool bInTri = this-> bInTheTriangle(thePt, triangleVector, pointZ);
	//�ȿ���������Ƿ������������ϣ�����ڣ����Ϸ�����
	if (bInTri)
	{
		bIntersect = true;
		return bIntersect;
	}

	bool bHasIntersectionPoint = this-> computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
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
	double intersectZ = this-> getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //�ó������X,Y,Z����
	//std::cout << "��ֵ��߶�Ϊ:" << intersectZ << std::endl;

	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = this-> getPointZfromSeg(seg3_CInterSection, thePt);
	//std::cout << "�趨��߶�Ϊ:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}
//���߶����˵�����õ������ĸ߶�
double PCLTif::getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint)
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
//���ݸ��������μ�����ڸ���������Ӿ��ε�դ�񼯺ϣ���ֵ
void PCLTif::getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3, int xSize, int ySize)
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

	int minRow = (minY - _theDetail.minY) / _yResolution;
	int maxRow = (maxY - _theDetail.minY) / _yResolution;
	int minCol = (minX - _theDetail.minX) / _xResolution;
	int maxCol = (maxX - _theDetail.minX) / _xResolution;
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
		for (int i = minCol; i <= maxCol; i++)
		{
			Pt3 thePt3 = _rasterVecVec[j][i];

			double x = thePt3.x();
			double y = thePt3.y();
			Pt2 thePt2 = Pt2(x, y);
			//�ж��Ƿ�����������
			Vector3 vecPt(x, y, 0);
			bool bTestInOrNot = this-> testPointInTriangle(vecA, vecB, vecC, vecPt);
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

//ͨ��դ�񴴽�TIF�ļ�
void PCLTif::createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, double topLeftX, double topLeftY, double rotationX, double rotationY)
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
	dGeotransform[1] = _xResolution;	//����ֱ���
	dGeotransform[2] = rotationX;	//��ת�Ƕ�
	dGeotransform[3] = topLeftY;	//��������Y
	dGeotransform[4] = rotationY;	//��ת�Ƕ�
	dGeotransform[5] = _yResolution * (-1.0);	//y�ֱ��ʣ���ֵ��

	poDS->SetGeoTransform(dGeotransform);
	//����-9999Ϊ��Чֵ


	GDALClose((GDALDatasetH)poDS);
	std::cout << "���ݼ��رգ�" << std::endl;


}
//ͨ������դ�����ݸ���TIFF�ļ�
void PCLTif::UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
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
	pBand1->SetNoDataValue(-9999);	//ȥ��-9999ֵ

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
