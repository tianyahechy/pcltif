#include "PCLTif.h"

PCLTif::PCLTif(std::string strFileName, double xResolution, double yResolution)
{
	_rasterVecVec.clear();
	_pointCloudDataSet.clear();
	_myTriangleSet.clear();

	_xResolution = xResolution;
	_yResolution = yResolution;

	//从点云获得点集
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	//读取点云数据
	std::cout << "读取点云数据:" << std::endl;
	reader.read( strFileName, *cloud);

	std::cout << "总数:" << cloud->points.size() << std::endl;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		Pt3 theVector(Pt3(x, y, z));
		_pointCloudDataSet.insert(pt3Pair(i, theVector));
	}
	//根据点云集求点云的细节
	_theDetail = this->getPCLDetailFromDataSet();
	//求X,Y方向的像素个数,
	double xSizef = abs( _theDetail.xDistance / xResolution );
	double ySizef = abs( _theDetail.yDistance / yResolution );
	//取整
	_xSize = (int)xSizef;
	_ySize = (int)ySizef;
}

PCLTif::~PCLTif()
{
	_rasterVecVec.clear();
	_pointCloudDataSet.clear();
	_myTriangleSet.clear();
}
//设置点云细节
void PCLTif::setPointCloudDetail(PCLDetail theDetail)
{
	_theDetail = theDetail;
}

//根据点云集，求点云的细节内容
PCLDetail PCLTif::getPCLDetailFromDataSet()
{
	//判断输入集合的范围
	//首先给定输入集合的X,Y坐标
	pt3Set::iterator
		iterInputExtentCur = _pointCloudDataSet.begin(),
		iterInputExtentEnd = _pointCloudDataSet.end();

	//判断XY的最小最大值，以此判定范围
	iterInputExtentCur = _pointCloudDataSet.begin();
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

	std::cout << "X的范围:(" << minX << "," << maxX << ")" << std::endl;
	std::cout << "Y的范围:(" << minY << "," << maxY << ")" << std::endl;

	//判断数据集的大小，
	double distanceX = maxX - minX;
	double distanceY = maxY - minY;
	/*
	PCLDetail theDetail;
	theDetail.minX = minX;
	theDetail.minY = minY;
	theDetail.maxX = maxX;
	theDetail.maxY = maxY;
	theDetail.xDistance = distanceX;
	theDetail.yDistance = distanceY;
	*/
	PCLDetail theDetail;
	theDetail.leftTopX = minX;
	theDetail.leftTopY = maxY;
	theDetail.xDistance = distanceX;
	theDetail.yDistance = distanceY;
	return theDetail;

}
//根据输入集合求等分插值XYZ坐标
void PCLTif::getEqualXYZVectorFromDataSet()
{
	_rasterVecVec.resize(_ySize);

	//进行输入点云坐标三维坐标的输出
	for (int i = 0; i < _ySize; i++)
	{
		std::vector<Pt3> imagePointVector3;
		imagePointVector3.clear();
		for (int j = 0; j < _xSize; j++)
		{
			double x = _theDetail.leftTopX + _xResolution * j;
			double y = _theDetail.leftTopY + _yResolution * i;
			double z = -9999;			//灰度值初值均为0
			//double z = 0;			//灰度值初值均为0
			Pt3 thisPoint = Pt3(x, y, z);
			imagePointVector3.push_back(thisPoint);
		}
		//int id = _ySize - i - 1;
		//_rasterVecVec[id] = imagePointVector3;
		_rasterVecVec[i] = imagePointVector3;
		
	}

}

//输入数据集合，三角化后输出三角形集合
void PCLTif::getTriangleSetFromDataSet()
{
	//三角化数据集合中的X,Y坐标

	mapVertexHandleAndPt3 vertexHandleAndPt3Set;
	vertexHandleAndPt3Set.clear();

	Delaunay tr;
	pt3Set::iterator iterCurVector3 = _pointCloudDataSet.begin(),
		iterEndVector3 = _pointCloudDataSet.end();
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

	//三角化后的XY，重组带Z值的三角形集合
	int faceID = 0;	//面的ID
	
	//std::cout << "输出面包含的顶点:" << std::endl;
	Delaunay::Finite_faces_iterator faceCur = tr.finite_faces_begin(),
		faceEnd = tr.finite_faces_end();
	for (; faceCur != faceEnd; faceCur++)
	{
		//根据该面的三个顶点查找Z值
		std::vector<Pt3> theVector;
		theVector.clear();
		theVector.resize(3);
		for (int i = 0; i < 3; i++)
		{
			Delaunay::Vertex_handle theVertexHandle = faceCur->vertex(i);
			Pt3 thePt3 = vertexHandleAndPt3Set[theVertexHandle];
			theVector[i] = thePt3;

		}

		_myTriangleSet.insert(trianglePair(faceID, theVector));
		faceID++;
	}
	vertexHandleAndPt3Set.clear();

}

//根据三角形集合计算出栅格集合，插值
void PCLTif::getRasterVec3SetFromTriangleSet()
{
	//根据各个三角形计算出在该三角形外接矩形的栅格集合，插值
	triangleSet::iterator
		iterCurTriangle = _myTriangleSet.begin(),
		iterEndTrianlge = _myTriangleSet.end();
	for (; iterCurTriangle != iterEndTrianlge; iterCurTriangle++)
	{
		std::vector<Pt3> theTrianglePoint3 = iterCurTriangle->second;
		this->getRasterVec3SetFromTriangle(theTrianglePoint3);
	}

}
//根据三角形坐标求外接矩形的坐标极值，4个点的坐标
std::vector<Pt2> PCLTif::_getRectanglePt2VectorFromTriangle(std::vector<Pt3> trianglePt)
{
	std::vector<Pt2> rectangleVector;
	rectangleVector.clear();
	rectangleVector.resize(4);

	//求最大最小值
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

	//给矩形赋值
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

//返回三维栅格二维数组
std::vector<std::vector<Pt3>> PCLTif::getRasterVecVec3()
{
	return _rasterVecVec;
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
//判断两点是否重合
bool PCLTif::bTheSamePoint(Pt2 firstPt2, Pt3 theComparedPt3)
{
	bool bSame = false;

	double x1 = firstPt2.x();
	double y1 = firstPt2.y();
	double x2 = theComparedPt3.x();
	double y2 = theComparedPt3.y();

	//插值，若点在某端点，则直接取值；
	double distance = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	if (distance < 0.0001)   //若交点就是A点
	{
		bSame = true;
		return bSame;
	}
	return bSame;
}
//判断该点在三角形中是否有重合点
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
//测试射线与线段是否相交
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
		//std::cout << "不是交点" << std::endl;
		bInsect = false;
		return bInsect;
	}

	bInsect = true;
	return bInsect;
}
//从三角形面的VECTOR和XY，得到指定点的Z值
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
	//先看看插入点是否在三个顶点上，如果在，果断返回真
	if (bInTri)
	{
		bIntersect = true;
		return bIntersect;
	}

	bool bHasIntersectionPoint = this-> computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
	if (!bHasIntersectionPoint)
	{
		//std::cout << "没有交点" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	//std::cout << "交点坐标：（" << interSectionPoint << ")" << std::endl;

	//两次插值，

	Pt3 vecA = triangleVector[0];
	Pt3 vecB = triangleVector[1];
	Pt3 vecC = triangleVector[2];

	Pt3 pA3(vecA.x(), vecA.y(), vecA.z());
	Pt3 pB3(vecB.x(), vecB.y(), vecB.z());
	Pt3 pC3(vecC.x(), vecC.y(), vecC.z());

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = this-> getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //得出交点的X,Y,Z坐标
	//std::cout << "插值点高度为:" << intersectZ << std::endl;

	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = this-> getPointZfromSeg(seg3_CInterSection, thePt);
	//std::cout << "设定点高度为:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}
//从线段两端的坐标得到插入点的高度
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
	//要求的高度
	double inSectPointZ = 0;

	//插值，若点在某端点，则直接取值；
	double distanceStartPoint_Insection = sqrt((startX - insectPointX) * (startX - insectPointX) + (startY - insectPointY) * (startY - insectPointY));
	double distanceEndPoint_Insection = sqrt((endX - insectPointX) * (endX - insectPointX) + (endY - insectPointY) * (endY - insectPointY));

	double deltaAP = distanceStartPoint_Insection / (distanceStartPoint_Insection + distanceEndPoint_Insection);
	inSectPointZ = startZ + deltaAP * (endZ - startZ);

	return inSectPointZ;
}
//根据各个三角形计算出在该三角形外接矩形的栅格集合，插值
void PCLTif::getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3)
{
	//1,求该三角形的外接矩形
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

	//判断点在哪个三角形上

	//2，遍历栅格集合，如果栅格里的点在此外接矩形范围内，则判断是否在三角形里，确定是否插值

	//将行缩小范围

	int minRow =  (maxY - _theDetail.leftTopY) / _yResolution;
	int maxRow = (minY - _theDetail.leftTopY) / _yResolution;
	int minCol = (minX - _theDetail.leftTopX) / _xResolution;
	int maxCol = (maxX - _theDetail.leftTopX) / _xResolution;
	if (maxRow >= _ySize)
	{
		maxRow = _ySize - 1;
	}
	if (maxCol >= _xSize)
	{
		maxCol = _xSize - 1;
	}
	if (minRow >= _ySize)
	{
		minRow = _ySize - 1;
	}
	if (minCol >= _xSize)
	{
		minCol = _xSize - 1;
	}

	for (int j = minRow; j <= maxRow; j++)
	{
		for (int i = minCol; i <= maxCol; i++)
		{
			Pt3 thePt3 = _rasterVecVec[j][i];

			double x = thePt3.x();
			double y = thePt3.y();
			Pt2 thePt2 = Pt2(x, y);
			//判断是否在三角形内
			Vector3 vecPt(x, y, 0);
			bool bTestInOrNot = this-> testPointInTriangle(vecA, vecB, vecC, vecPt);
			if (bTestInOrNot)
			{
				//在三角形内则插值
				double pointZ = -1;
				bool bInterSect = getPointZfromTriangleVector(theTrianglePt3, thePt2, pointZ);
				if (bInterSect)
				{
					//printf("插值后坐标为(%0.6f,%0.6f,%0.6f)\n", thePoint.x(), thePoint.y(), pointZ);
					thePt3 = Pt3(x, y, pointZ);

				}

			}
			_rasterVecVec[j][i] = thePt3;

		}
	}

}

//处理过程，将合并这几步
void PCLTif::process()
{
	//根据输入点集合来等分XYZ间距，得到图像坐标集合vector,并初始化灰度值为0
	this->getEqualXYZVectorFromDataSet();
	//三角化数据集合中的X,Y坐标,
	std::cout << "三角化数据集合中的X,Y坐标,重组带Z值的三角形集合" << std::endl;
	this->getTriangleSetFromDataSet();
	//根据三角形集合计算出栅格集合，插值
	this->getRasterVec3SetFromTriangleSet();
}
//得到点云细节
PCLDetail PCLTif::getDetailOfthePointCloud()
{
	return _theDetail;
}

//得到点云的X方向像素数
int PCLTif::getXSize()
{
	return _xSize;
}
//得到点云的Y方向像素数
int PCLTif::getYSize()
{
	return _ySize;
}