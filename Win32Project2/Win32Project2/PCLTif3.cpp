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
//设置点云细节
void PCLTif::setPointCloudDetail(PCLDetail theDetail)
{
	_theDetail = theDetail;
}

//根据点云集，求点云的细节内容
PCLDetail PCLTif::getPCLDetail(pt3Set dataSet)
{
	//判断输入集合的范围
	//首先给定输入集合的X,Y坐标
	pt3Set::iterator
		iterInputExtentCur = dataSet.begin(),
		iterInputExtentEnd = dataSet.end();

	//判断XY的最小最大值，以此判定范围
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

	std::cout << "X的范围:(" << minX << "," << maxX << ")" << std::endl;
	std::cout << "Y的范围:(" << minY << "," << maxY << ")" << std::endl;

	//判断数据集的大小，
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
//根据输入集合求等分插值XYZ坐标
void PCLTif::getEqualXYZVectorFromDataSet(pt3Set dataSet, int xSize, int ySize )
{
	_rasterVecVec.resize(ySize);

	//进行输入点云坐标三维坐标的输出
	for (int i = 0; i < ySize; i++)
	{
		std::vector<Pt3> imagePointVector3;
		imagePointVector3.clear();
		for (int j = 0; j < xSize; j++)
		{
			double x = _theDetail.minX + _xResolution * j;
			double y = _theDetail.minY + _yResolution * i;
			double z = -9999;			//灰度值初值均为0
			Pt3 thisPoint = Pt3(x, y, z);
			imagePointVector3.push_back(thisPoint);
		}
		_rasterVecVec[i] = imagePointVector3;
	}

}

//输入数据集合，三角化后输出三角形集合
triangleSet PCLTif::getTriangleSetFromDataSet(pt3Set dataSet)
{
	//三角化数据集合中的X,Y坐标

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

	//三角化后的XY，重组带Z值的三角形集合
	int faceID = 0;	//面的ID
	triangleSet myTriangleSet;
	myTriangleSet.clear();

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

		myTriangleSet.insert(trianglePair(faceID, theVector));
		faceID++;
	}

	return myTriangleSet;
}

//根据三角形集合计算出栅格集合，插值
void PCLTif::getRasterVec3SetFromTriangleSet(triangleSet theTriangleSet, int xSize, int ySize)
{
	//根据各个三角形计算出在该三角形外接矩形的栅格集合，插值
	triangleSet::iterator
		iterCurTriangle = theTriangleSet.begin(),
		iterEndTrianlge = theTriangleSet.end();
	for (; iterCurTriangle != iterEndTrianlge; iterCurTriangle++)
	{
		std::vector<Pt3> theTrianglePoint3 = iterCurTriangle->second;
		this->getRasterVec3SetFromTriangle(theTrianglePoint3, xSize, ySize);
	}

	//填充栅格集合
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
//返回三维栅格集合
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
void PCLTif::getRasterVec3SetFromTriangle(std::vector<Pt3> theTrianglePt3, int xSize, int ySize)
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

//通过栅格创建TIF文件
void PCLTif::createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, double topLeftX, double topLeftY, double rotationX, double rotationY)
{
	//设定支持中文路径
	CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
	//注册栅格驱动
	GDALAllRegister();
	//下面获取指定格式的驱动，用于创建图像
	const char * pszFormat = "GTiff";
	GDALDriver * poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if (poDriver == NULL)
	{
		std::cout << "格式" << pszFormat << "不支持create()方法" << std::endl;
	}
	else
	{
		std::cout << "驱动OK！" << std::endl;
	}

	//获取该驱动的一些元数据信息
	char ** papszMetadata = poDriver->GetMetadata();
	if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
	{
		std::cout << pszFormat << "支持Create()方法" << std::endl;
	}
	if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
	{
		std::cout << pszFormat << "支持CreateCopy()方法" << std::endl;
	}

	//创建输出文件，大小为512*512*3，三个波段8bit的数据
	GDALDataset * poDS = poDriver->Create(strImageName, xSize, ySize, bandSize, GDT_Float32, NULL);
	if (poDS == NULL)
	{
		std::cout << "创建图像" << strImageName << "失败" << std::endl;
		return;
	}
	//设置六参数，第一个和第四个是图像左上角的坐标，第2个和第6个为横向和纵向分辨率，余下两个为旋转角度
	double dGeotransform[6] = { 0, 0, 0, 0, 0, 0 };
	dGeotransform[0] = topLeftX;   //左上坐标X
	dGeotransform[1] = _xResolution;	//横向分辨率
	dGeotransform[2] = rotationX;	//旋转角度
	dGeotransform[3] = topLeftY;	//左上坐标Y
	dGeotransform[4] = rotationY;	//旋转角度
	dGeotransform[5] = _yResolution * (-1.0);	//y分辨率（负值）

	poDS->SetGeoTransform(dGeotransform);
	//设置-9999为无效值


	GDALClose((GDALDatasetH)poDS);
	std::cout << "数据集关闭！" << std::endl;


}
//通过更改栅格数据更新TIFF文件
void PCLTif::UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
{
	//设定支持中文路径
	CPLSetConfigOption("GDAL_FILENAME_IS_UT8", "NO");
	//注册栅格驱动
	GDALAllRegister();
	//打开要更新的数据，注意第二个参数使用GA_UPDATE
	GDALDataset * poDS = (GDALDataset*)GDALOpen(strImageName, GA_Update);
	if (poDS == NULL)
	{
		std::cout << "打开图像" << strImageName << "失败" << std::endl;
	}

	//获取图像大小
	int iWidth = poDS->GetRasterXSize();
	int iHeight = poDS->GetRasterYSize();

	//声明一行图像大小的空间
	float * pBuf = new float[iWidth];
	memset(pBuf, 0, sizeof(float) * iWidth);
	//声明一个像素值的Vector,用以给TIF文件每行赋值
	std::vector<float> imagePerLineVector;
	imagePerLineVector.clear();

	//获取第一波段
	GDALRasterBand * pBand1 = poDS->GetRasterBand(1);
	pBand1->SetNoDataValue(-9999);	//去除-9999值

	//循环图像高，更新图像里面的像素值
	rasterVec3Set::iterator
		iterRasterCur = theRasterSet.begin(),
		iterRasterEnd = theRasterSet.end();
	for (; iterRasterCur != iterRasterEnd; iterRasterCur++)
	{
		//上下颠倒下
		int id = iHeight - 1 - iterRasterCur->first;
		std::vector<Pt3> theLine = iterRasterCur->second;

		//清空两段内存
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
			//该行逐像素赋灰度值给VECTOR
			imagePerLineVector.push_back(theValue);
		}

		//将该行像素值给内存
		memcpy(pBuf, (float*)&imagePerLineVector[0], sizeof(float) * iWidth);
		//memset(pBuf, 255, sizeof(float) * iWidth);
		//将像元值写入图像
		//pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, pBuf, iWidth, 1, GDT_Byte, 0, 0);
		pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, pBuf, iWidth, 1, GDT_Float32, 0, 0);
	}
	//清空两段内存
	imagePerLineVector.clear();
	memset(pBuf, 0, sizeof(float) * iWidth);

	GDALClose((GDALDatasetH)poDS);
}
