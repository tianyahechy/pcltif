#include <pcl/point_types.h>		//点类型定义头文件
#include <pcl/point_cloud.h>		//点云类定义头文件
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/openni2_grabber.h>	//OpenNI数据流获取类头文件
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>			//最小二乘法平滑处理类定义的头文件
#include <pcl/ModelCoefficients.h>		//采样一致性模型相关类头文件
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>	//滤波相关类头文件
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>	//基于采样一致性分割类定义的头文件
#include <pcl/surface/concave_hull.h>	//创建凹多边形类定义的头文件
#include <pcl/features/normal_3d.h>		//法向量特征估计相关类定义的头文件
#include <pcl/surface/gp3.h>			//贪婪投影三角化算法类定义的头文件
#include <pcl/io/vtk_io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Point_2.h>
#include <CGAL/Point_3.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_2_Point_2_intersection.h>
#include <CGAL/CORE_Expr.h>
#include <vector>
#include <list>
#include <map>

#include "gdal_priv.h"
#include <iostream>
#include "ogr_spatialref.h"
#include "ogr_api.h"
#include <ogrsf_frmts.h>
#include <cpl_string.h>
#include "ogr_core.h"
#include "gdal.h"

#include "theVector3.h"


typedef CGAL::Cartesian<double>								theKernel;
typedef CGAL::Delaunay_triangulation_2<theKernel>			Delaunay;
typedef CGAL::Point_2<theKernel>							Pt2;
typedef CGAL::Point_3<theKernel>							Pt3;
typedef CGAL::Segment_2<theKernel>							Segment;
typedef CGAL::Segment_3<theKernel>							Segment3;
typedef CGAL::Ray_2<theKernel>								Ray2;

/*
typedef	CGAL::Triangulation_2<theKernel>					Triangulation;
typedef Triangulation::Face_circulator						Face_circulator;
typedef Triangulation::Point								Point;

typedef Triangulation::Face_iterator faceIterator;
typedef Triangulation::Finite_faces_iterator finiteFaceIterator;
typedef Triangulation::Face Face;
typedef Triangulation::Face_handle faceHandle;

typedef Triangulation::Vertex Vertex;
typedef Triangulation::Vertex_iterator Vertex_iterator;
typedef Triangulation::Vertex_handle vertexHandle;
*/
//点云数据坐标数据集
typedef std::map<int, Pt3> pt3Set;
typedef std::pair<int, Pt3> pt3Pair;

typedef std::map<int, int> testSet;
typedef std::pair<int, int> testPair;

//三角面片对应的集合
typedef std::map<int, std::vector<Pt3> >	triangleSet;
typedef std::pair<int, std::vector<Pt3> >	trianglePair;

//栅格逐行集合（XY坐标）
typedef std::map<int, std::vector<Pt2>>		rasterVec2Set;
typedef std::pair<int, std::vector<Pt2>>	rasterVec2Pair;

//栅格逐行集合（XYZ坐标）
typedef std::map<int, std::vector<Pt3>>		rasterVec3Set;
typedef std::pair<int, std::vector<Pt3>>	rasterVec3Pair;

//点云细节
struct PCLDetail
{
	double minX;	//点云的四至
	double minY;
	double maxX;
	double maxY;
	double xDistance;//点云的X距离
	double yDistance;//点云的Y距离
};
//指定点XY集合得出

//测试线段和指定点的朝向，(左旋，右旋还是共面)
int testOrientationBetweenPointInSegment(Pt2 thePoint, Pt2 startPoint, Pt2 endPoint)
{
	int result = -100;
	switch (CGAL::orientation(startPoint, thePoint, endPoint))
	{
	case  CGAL::COLLINEAR:
		//共线时，这里要考虑到测试点有没有在该线段的延长线上，在延长线上，则不能认为其在三角形里
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

// 测试坐标是否在三角形内
bool testRange(Pt2 thePoint, std::vector<Pt2> pt)
{
	bool bRanged = false;

	double maxX = pt[0].x();
	double maxY = pt[0].y();
	double minX = pt[0].x();
	double minY = pt[0].y();

	//求出三角形的最大XY值
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

	//判断该点是否在范围内
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
bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	Vector3 AB = B - A;
	Vector3 AC = C - A;
	Vector3 AP = P - A;

	Vector3 v1 = AB.Cross(AC);
	Vector3 v2 = AB.Cross(AP);

	// v1 and v2 should point to the same direction
	return v1.Dot(v2) >= 0;
}

// Same side method
// Determine whether point P in triangle ABC
bool testPointInRectangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
	return SameSide(A, B, C, P) &&
		SameSide(B, C, A, P) &&
		SameSide(C, A, B, P);
}

//测试射线与线段是否相交
bool computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint)
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

//从线段两端的坐标得到插入点的高度
double getPointZfromSeg(Segment3 theSeg, Pt2 inSectPoint)
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
	if (distanceStartPoint_Insection < 0.0001)   //若交点就是A点
	{
		inSectPointZ = startZ;
	}
	else
		if (distanceEndPoint_Insection < 0.0001)   //若交点就是B点
		{
			inSectPointZ = endZ;
		}
		else                               //若交点不在A或B，则按比例线性插值
		{
			double deltaAP = distanceStartPoint_Insection / (distanceStartPoint_Insection + distanceEndPoint_Insection);
			inSectPointZ = startZ + deltaAP * (endZ - startZ);
		}

	return inSectPointZ;
}

//从三角形面的VECTOR和XY，得到指定点的Z值
bool getPointZfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ)
{
	bool bIntersect = false;

	Pt2 pA = Pt2(triangleVector[0].x(), triangleVector[0].y());
	Pt2 pB = Pt2(triangleVector[1].x(), triangleVector[1].y());
	Pt2 pC = Pt2(triangleVector[2].x(), triangleVector[2].y());
	Pt2 interSectionPoint(0, 0);
	Segment segAB(pA, pB);
	Ray2 rayCP(pC, thePt);

	bool bHasIntersectionPoint = computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
	if (!bHasIntersectionPoint)
	{
		//std::cout << "没有交点" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	//std::cout << "交点坐标：（" << interSectionPoint << ")" << std::endl;

	//两次插值，若点在某端点，则直接取值

	Pt3 vecA = triangleVector[0];
	Pt3 vecB = triangleVector[1];
	Pt3 vecC = triangleVector[2];

	Pt3 pA3(vecA.x(), vecA.y(), vecA.z());
	Pt3 pB3(vecB.x(), vecB.y(), vecB.z());
	Pt3 pC3(vecC.x(), vecC.y(), vecC.z());

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //得出交点的X,Y,Z坐标
	//std::cout << "插值点高度为:" << intersectZ << std::endl;

	//指定点若在某端点，则直接取值；
	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = getPointZfromSeg(seg3_CInterSection, thePt);
	Pt3 thePt3(thePt.x(), thePt.y(), pointZ);
	//std::cout << "设定点高度为:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}

// 从三角形面的VECTOR和XY，得到指定点的Z值
bool getPointZAndTraingleVectorfromTriangleVector(std::vector<Pt3> triangleVector, Pt2 thePt, double& pointZ)
{
	bool bIntersect = false;

	Pt2 pA = Pt2(triangleVector[0].x(), triangleVector[0].y());
	Pt2 pB = Pt2(triangleVector[1].x(), triangleVector[1].y());
	Pt2 pC = Pt2(triangleVector[2].x(), triangleVector[2].y());
	Pt2 interSectionPoint(0, 0);
	Segment segAB(pA, pB);
	Ray2 rayCP(pC, thePt);

	bool bHasIntersectionPoint = computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
	if (!bHasIntersectionPoint)
	{
		//std::cout << "没有交点" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	//std::cout << "交点坐标：（" << interSectionPoint << ")" << std::endl;

	//两次插值，若点在某端点，则直接取值

	Pt3 vecA = triangleVector[0];
	Pt3 vecB = triangleVector[1];
	Pt3 vecC = triangleVector[2];

	Pt3 pA3(vecA.x(), vecA.y(), vecA.z());
	Pt3 pB3(vecB.x(), vecB.y(), vecB.z());
	Pt3 pC3(vecC.x(), vecC.y(), vecC.z());

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //得出交点的X,Y,Z坐标
	//std::cout << "插值点高度为:" << intersectZ << std::endl;

	//指定点若在某端点，则直接取值；
	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = getPointZfromSeg(seg3_CInterSection, thePt);
	Pt3 thePt3(thePt.x(), thePt.y(), pointZ);
	//std::cout << "设定点高度为:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}
//根据指定点的X,Y和集合，判断在集合的哪个Id的三角形上
int getFaceIDfromthePointAndTriangleSet(Pt2 thePt, triangleSet theTriangleSet)
{
	int theFaceID = -1;

	Vector3 vecA(0, 0, 0);
	Vector3 vecB(0, 0, 0);
	Vector3 vecC(0, 0, 0);
	Vector3 vecPt(thePt.x(), thePt.y(), 0);
	//判断点在哪个三角形上
	triangleSet::iterator
		iterTriangleCur = theTriangleSet.begin(),
		iterTriangleEnd = theTriangleSet.end();
	for (; iterTriangleCur != iterTriangleEnd; iterTriangleCur++)
	{
		int faceID = iterTriangleCur->first;
		std::vector<Pt3> thevec = iterTriangleCur->second;

		vecA.x = thevec[0].x();
		vecA.y = thevec[0].y();
		vecB.x = thevec[1].x();
		vecB.y = thevec[1].y();
		vecC.x = thevec[2].x();
		vecC.y = thevec[2].y();

		Pt2 p0 = Pt2(thevec[0].x(), thevec[0].y());
		Pt2 p1 = Pt2(thevec[1].x(), thevec[1].y());
		Pt2 p2 = Pt2(thevec[2].x(), thevec[2].y());

		bool bTestInOrNot = testPointInRectangle(vecA, vecB, vecC, vecPt);
		if (bTestInOrNot)
		{
			std::cout << "点(" << thePt << ")在第" << faceID << "个三角形内" << std::endl;
			break;
		}
	}

	//获得面的ID
	if (iterTriangleCur == iterTriangleEnd)
	{
		//std::cout << "点" << thePt << "没有在三角形内" << std::endl;
		return theFaceID;
	}

	theFaceID = iterTriangleCur->first;

	return theFaceID;
}

//输入数据集合，三角化后输出三角形集合
triangleSet getTriangleSetFromDataSet(pt3Set dataSet)
{
	//三角化数据集合中的X,Y坐标
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
			double x = faceCur->vertex(i)->point().hx();
			double y = faceCur->vertex(i)->point().hy();
			double z = 0;
			//根据XY查找Z值
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

			//std::cout << "三角化后各个三角面的坐标,面" << faceID << ",顶点:" << theVector[i] << std::endl;
		}

		//求该三角形的中心点，以划分范围
		//double centerX = (theVector[0].x() + theVector[1].x() + theVector[2].x()) / 3.0;
		//double centerY = (theVector[0].y() + theVector[1].y() + theVector[2].y()) / 3.0;
		//int triangleZone = testWhichTraingle(centerX, centerY, numberofEachColumn, minX, minY, maxX, maxY);
		myTriangleSet.insert(trianglePair(faceID, theVector));

		faceID++;

	}

	return myTriangleSet;
}
//从数据集合和三角面集合中得出插入点的高度和插入的三角形点
bool getPt3InsertAndPt3VecInOneTriangleFromDataSet(pt3Set dataSet, Pt2 thePoint, Pt3& outPutPoint3d, triangleSet myTriangleSet, std::vector<Pt3>& trianglePointVector)
{
	bool bPointInPointCloud = false;

	//判断点在哪个三角形上
	int selectedQuadID = getFaceIDfromthePointAndTriangleSet(thePoint, myTriangleSet);
	//根据ID查找到三角形三顶点坐标
	triangleSet::iterator iterTriangleCur = myTriangleSet.find(selectedQuadID),
		iterTriangleEnd = myTriangleSet.end();
	if (iterTriangleCur == iterTriangleEnd)
	{
		std::cout << "点(" << thePoint << ")没有在三角形内" << std::endl;
		bPointInPointCloud = false;
		return bPointInPointCloud;
	}

	//std::cout << "选中的三角面ID:" << selectedQuadID << " 三角形坐标为:" << std::endl;
	trianglePointVector = iterTriangleCur->second;
	printf("(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f)\n",
		trianglePointVector[0].x(), trianglePointVector[0].y(), trianglePointVector[0].z(),
		trianglePointVector[1].x(), trianglePointVector[1].y(), trianglePointVector[1].z(),
		trianglePointVector[2].x(), trianglePointVector[2].y(), trianglePointVector[2].z());

	double pointZ = -1;
	bool bInterSect = getPointZfromTriangleVector(trianglePointVector, thePoint, pointZ);
	if (bInterSect)
	{
		printf("插值后坐标为(%0.6f,%0.6f,%0.6f)\n", thePoint.x(), thePoint.y(), pointZ);
		outPutPoint3d = Pt3(thePoint.x(), thePoint.y(), pointZ);
		bPointInPointCloud = true;
		return bPointInPointCloud;
	}

	return bPointInPointCloud;
}

//从点云数据集合和坐标XYvector和三角形集合中,得到栅格的三维集合
rasterVec3Set getPt3SetFromDataSetAndPt2Vector(pt3Set dataSet, rasterVec2Set Pt2Set, triangleSet theTriangleSet)
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
			bool bPtInCloud = getPt3InsertAndPt3VecInOneTriangleFromDataSet(dataSet, thePoint, pointXYZ, theTriangleSet,triangleVec);
			if (bPtInCloud)
			{
				theVec3.push_back(pointXYZ);
			}
			else
			{
				//没在点云内就赋值为0
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

//根据点云集，求点云的细节内容
PCLDetail getPCLDetail(pt3Set dataSet)
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
	double distanceX = abs(maxX - minX);
	double distanceY = abs(maxY - minY);

	PCLDetail theDetail;
	theDetail.minX = minX;
	theDetail.minY = minY;
	theDetail.maxX = maxX;
	theDetail.maxY = maxY;
	theDetail.xDistance = distanceX;
	theDetail.yDistance = distanceY;

	return theDetail;

}
//根据输入集合求等分插值XY坐标
rasterVec2Set getEqualXYVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution )
{
	//进行输入点云坐标二维坐标的输出
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
			//std::cout << "从点云输入，输出二维坐标:(" << i << "," << j << ")=" << thisPoint << std::endl;
		}
		myRaster.insert(rasterVec2Pair(i, imagePointVector2));
	}

	return myRaster;
}
//通过栅格创建TIF文件
void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, int xResolution, int yResolution, double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0)
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
	dGeotransform[1] = xResolution;	//横向分辨率
	dGeotransform[2] = rotationX;	//旋转角度
	dGeotransform[3] = topLeftY;	//左上坐标Y
	dGeotransform[4] = rotationY;	//旋转角度
	dGeotransform[5] = yResolution * (-1.0);	//y分辨率（负值）

	poDS->SetGeoTransform(dGeotransform);

	GDALClose((GDALDatasetH)poDS);
	std::cout << "数据集关闭！" << std::endl;


}
//通过更改栅格数据更新TIFF文件
void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
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
	//循环图像高，更新图像里面的像素值
	rasterVec3Set::iterator
		iterRasterCur = theRasterSet.begin(),
		iterRasterEnd = theRasterSet.end();
	for (; iterRasterCur != iterRasterEnd; iterRasterCur++)
	{
		int id = iterRasterCur->first;
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

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	//读取点云数据
	std::cout << "读取点云数据:" << std::endl;
	reader.read("d:\\1_xyz.pcd", *cloud);
	pt3Set vector3Set;
	vector3Set.clear();
	//for (size_t i = 0; i < cloud->points.size(); i++)
	for (size_t i = 0; i < 20000; i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		Pt3 theVector(Pt3(x, y, z));
		vector3Set.insert(pt3Pair(i, theVector));
		//std::cout << "第" << i << "个数据：(" <<theVector.x() << "," << theVector.y() << "," << theVector.z() << ")" << std::endl;
	}
	
	//根据点云集求点云的细节
	PCLDetail theDetail = getPCLDetail(vector3Set);
	
	//设置X，Y方向的像素分辨率
	double xResolution = 1.0;
	double yResolution = 1.0;

	//求X,Y方向的像素个数,
	double distanceX = theDetail.xDistance;
	double distanceY = theDetail.yDistance;
	double xSizef = distanceX / xResolution;
	double ySizef = distanceY / yResolution;
	//取整
	int xSize = (int)xSizef;
	int ySize = (int)ySizef;

	//设置左上角为minx,maxY
	double minX = theDetail.minX;
	double minY = theDetail.minY;
	
	//根据输入点集合来等分XY间距，得到图像坐标集合vector
	rasterVec2Set imagePointVector2Set = getEqualXYVectorFromDataSet(vector3Set, minX, minY, xSize, ySize,xResolution,yResolution);
	
	//三角化数据集合中的X,Y坐标,重组带Z值的三角形集合
	std::cout << "三角化数据集合中的X,Y坐标,重组带Z值的三角形集合" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);
	rasterVec3Set imageSet = getPt3SetFromDataSetAndPt2Vector(vector3Set, imagePointVector2Set,myTriangleSet);
	
	//创建网格文件
	const char * pszRasterFile = "E:\\PCLOutPut_20000C.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//设定图像左上角
	float topLeftX = minX;
	float topLeftY = maxY;
	createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//更新网格文件,生成.tiff文件
	UpdateRasterFile(pszRasterFile, imageSet);
	
	
	/*
	double testZ = -1;
	//得到三角面集合。提取出来，
	//三角化数据集合中的X,Y坐标,重组带Z值的三角形集合
	std::cout << "三角化数据集合中的X,Y坐标,重组带Z值的三角形集合" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);

	//得到图像点的坐标XYZ
	std::cout << "得到图像点的坐标XYZ" << std::endl;

	//输出到文件
	FILE * file = fopen("e:\\outputTriangleAndPoint", "w");

	for (int x = 188; x < 192; x++)
	{
		for (int y = 54; y < 58; y++ )
		{
			double theX = minX + xResolution * x;
			double theY = minY + yResolution * y;
			Pt2 thisPoint = Pt2(theX, theY);
			Pt3 thept3(0, 0, 0);
			std::vector<Pt3> pt3Vec;
			pt3Vec.clear();
			getPt3InsertAndPt3VecInOneTriangleFromDataSet(vector3Set, thisPoint, thept3, myTriangleSet, pt3Vec);
			std::cout << "大小为：" << pt3Vec.size() << std::endl;
	
			fprintf( file, "(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f)\n",
				pt3Vec[0].x(), pt3Vec[0].y(), pt3Vec[0].z(),
				pt3Vec[1].x(), pt3Vec[1].y(), pt3Vec[1].z(),
				pt3Vec[2].x(), pt3Vec[2].y(), pt3Vec[2].z(),
				thept3.x(), thept3.y(), thept3.z());
		}
	}
	fclose(file);
	*/
	
	return 0;
}