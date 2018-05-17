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
#include <CGAL/Triangulation_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Point_3.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_2_Point_2_intersection.h>
#include <vector>
#include <list>
#include <map>

typedef CGAL::Cartesian<double>								theKernel;
typedef	CGAL::Triangulation_2<theKernel>					Triangulation;
typedef CGAL::Point_2<theKernel>							Pt2;
typedef CGAL::Point_3<theKernel>							Pt3;
typedef CGAL::Segment_2<theKernel>							Segment;
typedef CGAL::Segment_3<theKernel>							Segment3;
typedef CGAL::Ray_2<theKernel>								Ray2;

typedef Triangulation::Face_circulator						Face_circulator;
typedef Triangulation::Point								Point;

typedef Triangulation::Face_iterator faceIterator;
typedef Triangulation::Face Face;
typedef Triangulation::Face_handle faceHandle;

typedef Triangulation::Vertex Vertex;
typedef Triangulation::Vertex_iterator Vertex_iterator;
typedef Triangulation::Vertex_handle vertexHandle;

struct  myVector3
{
	Pt2 theXY;
	double z;
};

//输入点集
typedef std::map<int, myVector3> myVector3Set;
typedef std::pair<int, myVector3> myVector3Pair;
//三角面片对应的集合
typedef std::map<int, std::vector<myVector3> >	triangleSet;
typedef std::pair<int, std::vector<myVector3> >	trianglePair;

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

//测试指定点是否在三角形内
bool testPointInRectangle(Pt2 thePoint, Pt2 pt1, Pt2 pt2, Pt2 pt3)
{
	bool bTest = false;
	//测试范围和朝向，两者均过关才行
	bool bTestRange = false;
	bool bTestOrientation = false;

	//通过坐标判断是否在范围内
	std::vector<Pt2> pt2Vector;
	pt2Vector.clear();
	pt2Vector.push_back(pt1);
	pt2Vector.push_back(pt2);
	pt2Vector.push_back(pt3);

	bTestRange = testRange(thePoint, pt2Vector);
	if (!bTestRange)
	{
		bTest = false;

		return bTest;
	}

	//通过朝向判断是否在范围内

	int leftTurnCount = -1;
	int rightTurnCount = -1;
	int OnLineCount = -1;

	int firstTest = testOrientationBetweenPointInSegment(thePoint, pt1, pt2);
	int secondTest = testOrientationBetweenPointInSegment(thePoint, pt2, pt3);
	int thirdTest = testOrientationBetweenPointInSegment(thePoint, pt3, pt1);

	//如果有一个在线上，就说明点在三角形内
	if (firstTest == 0 || secondTest == 0 || thirdTest == 0)
	{
		bTestOrientation = true;
	}
	//不在线上，
	//如果有三个均相等，则同向在内
	if ((firstTest == secondTest) &&
		(firstTest == thirdTest))
	{
		bTestOrientation = true;
	}
	if (bTestRange && bTestOrientation)
	{
		bTest = true;
	}
	return bTest;
}

//测试射线与线段是否相交
bool computeIntersectionPointBetweenSegAndRay(Segment theSeg, Ray2 theRay, Pt2& intersectionPoint)
{
	bool bInsect = false;
	if (!CGAL::do_intersect(theSeg, theRay))
	{
		std::cout << "do not intersect" << std::endl;
		bInsect = false;
		return bInsect;
	}

	CGAL::Object result = CGAL::intersection(theSeg, theRay);
	if (!CGAL::assign(intersectionPoint, result))
	{
		std::cout << "不是交点" << std::endl;
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
bool getPointZfromTriangleVector(std::vector<myVector3> triangleVector, Pt2 thePt, double& pointZ)
{
	bool bIntersect = false;

	Pt2 pA = triangleVector[0].theXY;
	Pt2 pB = triangleVector[1].theXY;
	Pt2 pC = triangleVector[2].theXY;
	Pt2 interSectionPoint(0, 0);
	Segment segAB(pA, pB);
	Ray2 rayCP(pC, thePt);

	bool bHasIntersectionPoint = computeIntersectionPointBetweenSegAndRay(segAB, rayCP, interSectionPoint);
	if (!bHasIntersectionPoint)
	{
		std::cout << "没有交点" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	std::cout << "交点坐标：（" << interSectionPoint << ")" << std::endl;

	//两次插值，若点在某端点，则直接取值

	myVector3 vecA = triangleVector[0];
	myVector3 vecB = triangleVector[1];
	myVector3 vecC = triangleVector[2];

	Pt3 pA3(vecA.theXY.x(), vecA.theXY.y(), vecA.z);
	Pt3 pB3(vecB.theXY.x(), vecB.theXY.y(), vecB.z);
	Pt3 pC3(vecC.theXY.x(), vecC.theXY.y(), vecC.z);

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //得出交点的X,Y,Z坐标
	std::cout << "插值点高度为:" << intersectZ << std::endl;

	//指定点若在某端点，则直接取值；
	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = getPointZfromSeg(seg3_CInterSection, thePt);
	Pt3 thePt3(thePt.x(), thePt.y(), pointZ);
	std::cout << "设定点高度为:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}

//根据指定点的X,Y和集合，判断在集合的哪个Id的三角形上
int getFaceIDfromthePointAndTriangleSet(Pt2 thePt, triangleSet theTriangleSet)
{
	int theFaceID = -1;

	//判断点在哪个三角形上
	triangleSet::iterator
		iterTriangleCur = theTriangleSet.begin(),
		iterTriangleEnd = theTriangleSet.end();
	for (; iterTriangleCur != iterTriangleEnd; iterTriangleCur++)
	{
		int faceID = iterTriangleCur->first;
		std::vector<myVector3> thevec = iterTriangleCur->second;

		Pt2 p0 = thevec[0].theXY;
		Pt2 p1 = thevec[1].theXY;
		Pt2 p2 = thevec[2].theXY;

		bool bTestInOrNot = testPointInRectangle(thePt, p0, p1, p2);
		if (bTestInOrNot)
		{
			std::cout << "点(" << thePt << ")在第" << faceID << "个三角形内" << std::endl;
			break;
		}
	}

	//获得面的ID
	if (iterTriangleCur == iterTriangleEnd)
	{
		std::cout << "指定点没有在三角化后的三角形集合范围内" << std::endl;
		return theFaceID;
	}

	theFaceID = iterTriangleCur->first;

	return theFaceID;
}

//输入数据集合，三角化后输出三角形集合
triangleSet getTriangleSetFromDataSet(myVector3Set dataSet)
{
	//三角化数据集合中的X,Y坐标
	Triangulation tr;
	myVector3Set::iterator iterCurVector3 = dataSet.begin(),
		iterEndVector3 = dataSet.end();
	for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
	{
		Pt2 xy = iterCurVector3->second.theXY;
		tr.insert(xy);
	}

	//三角化后的XY，重组带Z值的三角形集合
	int faceID = 0;	//面的ID
	triangleSet myTriangleSet;
	myTriangleSet.clear();

	std::cout << "输出面包含的顶点:" << std::endl;
	faceIterator faceCur = tr.faces_begin(),
		faceEnd = tr.faces_end();
	for (; faceCur != faceEnd; faceCur++)
	{
		//根据该面的三个顶点查找Z值
		std::vector<myVector3> theVector;
		theVector.clear();
		theVector.resize(3);
		for (int i = 0; i < 3; i++)
		{
			double x = tr.triangle(faceCur).vertex(i).x();
			double y = tr.triangle(faceCur).vertex(i).y();
			double z = 0;
			//根据XY查找Z值
			myVector3Set::iterator iterCurVector3 = dataSet.begin(),
				iterEndVector3 = dataSet.end();
			for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
			{
				double xRef = iterCurVector3->second.theXY.x();
				double yRef = iterCurVector3->second.theXY.y();
				double deltaDistance = sqrt((x - xRef) * (x - xRef) + (y - yRef) * (y - yRef));
				if (deltaDistance < 0.001)
				{
					z = iterCurVector3->second.z;
				}
			}

			myVector3 vec;
			vec.theXY = tr.triangle(faceCur).vertex(i);
			vec.z = z;
			theVector[i] = vec;
		}

		myTriangleSet.insert(trianglePair(faceID, theVector));

		faceID++;

	}

	return myTriangleSet;
}
//从数据集合中得出指定点的高度
bool getPointZFromDataSet(myVector3Set dataSet, Pt2 thePoint, double& pointZ)
{
	bool bPointInPointCloud = false;

	//三角化数据集合中的X,Y坐标,重组带Z值的三角形集合
	triangleSet myTriangleSet = getTriangleSetFromDataSet(dataSet);
	//判断点在哪个三角形上
	int selectedQuadID = getFaceIDfromthePointAndTriangleSet(thePoint, myTriangleSet);

	//根据ID查找到三角形三顶点坐标
	triangleSet::iterator iterTriangleCur = myTriangleSet.find(selectedQuadID),
		iterTriangleEnd = myTriangleSet.end();
	if (iterTriangleCur == iterTriangleEnd)
	{
		std::cout << "该点没在三角形集合内" << std::endl;
		bPointInPointCloud = false;
		return bPointInPointCloud;
	}

	std::cout << "选中的三角面ID:" << selectedQuadID << " 坐标为:" << std::endl;
	std::vector<myVector3> thevec = iterTriangleCur->second;

	bool bInterSect = getPointZfromTriangleVector(thevec, thePoint, pointZ);
	if (bInterSect)
	{
		std::cout << "指定点高度为:" << pointZ << std::endl;
		bPointInPointCloud = true;
		return bPointInPointCloud;
	}

	return bPointInPointCloud;
}
//从数据集合中得出指定点的坐标XYZ
bool getPointXYZFromDataSet(myVector3Set dataSet, Pt2 thePoint, Pt3& pointXYZ)
{

	double pointZ = 0;
	bool bPointInCloud = getPointZFromDataSet(dataSet, thePoint, pointZ);
	if (bPointInCloud)
	{
		pointXYZ = Pt3(thePoint.x(), thePoint.y(), pointZ);
	}
	return bPointInCloud;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	//读取点云数据
	reader.read("E:\\VS2013\\CSite1origc.pcd", *cloud);
	
	myVector3Set vector3Set;
	vector3Set.clear();
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		myVector3 theVector;
		theVector.theXY = Pt2(x, y);
		theVector.z = z;
		vector3Set.insert(myVector3Pair(i, theVector));
		std::cout << "第" << i << "个数据：(" <<theVector.theXY.x() << "," << theVector.theXY.y() << "," << theVector.z << ")" << std::endl;
	}

	//设定测试点
	std::vector<Pt2> testPointVector;
	testPointVector.clear();
	Pt2 testPoint = Pt2(5.0, 0);
	testPointVector.push_back(testPoint);
	std::vector<Pt2>::iterator
		iterPt2Cur = testPointVector.begin(),
		iterPt2End = testPointVector.end();
	for (; iterPt2Cur != iterPt2End; iterPt2Cur++ )
	{
		Pt3 pointXYZ(0, 0, 0);
		bool bPtInCloud = getPointXYZFromDataSet(vector3Set, *iterPt2Cur, pointXYZ);
		if (bPtInCloud)
		{
			std::cout << "指定点的坐标是:" << pointXYZ << std::endl;
		}
	}

	//判断输入数据集合的范围XY


	return 0;
}