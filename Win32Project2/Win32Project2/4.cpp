#include <pcl/point_types.h>		//�����Ͷ���ͷ�ļ�
#include <pcl/point_cloud.h>		//�����ඨ��ͷ�ļ�
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/openni2_grabber.h>	//OpenNI��������ȡ��ͷ�ļ�
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>			//��С���˷�ƽ�������ඨ���ͷ�ļ�
#include <pcl/ModelCoefficients.h>		//����һ����ģ�������ͷ�ļ�
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>	//�˲������ͷ�ļ�
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>	//���ڲ���һ���Էָ��ඨ���ͷ�ļ�
#include <pcl/surface/concave_hull.h>	//������������ඨ���ͷ�ļ�
#include <pcl/features/normal_3d.h>		//������������������ඨ���ͷ�ļ�
#include <pcl/surface/gp3.h>			//̰��ͶӰ���ǻ��㷨�ඨ���ͷ�ļ�
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

//����㼯
typedef std::map<int, myVector3> myVector3Set;
typedef std::pair<int, myVector3> myVector3Pair;
//������Ƭ��Ӧ�ļ���
typedef std::map<int, std::vector<myVector3> >	triangleSet;
typedef std::pair<int, std::vector<myVector3> >	trianglePair;

//�����߶κ�ָ����ĳ���(�������������ǹ���)
int testOrientationBetweenPointInSegment(Pt2 thePoint, Pt2 startPoint, Pt2 endPoint)
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
bool testRange(Pt2 thePoint, std::vector<Pt2> pt)
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

//����ָ�����Ƿ�����������
bool testPointInRectangle(Pt2 thePoint, Pt2 pt1, Pt2 pt2, Pt2 pt3)
{
	bool bTest = false;
	//���Է�Χ�ͳ������߾����ز���
	bool bTestRange = false;
	bool bTestOrientation = false;

	//ͨ�������ж��Ƿ��ڷ�Χ��
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

	//ͨ�������ж��Ƿ��ڷ�Χ��

	int leftTurnCount = -1;
	int rightTurnCount = -1;
	int OnLineCount = -1;

	int firstTest = testOrientationBetweenPointInSegment(thePoint, pt1, pt2);
	int secondTest = testOrientationBetweenPointInSegment(thePoint, pt2, pt3);
	int thirdTest = testOrientationBetweenPointInSegment(thePoint, pt3, pt1);

	//�����һ�������ϣ���˵��������������
	if (firstTest == 0 || secondTest == 0 || thirdTest == 0)
	{
		bTestOrientation = true;
	}
	//�������ϣ�
	//�������������ȣ���ͬ������
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

//�����������߶��Ƿ��ཻ
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
		std::cout << "���ǽ���" << std::endl;
		bInsect = false;
		return bInsect;
	}

	bInsect = true;
	return bInsect;
}

//���߶����˵�����õ������ĸ߶�
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
	//Ҫ��ĸ߶�
	double inSectPointZ = 0;

	//��ֵ��������ĳ�˵㣬��ֱ��ȡֵ��
	double distanceStartPoint_Insection = sqrt((startX - insectPointX) * (startX - insectPointX) + (startY - insectPointY) * (startY - insectPointY));
	double distanceEndPoint_Insection = sqrt((endX - insectPointX) * (endX - insectPointX) + (endY - insectPointY) * (endY - insectPointY));
	if (distanceStartPoint_Insection < 0.0001)   //���������A��
	{
		inSectPointZ = startZ;
	}
	else
		if (distanceEndPoint_Insection < 0.0001)   //���������B��
		{
			inSectPointZ = endZ;
		}
		else                               //�����㲻��A��B���򰴱������Բ�ֵ
		{
			double deltaAP = distanceStartPoint_Insection / (distanceStartPoint_Insection + distanceEndPoint_Insection);
			inSectPointZ = startZ + deltaAP * (endZ - startZ);
		}

	return inSectPointZ;
}

//�����������VECTOR��XY���õ�ָ�����Zֵ
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
		std::cout << "û�н���" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	std::cout << "�������꣺��" << interSectionPoint << ")" << std::endl;

	//���β�ֵ��������ĳ�˵㣬��ֱ��ȡֵ

	myVector3 vecA = triangleVector[0];
	myVector3 vecB = triangleVector[1];
	myVector3 vecC = triangleVector[2];

	Pt3 pA3(vecA.theXY.x(), vecA.theXY.y(), vecA.z);
	Pt3 pB3(vecB.theXY.x(), vecB.theXY.y(), vecB.z);
	Pt3 pC3(vecC.theXY.x(), vecC.theXY.y(), vecC.z);

	Segment3 seg3_AB(pA3, pB3);
	double intersectZ = getPointZfromSeg(seg3_AB, interSectionPoint);
	Pt3	interSectionPoint3(interSectionPoint.x(), interSectionPoint.y(), intersectZ); //�ó������X,Y,Z����
	std::cout << "��ֵ��߶�Ϊ:" << intersectZ << std::endl;

	//ָ��������ĳ�˵㣬��ֱ��ȡֵ��
	Segment3 seg3_CInterSection(pC3, interSectionPoint3);
	pointZ = getPointZfromSeg(seg3_CInterSection, thePt);
	Pt3 thePt3(thePt.x(), thePt.y(), pointZ);
	std::cout << "�趨��߶�Ϊ:" << pointZ << std::endl;

	bIntersect = true;
	return bIntersect;

}

//����ָ�����X,Y�ͼ��ϣ��ж��ڼ��ϵ��ĸ�Id����������
int getFaceIDfromthePointAndTriangleSet(Pt2 thePt, triangleSet theTriangleSet)
{
	int theFaceID = -1;

	//�жϵ����ĸ���������
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
			std::cout << "��(" << thePt << ")�ڵ�" << faceID << "����������" << std::endl;
			break;
		}
	}

	//������ID
	if (iterTriangleCur == iterTriangleEnd)
	{
		std::cout << "ָ����û�������ǻ���������μ��Ϸ�Χ��" << std::endl;
		return theFaceID;
	}

	theFaceID = iterTriangleCur->first;

	return theFaceID;
}

//�������ݼ��ϣ����ǻ�����������μ���
triangleSet getTriangleSetFromDataSet(myVector3Set dataSet)
{
	//���ǻ����ݼ����е�X,Y����
	Triangulation tr;
	myVector3Set::iterator iterCurVector3 = dataSet.begin(),
		iterEndVector3 = dataSet.end();
	for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
	{
		Pt2 xy = iterCurVector3->second.theXY;
		tr.insert(xy);
	}

	//���ǻ����XY�������Zֵ�������μ���
	int faceID = 0;	//���ID
	triangleSet myTriangleSet;
	myTriangleSet.clear();

	std::cout << "���������Ķ���:" << std::endl;
	faceIterator faceCur = tr.faces_begin(),
		faceEnd = tr.faces_end();
	for (; faceCur != faceEnd; faceCur++)
	{
		//���ݸ���������������Zֵ
		std::vector<myVector3> theVector;
		theVector.clear();
		theVector.resize(3);
		for (int i = 0; i < 3; i++)
		{
			double x = tr.triangle(faceCur).vertex(i).x();
			double y = tr.triangle(faceCur).vertex(i).y();
			double z = 0;
			//����XY����Zֵ
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
//�����ݼ����еó�ָ����ĸ߶�
bool getPointZFromDataSet(myVector3Set dataSet, Pt2 thePoint, double& pointZ)
{
	bool bPointInPointCloud = false;

	//���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���
	triangleSet myTriangleSet = getTriangleSetFromDataSet(dataSet);
	//�жϵ����ĸ���������
	int selectedQuadID = getFaceIDfromthePointAndTriangleSet(thePoint, myTriangleSet);

	//����ID���ҵ�����������������
	triangleSet::iterator iterTriangleCur = myTriangleSet.find(selectedQuadID),
		iterTriangleEnd = myTriangleSet.end();
	if (iterTriangleCur == iterTriangleEnd)
	{
		std::cout << "�õ�û�������μ�����" << std::endl;
		bPointInPointCloud = false;
		return bPointInPointCloud;
	}

	std::cout << "ѡ�е�������ID:" << selectedQuadID << " ����Ϊ:" << std::endl;
	std::vector<myVector3> thevec = iterTriangleCur->second;

	bool bInterSect = getPointZfromTriangleVector(thevec, thePoint, pointZ);
	if (bInterSect)
	{
		std::cout << "ָ����߶�Ϊ:" << pointZ << std::endl;
		bPointInPointCloud = true;
		return bPointInPointCloud;
	}

	return bPointInPointCloud;
}
//�����ݼ����еó�ָ���������XYZ
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
	//��ȡ��������
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
		std::cout << "��" << i << "�����ݣ�(" <<theVector.theXY.x() << "," << theVector.theXY.y() << "," << theVector.z << ")" << std::endl;
	}

	//�趨���Ե�
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
			std::cout << "ָ�����������:" << pointXYZ << std::endl;
		}
	}

	//�ж��������ݼ��ϵķ�ΧXY


	return 0;
}