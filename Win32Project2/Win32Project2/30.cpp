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
//���������������ݼ�
typedef std::map<int, Pt3> pt3Set;
typedef std::pair<int, Pt3> pt3Pair;

typedef std::map<int, int> testSet;
typedef std::pair<int, int> testPair;

//������Ƭ��Ӧ�ļ���
typedef std::map<int, std::vector<Pt3> >	triangleSet;
typedef std::pair<int, std::vector<Pt3> >	trianglePair;

//դ�����м��ϣ�XY���꣩
typedef std::map<int, std::vector<Pt2>>		rasterVec2Set;
typedef std::pair<int, std::vector<Pt2>>	rasterVec2Pair;

//դ�����м��ϣ�XYZ���꣩
typedef std::map<int, std::vector<Pt3>>		rasterVec3Set;
typedef std::pair<int, std::vector<Pt3>>	rasterVec3Pair;

//����ϸ��
struct PCLDetail
{
	double minX;	//���Ƶ�����
	double minY;
	double maxX;
	double maxY;
	double xDistance;//���Ƶ�X����
	double yDistance;//���Ƶ�Y����
};
//ָ����XY���ϵó�

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

//�����������߶��Ƿ��ཻ
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
		//std::cout << "���ǽ���" << std::endl;
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

// �����������VECTOR��XY���õ�ָ�����Zֵ
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
//����ָ�����X,Y�ͼ��ϣ��ж��ڼ��ϵ��ĸ�Id����������
int getFaceIDfromthePointAndTriangleSet(Pt2 thePt, triangleSet theTriangleSet)
{
	int theFaceID = -1;

	Vector3 vecA(0, 0, 0);
	Vector3 vecB(0, 0, 0);
	Vector3 vecC(0, 0, 0);
	Vector3 vecPt(thePt.x(), thePt.y(), 0);
	//�жϵ����ĸ���������
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
			std::cout << "��(" << thePt << ")�ڵ�" << faceID << "����������" << std::endl;
			break;
		}
	}

	//������ID
	if (iterTriangleCur == iterTriangleEnd)
	{
		//std::cout << "��" << thePt << "û������������" << std::endl;
		return theFaceID;
	}

	theFaceID = iterTriangleCur->first;

	return theFaceID;
}

//�������ݼ��ϣ����ǻ�����������μ���
triangleSet getTriangleSetFromDataSet(pt3Set dataSet)
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

		//��������ε����ĵ㣬�Ի��ַ�Χ
		//double centerX = (theVector[0].x() + theVector[1].x() + theVector[2].x()) / 3.0;
		//double centerY = (theVector[0].y() + theVector[1].y() + theVector[2].y()) / 3.0;
		//int triangleZone = testWhichTraingle(centerX, centerY, numberofEachColumn, minX, minY, maxX, maxY);
		myTriangleSet.insert(trianglePair(faceID, theVector));

		faceID++;

	}

	return myTriangleSet;
}
//�����ݼ��Ϻ������漯���еó������ĸ߶ȺͲ���������ε�
bool getPt3InsertAndPt3VecInOneTriangleFromDataSet(pt3Set dataSet, Pt2 thePoint, Pt3& outPutPoint3d, triangleSet myTriangleSet, std::vector<Pt3>& trianglePointVector)
{
	bool bPointInPointCloud = false;

	//�жϵ����ĸ���������
	int selectedQuadID = getFaceIDfromthePointAndTriangleSet(thePoint, myTriangleSet);
	//����ID���ҵ�����������������
	triangleSet::iterator iterTriangleCur = myTriangleSet.find(selectedQuadID),
		iterTriangleEnd = myTriangleSet.end();
	if (iterTriangleCur == iterTriangleEnd)
	{
		std::cout << "��(" << thePoint << ")û������������" << std::endl;
		bPointInPointCloud = false;
		return bPointInPointCloud;
	}

	//std::cout << "ѡ�е�������ID:" << selectedQuadID << " ����������Ϊ:" << std::endl;
	trianglePointVector = iterTriangleCur->second;
	printf("(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f)\n",
		trianglePointVector[0].x(), trianglePointVector[0].y(), trianglePointVector[0].z(),
		trianglePointVector[1].x(), trianglePointVector[1].y(), trianglePointVector[1].z(),
		trianglePointVector[2].x(), trianglePointVector[2].y(), trianglePointVector[2].z());

	double pointZ = -1;
	bool bInterSect = getPointZfromTriangleVector(trianglePointVector, thePoint, pointZ);
	if (bInterSect)
	{
		printf("��ֵ������Ϊ(%0.6f,%0.6f,%0.6f)\n", thePoint.x(), thePoint.y(), pointZ);
		outPutPoint3d = Pt3(thePoint.x(), thePoint.y(), pointZ);
		bPointInPointCloud = true;
		return bPointInPointCloud;
	}

	return bPointInPointCloud;
}

//�ӵ������ݼ��Ϻ�����XYvector�������μ�����,�õ�դ�����ά����
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

//���ݵ��Ƽ�������Ƶ�ϸ������
PCLDetail getPCLDetail(pt3Set dataSet)
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
//�������뼯����ȷֲ�ֵXY����
rasterVec2Set getEqualXYVectorFromDataSet(pt3Set dataSet, double minX, double minY, int xSize, int ySize, double xResolution, double yResolution )
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
//ͨ��դ�񴴽�TIF�ļ�
void createRasterFile(const char* strImageName, int bandSize, int xSize, int ySize, int xResolution, int yResolution, double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0)
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
void UpdateRasterFile(const char* strImageName, rasterVec3Set theRasterSet)
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
		int id = iterRasterCur->first;
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

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	//��ȡ��������
	std::cout << "��ȡ��������:" << std::endl;
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
		//std::cout << "��" << i << "�����ݣ�(" <<theVector.x() << "," << theVector.y() << "," << theVector.z() << ")" << std::endl;
	}
	
	//���ݵ��Ƽ�����Ƶ�ϸ��
	PCLDetail theDetail = getPCLDetail(vector3Set);
	
	//����X��Y��������طֱ���
	double xResolution = 1.0;
	double yResolution = 1.0;

	//��X,Y��������ظ���,
	double distanceX = theDetail.xDistance;
	double distanceY = theDetail.yDistance;
	double xSizef = distanceX / xResolution;
	double ySizef = distanceY / yResolution;
	//ȡ��
	int xSize = (int)xSizef;
	int ySize = (int)ySizef;

	//�������Ͻ�Ϊminx,maxY
	double minX = theDetail.minX;
	double minY = theDetail.minY;
	
	//��������㼯�����ȷ�XY��࣬�õ�ͼ�����꼯��vector
	rasterVec2Set imagePointVector2Set = getEqualXYVectorFromDataSet(vector3Set, minX, minY, xSize, ySize,xResolution,yResolution);
	
	//���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���
	std::cout << "���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);
	rasterVec3Set imageSet = getPt3SetFromDataSetAndPt2Vector(vector3Set, imagePointVector2Set,myTriangleSet);
	
	//���������ļ�
	const char * pszRasterFile = "E:\\PCLOutPut_20000C.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//�趨ͼ�����Ͻ�
	float topLeftX = minX;
	float topLeftY = maxY;
	createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	UpdateRasterFile(pszRasterFile, imageSet);
	
	
	/*
	double testZ = -1;
	//�õ������漯�ϡ���ȡ������
	//���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���
	std::cout << "���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);

	//�õ�ͼ��������XYZ
	std::cout << "�õ�ͼ��������XYZ" << std::endl;

	//������ļ�
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
			std::cout << "��СΪ��" << pt3Vec.size() << std::endl;
	
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