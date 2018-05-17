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

#include "gdal_priv.h"
#include <iostream>
#include "ogr_spatialref.h"
#include "ogr_api.h"
#include <ogrsf_frmts.h>
#include <cpl_string.h>
#include "ogr_core.h"
#include "gdal.h"

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
		std::cout << "û�н���" << std::endl;
		bIntersect = false;
		return bIntersect;
	}
	std::cout << "�������꣺��" << interSectionPoint << ")" << std::endl;

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
		std::vector<Pt3> thevec = iterTriangleCur->second;

		Pt2 p0 = Pt2(thevec[0].x(), thevec[0].y());
		Pt2 p1 = Pt2(thevec[1].x(), thevec[1].y());
		Pt2 p2 = Pt2(thevec[2].x(), thevec[2].y());

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
triangleSet getTriangleSetFromDataSet(pt3Set dataSet)
{
	//���ǻ����ݼ����е�X,Y����
	Triangulation tr;
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

	std::cout << "���������Ķ���:" << std::endl;
	faceIterator faceCur = tr.faces_begin(),
		faceEnd = tr.faces_end();
	for (; faceCur != faceEnd; faceCur++)
	{
		//���ݸ���������������Zֵ
		std::vector<Pt3> theVector;
		theVector.clear();
		theVector.resize(3);
		for (int i = 0; i < 3; i++)
		{
			double x = tr.triangle(faceCur).vertex(i).x();
			double y = tr.triangle(faceCur).vertex(i).y();
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

			double theX = tr.triangle(faceCur).vertex(i).x();
			double theY = tr.triangle(faceCur).vertex(i).y();
			double theZ = z;
			Pt3 vec(Pt3(theX, theY, theZ));
			theVector[i] = vec;

			std::cout << "���ǻ�����������������,��" << faceID << ",����:" << theVector[i] << std::endl;
		}

		myTriangleSet.insert(trianglePair(faceID, theVector));

		faceID++;

	}

	return myTriangleSet;
}
//�����ݼ��Ϻ������漯���еó�ָ����ĸ߶�
bool getPointZFromDataSet(pt3Set dataSet, Pt2 thePoint, double& pointZ, triangleSet myTriangleSet)
{
	bool bPointInPointCloud = false;

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
	std::vector<Pt3> thevec = iterTriangleCur->second;

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
bool getPointXYZFromDataSet(pt3Set dataSet, Pt2 thePoint, Pt3& pointXYZ,triangleSet theTriangleSet)
{

	double pointZ = 0;
	bool bPointInCloud = getPointZFromDataSet(dataSet, thePoint, pointZ, theTriangleSet);
	if (bPointInCloud)
	{
		pointXYZ = Pt3(thePoint.x(), thePoint.y(), pointZ);
	}
	return bPointInCloud;
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
			bool bPtInCloud = getPointXYZFromDataSet(dataSet, thePoint, pointXYZ, theTriangleSet);
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
rasterVec2Set getEqualXYVectorFromDataSet(pt3Set dataSet, double & minX, double& minY, int& xnumber, int& ynumber)
{
	//�ж����뼯�ϵķ�Χ
	//���ȸ������뼯�ϵ�X,Y����
	pt3Set::iterator
		iterInputExtentCur = dataSet.begin(),
		iterInputExtentEnd = dataSet.end();

	//�ж�XY����С���ֵ���Դ��ж���Χ
	iterInputExtentCur = dataSet.begin();
	minX = iterInputExtentCur->second.x();
	minY = iterInputExtentCur->second.y();
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

	//�ж����ݼ��Ĵ�С�����K=distanceX/distanceY���ٿ���,������
	double distanceX = abs(maxX - minX);
	double distanceY = abs(maxY - minY);
	double k = distanceX * 1.0 / distanceY;
	std::cout << "distanceX = " << distanceX << ", distanceY =" << distanceY << ",k=" << k << std::endl;

	//�����ı�׼
	double totalNumberofInput = dataSet.size();
	std::cout << "��������Ϊ:" << totalNumberofInput << std::endl;

	//��X,Y��������ظ���
	double ynumberf = (sqrt(totalNumberofInput*1.0 / k));
	double xnumberf = ynumberf * k;

	std::cout << "x�������ظ���:" << xnumberf << std::endl;
	std::cout << "y�������ظ���:" << ynumberf << std::endl;

	//ȡ��
	xnumber = (int)xnumberf;
	ynumber = (int)ynumberf;
	std::cout << "ȡ����" << std::endl;
	std::cout << "x�������ظ���:" << xnumber << std::endl;
	std::cout << "y�������ظ���:" << ynumber << std::endl;

	//����X��y��������ظ���������
	double perXDistance = distanceX * 1.0 / xnumber;
	double perYDistance = distanceY * 1.0 / ynumber;

	std::cout << "������X����ļ��" << perXDistance << std::endl;
	std::cout << "������Y����ļ��" << perYDistance << std::endl;

	//����������������ά��������

	rasterVec2Set myRaster;
	myRaster.clear();

	for (int i = 0; i < ynumber; i++)
	{
		std::vector<Pt2> imagePointVector2;
		imagePointVector2.clear();
		for (int j = 0; j < xnumber; j++)
		{
			double x = minX + perXDistance * j;
			double y = minY + perYDistance * i;
			Pt2 thisPoint = Pt2(x, y);
			imagePointVector2.push_back(thisPoint);
			std::cout << "�ӵ������룬�����ά����:(" << i << "," << j << ")=" << thisPoint << std::endl;
		}
		myRaster.insert(rasterVec2Pair(i, imagePointVector2));
	}

	return myRaster;
}

void createRasterFile(const char* strImageName, int bandSize, int xResolution, int yRosultion, double topLeftX, double topLeftY, double rotationX = 0, double rotationY = 0)
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
	GDALDataset * poDS = poDriver->Create(strImageName, xResolution, yRosultion, bandSize, GDT_Byte, NULL);
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
	dGeotransform[4] = yRosultion;	//����ֱ���
	dGeotransform[5] = rotationY;	//��ת�Ƕ�

	poDS->SetGeoTransform(dGeotransform);
	GDALClose((GDALDatasetH)poDS);
	std::cout << "���ݼ��رգ�" << std::endl;


}

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
	unsigned char * pBuf = new unsigned char[iWidth];
	memset(pBuf, 0, sizeof(unsigned char) * iWidth);

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
		std::vector<Pt3>::iterator
			iterCurLine = theLine.begin(),
			iterEndLine = theLine.end();
		for (; iterCurLine != iterEndLine; iterCurLine++)
		{
			Pt3 theRasterPoint = *iterCurLine;
			double valuef = theRasterPoint.z();
			std::cout << "the value is :" << valuef << std::endl;
			int theValue = abs((int)valuef) % 255;

			//��ʼ����ǰ�е���Ԫֵ��
			memset(pBuf, theValue, sizeof(unsigned char) * iWidth);

			//����Ԫֵд��ͼ��
			pBand1->RasterIO(GF_Write, 0, id, iWidth, 1, pBuf, iWidth, 1, GDT_Byte, 0, 0);
		}
	}

	GDALClose((GDALDatasetH)poDS);
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	//��ȡ��������
	reader.read("E:\\VS2013\\CSite1origc.pcd", *cloud);
	
	pt3Set vector3Set;
	vector3Set.clear();
	//for (size_t i = 0; i < cloud->points.size(); i++)
	for (size_t i = 0; i < 1000; i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		Pt3 theVector(Pt3(x, y, z));
		vector3Set.insert(pt3Pair(i, theVector));
		std::cout << "��" << i << "�����ݣ�(" <<theVector.x() << "," << theVector.y() << "," << theVector.z() << ")" << std::endl;
	}

	//��������㼯�����ȷ�XY��࣬�õ�ͼ�����꼯��vector
	int xRosolution = 0;
	int yRosolution = 0;
	double topLeftX = 0;
	double topLeftY = 0;
	rasterVec2Set imagePointVector2Set = getEqualXYVectorFromDataSet(vector3Set, topLeftX, topLeftY, xRosolution, yRosolution);

	//�õ������漯�ϡ���ȡ������

	//���ǻ����ݼ����е�X,Y����,�����Zֵ�������μ���
	std::cout << "���ǻ�" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);
	//�õ�ͼ��������XYZ
	rasterVec3Set imageSet = getPt3SetFromDataSetAndPt2Vector(vector3Set, imagePointVector2Set,myTriangleSet);
	rasterVec3Set::iterator
		iterImageCur = imageSet.begin(),
		iterImageEnd = imageSet.end();
	for (; iterImageCur != iterImageEnd; iterImageCur++)
	{
		std::vector<Pt3> theVec3Line = iterImageCur->second;
		std::vector<Pt3>::iterator
			iterCurPt3 = theVec3Line.begin(),
			iterEndPt3 = theVec3Line.end();
		for (; iterCurPt3 != iterEndPt3; iterCurPt3++)
		{
			Pt3 curImage = *iterCurPt3;
			std::cout << "������ͼ������Ϊ" << curImage << std::endl;
		}
	}

	//���������ļ�
	const char * pszRasterFile = "E:\\PCLOutPut.tif";
	int bandSize = 1;
	createRasterFile(pszRasterFile, bandSize, xRosolution, yRosolution, topLeftX, topLeftY);
	//���������ļ�,����.tiff�ļ�
	UpdateRasterFile(pszRasterFile, imageSet);
	return 0;
}