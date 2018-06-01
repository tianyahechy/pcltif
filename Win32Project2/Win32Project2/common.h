#pragma  once
#ifndef BOOST_SYSTEM_NO_DEPRECATED
#define BOOST_SYSTEM_NO_DEPRECATED
#endif
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
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/io/point_cloud_image_extractors.h"

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
#include <time.h>

#include "gdal_priv.h"
#include "ogr_spatialref.h"
#include "ogr_api.h"
#include <ogrsf_frmts.h>
#include <cpl_string.h>
#include "ogr_core.h"
#include "gdal.h"

#include "theVector3.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <fstream>
#include <liblas/header.hpp>

typedef CGAL::Cartesian<double>								theKernel;
typedef CGAL::Delaunay_triangulation_2<theKernel>			Delaunay;
typedef CGAL::Point_2<theKernel>							Pt2;
typedef CGAL::Point_3<theKernel>							Pt3;
typedef CGAL::Segment_2<theKernel>							Segment;
typedef CGAL::Segment_3<theKernel>							Segment3;
typedef CGAL::Ray_2<theKernel>								Ray2;

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

//��Ӿ���
struct myRect
{
	double minX;
	double minY;
	double maxX;
	double maxY;

};
//��������εĶ��㼯��
struct myTriangle
{
	std::vector<Pt3> pt3sInTriangle;  //����
	myRect theRect;					//��Ӿ���
};

//�����μ���,ÿ�ֿ�������μ���
typedef std::map<int, myTriangle> mapTrianglesOfEachQuad;
typedef std::pair<int, myTriangle> pairTrianglesOfEachQuad;

//���ֿ������μ���
typedef std::map<int, mapTrianglesOfEachQuad> mapTrianglesSetOfAllQuads;
typedef std::pair<int, mapTrianglesOfEachQuad> pairTrianglesSetOfAllQuads;

//����ϸ��
struct PCLDetail2
{
	double minX;	//���Ƶ�����
	double minY;
	double maxX;
	double maxY;
	double xDistance;//���Ƶ�X����
	double yDistance;//���Ƶ�Y����
};

//����ϸ��
struct PCLDetail
{
	double leftTopX;	//���Ƶ�����
	double leftTopY;
	double xDistance;//���Ƶ�X����
	double yDistance;//���Ƶ�Y����
};

//ÿ������
//��ά
typedef std::vector<Pt2> rasterLine2;
//��ά
typedef std::vector<Pt3> rasterLine3;

//ÿ���д��кŵĶ���
struct Pt3WithColumnID
{
	int col;	//�к�
	Pt3 pos;	//����
};

//���ǻ�����Ͷ�������Ķ�Ӧ��ϵ����
typedef std::map<Delaunay::Vertex_handle, Pt3> mapVertexHandleAndPt3;
typedef std::pair<Delaunay::Vertex_handle, Pt3> pairVertexHandleAndPt3;

enum PCLType
{	
	pointXYZ,
	pointXYZI
};

//8λ�����ݽṹ
struct Pt8Bit
{
	double x;
	double y;
	int z;
};

//8λ��ȡ
struct Pt8BitAndID
{
	int xID;
	int yID;
	int value;
};

//��ȡ��ÿ������
struct zone
{
	int xRoi;
	int yRoi;
	int widthRoi;
	int heightRoi;
};

//tif�Ĳ���
struct tifParameter
{
	double xResolution;
	double yResolution;
	double leftTopX;
	double leftTopY;
	int xSize;
	int ySize;

};