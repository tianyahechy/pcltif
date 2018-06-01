#pragma  once
#ifndef BOOST_SYSTEM_NO_DEPRECATED
#define BOOST_SYSTEM_NO_DEPRECATED
#endif
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

//外接矩形
struct myRect
{
	double minX;
	double minY;
	double maxX;
	double maxY;

};
//组成三角形的顶点集合
struct myTriangle
{
	std::vector<Pt3> pt3sInTriangle;  //顶点
	myRect theRect;					//外接矩形
};

//三角形集合,每分块的三角形集合
typedef std::map<int, myTriangle> mapTrianglesOfEachQuad;
typedef std::pair<int, myTriangle> pairTrianglesOfEachQuad;

//各分块三角形集合
typedef std::map<int, mapTrianglesOfEachQuad> mapTrianglesSetOfAllQuads;
typedef std::pair<int, mapTrianglesOfEachQuad> pairTrianglesSetOfAllQuads;

//点云细节
struct PCLDetail2
{
	double minX;	//点云的四至
	double minY;
	double maxX;
	double maxY;
	double xDistance;//点云的X距离
	double yDistance;//点云的Y距离
};

//点云细节
struct PCLDetail
{
	double leftTopX;	//点云的四至
	double leftTopY;
	double xDistance;//点云的X距离
	double yDistance;//点云的Y距离
};

//每行数据
//二维
typedef std::vector<Pt2> rasterLine2;
//三维
typedef std::vector<Pt3> rasterLine3;

//每行中带列号的顶点
struct Pt3WithColumnID
{
	int col;	//列号
	Pt3 pos;	//坐标
};

//三角化句柄和顶点坐标的对应关系集合
typedef std::map<Delaunay::Vertex_handle, Pt3> mapVertexHandleAndPt3;
typedef std::pair<Delaunay::Vertex_handle, Pt3> pairVertexHandleAndPt3;

enum PCLType
{	
	pointXYZ,
	pointXYZI
};

//8位的数据结构
struct Pt8Bit
{
	double x;
	double y;
	int z;
};

//8位读取
struct Pt8BitAndID
{
	int xID;
	int yID;
	int value;
};

//截取的每块区域
struct zone
{
	int xRoi;
	int yRoi;
	int widthRoi;
	int heightRoi;
};

//tif的参数
struct tifParameter
{
	double xResolution;
	double yResolution;
	double leftTopX;
	double leftTopY;
	int xSize;
	int ySize;

};