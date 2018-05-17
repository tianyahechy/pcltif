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
#include <CGAL/Triangle_2_Point_2_intersection.h>
#include <vector>
#include <list>
#include <map>

typedef CGAL::Cartesian<double>								theKernel;
typedef	CGAL::Triangulation_2<theKernel>					Triangulation;
typedef CGAL::Point_2<theKernel>							Pt2;

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

typedef std::map<int, myVector3> myVector3Set;
typedef std::pair<int, myVector3> myVector3Pair;


int main()
{
	/*
	pcl::PointCloud<MyPointType> cloud;
	cloud.points.resize(2);
	cloud.width = 2;
	cloud.height = 1;

	cloud.points[0].test = 1;
	cloud.points[1].test = 2;
	cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
	cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 3;

	pcl::io::savePCDFile("test.pcd", cloud);
	*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	//读取点云数据
	reader.read("E:\\VS2013\\CSite1origc.pcd", *cloud);
	
	Triangulation tr;
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
		tr.insert(theVector.theXY);
		std::cout << "第" << i << "个数据：(" <<theVector.theXY.x() << "," << theVector.theXY.y() << "," << theVector.z << ")" << std::endl;
	}

	
	std::cout << "测试map数据:" << std::endl;
	myVector3Set::iterator iterCurVector3 = vector3Set.begin(),
		iterEndVector3 = vector3Set.end();
	for (; iterCurVector3 != iterEndVector3; iterCurVector3++)
	{
		std::cout << "第" << iterCurVector3->first << "个数据：(" << iterCurVector3->second.theXY.x() << "," << iterCurVector3->second.theXY.y() << "," << iterCurVector3->second.z << ")" << std::endl;
		
	}
	std::cout << "输出三角化的顶点" << std::endl;
	Vertex_iterator vertexBegin = tr.vertices_begin(),
		vertexEnd = tr.vertices_end();
	for (; vertexBegin != vertexEnd; vertexBegin++)
	{
		std::cout << vertexBegin->point() << std::endl;
	}
	return 0;
}