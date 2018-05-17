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

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef	CGAL::Triangulation_2<K>							Triangulation;
typedef Triangulation::Face_circulator						Face_circulator;
typedef Triangulation::Point								Point;

typedef Triangulation::Face_iterator faceIterator;
typedef Triangulation::Face Face;
typedef Triangulation::Face_handle faceHandle;

struct triangleStruct
{
	double x;
	double y;
	double z;
};

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

	FILE * outFile = fopen("E:\\out.txt", "w");
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		//std::cout << "x=" << cloud->points[i].x << ", y = " << cloud->points[i].y << ",z=" << cloud->points[i].z << std::endl;
		fprintf(outFile, "%f ", cloud->points[i].x);
		fprintf(outFile, "%f ", cloud->points[i].y);
		//fprintf(outFile, "%f ", cloud->points[i].z); //Z坐标暂时不用三角化
	
	}
	fclose(outFile);

	//读文件流
	std::cout << "读文件流" << std::endl;

	std::ifstream instr("E:\\out.txt");
	std::istream_iterator<Point> triangleBegin(instr);
	std::istream_iterator<Point> triangleEnd;
	Triangulation tr;
	tr.insert(triangleBegin, triangleEnd);

	std::cout << "输出面:" << std::endl;
	faceIterator faceCur = tr.faces_begin(),
		faceEnd = tr.faces_end();
	Face face;
	faceHandle neighbor;
	for (; faceCur != faceEnd; faceCur++)
	{
		int count = 0;
		for (int i = 0; i < 3; i++)
		{
			neighbor = faceCur->neighbor(i);
			if (tr.is_infinite(neighbor))
			{
				count++;
			}
		}
		std::cout << tr.triangle(faceCur) << std::endl <<"has " << count << " infinite neighbors" << std::endl;

	}

	return 0;
}