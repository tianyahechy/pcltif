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
typedef	CGAL::Triangulation_2<K>									Triangulation;
typedef Triangulation::Vertex_circulator					Vertex_circulator;
typedef Triangulation::Point								Point;

class SimpleOpenNIViewer
{
private:
	pcl::visualization::CloudViewer _viewer;
public:
	SimpleOpenNIViewer() :_viewer("PCL OpenNIViewer") {}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
	{
		static unsigned count = 0;
		static double last = pcl::getTime();
		if (++count == 30)
		{
			double now = pcl::getTime();
			std::cout << "distance of center pixel:" << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z << "mm.Average framerate: " << double(count) / double(now - last)
				<< " HZ" << std::endl;
		}
	}

	void run()
	{
		
		//创建openNI采集对象
		pcl::Grabber * theInterface = new pcl::io::OpenNI2Grabber;
		
		//定义回调函数
		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
		
		//注册回调函数
		boost::signals2::connection c = theInterface->registerCallback(f);
		
		//开始接收点云数据
		theInterface->start();
		//等待知道用户按ctrl-c
		while (!_viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		//停止采集
		theInterface->stop();
		
	}
};
//定义点类型结构
struct MyPointType 
{
	PCL_ADD_POINT4D;					//该点类型有4个元素
	float test;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	//确保new操作符对其操作
}EIGEN_ALIGN16;							//强制SSE对齐

//注册点类型宏
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,
	( float, x, x )
	( float, y, y )
	( float, z, z )
	( float, test, test )
	)

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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> ),
		cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>),
		cloudProjected(new pcl::PointCloud<pcl::PointXYZ>),
		cloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	//读取点云数据
	//reader.read("E:\\VS2013\\CSite1origc.pcd", *cloud);

	// Fill in the cloud data
	cloudPCL->width = 10;
	cloudPCL->height = 1;
	cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

	//写入文档
	for (size_t i = 0; i < cloudPCL->points.size(); i++)
	{
		cloudPCL->points[i].x = 3*i;
		cloudPCL->points[i].y = 3*i+1;
		cloudPCL->points[i].z = 3*i+2;

	}
	

	FILE * outFile = fopen("E:\\out.txt", "w");
	for (size_t i = 0; i < cloudPCL->points.size(); i++ )
	{
		std::cout << "x=" << cloudPCL->points[i].x << ", y = " << cloudPCL->points[i].y << ",z=" << cloudPCL->points[i].z << std::endl;
		fprintf(outFile, "%f ", cloudPCL->points[i].x);
		fprintf(outFile, "%f ", cloudPCL->points[i].y);
		fprintf(outFile, "%f ", cloudPCL->points[i].z);
	
	}
	fclose(outFile);

	//读文件流
	std::cout << "读文件流" << std::endl;

	double theData;
	std::ifstream instr("E:\\out.txt");
	while (instr >> theData)
		std::cout << theData << std::endl;

	
	/*
	//法线估计对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//存储估计的法线
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//定义kd树指针
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//用cloud构建tree对象
	tree->setInputCloud(cloud);
	//为法线估计对象设置点云
	n.setInputCloud(cloud);
	//设置搜索方法
	n.setSearchMethod(tree);
	//设置k搜索的k值为20
	n.setKSearch(20);
	n.compute(*normals);

	//设置有向云
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	//存储有向云
	pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
	*/
	//PCL输出文件的格式

	//用CGAL进行
	
	/*
	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//利用点云构建搜索树
	tree2->setInputCloud(cloudWithNormals);
	//定义三角化对象
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//存储最终三角化的网格模型
	pcl::PolygonMesh triangles;
	//设置连接点之间的最大距离（即为三角形最大边长)
	gp3.setSearchRadius(0.025);
	//设置各参数特征值
	//设置被样本点搜索其邻近点的最大距离为2.5，为了适应点云密度的变化
	gp3.setMu(2.5);
	//设置样本点可搜索的邻域个数为100
	gp3.setMaximumNearestNeighbors(100);
	//设置某点法线方向偏离样本点法线方向的最大角度为45度
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	//设置三角化后得到三角形内角最小角度为10度
	gp3.setMinimumAngle(M_PI / 18);
	//设置三角化后得到三角形内角最大角度为120度
	gp3.setMaximumAngle(2 * M_PI / 3);
	//设置该参数保证法线朝向一致
	gp3.setNormalConsistency(true);
	//设置输入点云为有向点云
	gp3.setInputCloud(cloudWithNormals);
	//设置搜索方式为tree2
	gp3.setSearchMethod(tree2);
	//重建提取三角化
	gp3.reconstruct(triangles);
	pcl::io::saveVTKFile("mesh.vtk", triangles);

	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	*/



	/*
	std::cerr << "total number is " << cloud->points.size() << std::endl;
	
	//建立一个过滤器来消除杂散的NAN
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	//设置分割字段为z坐标
	pass.setFilterFieldName("z");
	//设置分割阈值范围为（0,1.1），将Z轴方向超过该范围的点集过滤掉
	pass.setFilterLimits(200, 600);
	pass.filter(*cloudFiltered);
	std::cerr << "pointCloud after filtering has:" << cloudFiltered->points.size() << "data points." << std::endl;

	//inliners指针存储点云分割后的结果
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//设置优化系数
	seg.setOptimizeCoefficients(true);
	//设置分割模型类型为平面模型
	seg.setModelType(pcl::SACMODEL_PLANE);
	//设置距离阈值为0.01
	seg.setDistanceThreshold(0.01);
	//设置输入点云为滤波后点云
	seg.setInputCloud(cloudFiltered);
	//执行分割
	pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients);
	seg.segment(*inliers, *coefficients);

	//将滤波后的点集投影到平面模型上
	//点云投影滤波对象
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	//设置投影模型为SACMODEL_PLANE
	proj.setModelType(pcl::SACMODEL_PLANE);
	//设置输入点云为滤波后的点云
	proj.setInputCloud(cloudFiltered);
	//将估计得到的平面coefficeints参数设置为投影平面模型系数
	proj.setModelCoefficients(coefficients);
	//得到投影后的点云
	proj.filter(*cloudProjected);
	std::cerr << "pointcloud after projected has:" << cloudProjected->points.size() << "data points" << std::endl;

	//存储提取多边形上的点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull(new pcl::PointCloud<pcl::PointXYZ>);
	//创建多边形提取对象
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	//设置输入点云为投影后点云
	chull.setInputCloud(cloudProjected);
	//设置alpha值为0.1
	chull.setAlpha(0.1);
	//重建提取创建凹多边形
	chull.reconstruct(*cloudHull);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test.pcd", * cloud) == -1 )
	{
		PCL_ERROR("couldn't read file test.pcd\n");
		//return -1;
	}
	
	//定义最小二乘实现的对象mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	//设置在最小二乘计算中需要进行法线估计
	mls.setComputeNormals(true);
	mls.setPolynomialFit(true);
	pcl::PointCloud<pcl::PointNormal> cloudNormalXYZ;
	//pcl::io::savePCDFile("test-mls.pcd", cloudNormalXYZ); 15章第一个例子有问题
	*/
	/*
	else
	{
		std::cout << "read correctly." << std::endl;
		for (size_t i = 0; i < cloud->points.size(); i++ )
		{
			std::cout << " " << cloud->points[i].x
				<< " " << cloud->points[i].y
				<< " " << cloud->points[i].z
				<< std::endl;

		}
	}
	
	pcl::PointCloud<pcl::PointXYZ> saveCloud;
	//创建点云
	saveCloud.width = 5;
	saveCloud.height = 1;
	saveCloud.is_dense = false;
	saveCloud.points.resize(saveCloud.width * saveCloud.height);
	for (size_t i = 0; i < saveCloud.points.size(); i++ )
	{
		saveCloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		saveCloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		saveCloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		
	}

	pcl::io::savePCDFileASCII("test2.pcd", saveCloud);
	for (size_t i = 0; i < saveCloud.points.size(); i++)
	{
		std::cout << " " << saveCloud.points[i].x
			<< " " << saveCloud.points[i].y
			<< " " << saveCloud.points[i].z
			<< std::endl;

	}

	pcl::PointCloud<pcl::PointXYZ> cloudA, cloudB, cloudC;
	//存储连接时需要normal点云
	pcl::PointCloud<pcl::Normal> nCloudB;
	//存储连接XYZ与normal后的点云
	pcl::PointCloud<pcl::PointNormal> pnCloudC;
	//创建点云数据
	//都设置为无序点云
	cloudA.height = cloudB.height = nCloudB.height = 1;
	//设置cloudA点个数为3
	cloudA.width = 3;
	cloudA.points.resize(cloudA.width * cloudA.height);

	cloudB.width = 2;
	cloudB.points.resize(cloudB.width * cloudB.height);

	nCloudB.width = 3;
	nCloudB.points.resize(nCloudB.width * nCloudB.height);

	for (size_t i = 0; i < cloudA.points.size(); i++)
	{
		cloudA.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudA.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudA.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

	}
	for (size_t i = 0; i < cloudB.points.size(); i++)
	{
		cloudB.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudB.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudB.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

	}
	for (size_t i = 0; i < nCloudB.points.size(); i++)
	{
		nCloudB.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
		nCloudB.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
		nCloudB.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);

	}
	//a+b=c
	std::cerr << "CloudA:" << std::endl;
	for (size_t i = 0; i < cloudA.points.size(); i++ )
	{
		std::cerr << " " << cloudA.points[i].x
			<< " " << cloudA.points[i].y
			<< " " << cloudA.points[i].z
			<< std::endl;
	}
	std::cerr << "CloudB:" << std::endl;
	for (size_t i = 0; i < cloudB.points.size(); i++)
	{
		std::cerr << " " << cloudB.points[i].x
			<< " " << cloudB.points[i].y
			<< " " << cloudB.points[i].z
			<< std::endl;
	}
	std::cerr << "nCloudB:" << std::endl;
	for (size_t i = 0; i < nCloudB.points.size(); i++)
	{
		std::cerr << " " << nCloudB.points[i].normal[0]
			<< " " << nCloudB.points[i].normal[1]
			<< " " << nCloudB.points[i].normal[2]
			<< std::endl;
	}

	//连接A+B=C
	//cloudC = cloudA + cloudB;
	std::cerr << "CloudC:" << std::endl;
	for (size_t i = 0; i < cloudC.points.size(); i++)
	{
		std::cerr << " " << cloudC.points[i].x
			<< " " << cloudC.points[i].y
			<< " " << cloudC.points[i].z
			<< std::endl;
	}

	//pnCloudC = cloudA + nCloudB
	pcl::concatenateFields(cloudA, nCloudB, pnCloudC);
	for (size_t i = 0; i < pnCloudC.points.size(); i++)
	{
		std::cerr << " " << pnCloudC.points[i].x
			<< " " << pnCloudC.points[i].y
			<< " " << pnCloudC.points[i].z 
			<< " " << pnCloudC.points[i].normal[0]
			<< " " << pnCloudC.points[i].normal[1]
			<< " " << pnCloudC.points[i].normal[2]
			<< std::endl;
	}

	//SimpleOpenNIViewer openViewer;
	//openViewer.run();
	
	//用系统时间初始化随机种子
	srand(time(NULL));
	//创建点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudkdTree(new pcl::PointCloud<pcl::PointXYZ>);
	//点云数量
	cloudkdTree->width = 1000;
	//无序点云
	cloudkdTree->height = 1;
	cloudkdTree->points.resize(cloudkdTree->width * cloudkdTree->height);
	for (size_t i = 0; i < cloudkdTree->points.size(); i++)
	{
		cloudkdTree->points[i].x = 1024.0 * rand() / (RAND_MAX + 1.0f);
		cloudkdTree->points[i].y = 1024.0 * rand() / (RAND_MAX + 1.0f);
		cloudkdTree->points[i].z = 1024.0 * rand() / (RAND_MAX + 1.0f);

	}
	//创建kdTreeFLANN对象
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//设置搜索空间
	kdtree.setInputCloud(cloudkdTree);
	//定义查询点并随机赋值
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0 * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0 * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0 * rand() / (RAND_MAX + 1.0f);

	//k近邻搜索
	int k = 10;

	std::cout << "k nearest neighbor search at ("
		<< searchPoint.x
		<< searchPoint.y
		<< searchPoint.z
		<< ") with k = " << k << std::endl;

	//存储查询点近邻索引
	std::vector<int> pointIdxNKNSearch(k);
	//存储近邻点对应距离平方
	std::vector<float> pointNKNSquaredDistance(k);
	
	//执行k紧邻搜索
	kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);

	//打印所有近邻坐标
	for (size_t i = 0; i < pointIdxNKNSearch.size();i++)
	{
		std::cout << " " << cloudkdTree->points[pointIdxNKNSearch[i]].x
			<< " " << cloudkdTree->points[pointIdxNKNSearch[i]].y
			<< " " << cloudkdTree->points[pointIdxNKNSearch[i]].z
			<< "(squared distance:" << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	//半径r内近邻搜索方式

	//存储查询点近邻索引
	std::vector<int> pointIdxRadiusSearch;
	//存储近邻点对应距离平方
	std::vector<float> pointRadiusSquaredDistance;
	//随机生成半径
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	//执行r半径搜索
	kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	std::cout << "radius search" << std::endl;
	//打印所有近邻坐标
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
	{
		std::cout << " " << cloudkdTree->points[pointIdxRadiusSearch[i]].x
			<< " " << cloudkdTree->points[pointIdxRadiusSearch[i]].y
			<< " " << cloudkdTree->points[pointIdxRadiusSearch[i]].z
			<< "(squared distance:" << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	*/
	return 0;
}