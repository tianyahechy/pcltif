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
		
		//����openNI�ɼ�����
		pcl::Grabber * theInterface = new pcl::io::OpenNI2Grabber;
		
		//����ص�����
		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
		
		//ע��ص�����
		boost::signals2::connection c = theInterface->registerCallback(f);
		
		//��ʼ���յ�������
		theInterface->start();
		//�ȴ�֪���û���ctrl-c
		while (!_viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		//ֹͣ�ɼ�
		theInterface->stop();
		
	}
};
//��������ͽṹ
struct MyPointType 
{
	PCL_ADD_POINT4D;					//�õ�������4��Ԫ��
	float test;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	//ȷ��new�������������
}EIGEN_ALIGN16;							//ǿ��SSE����

//ע������ͺ�
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
	//��ȡ��������
	//reader.read("E:\\VS2013\\CSite1origc.pcd", *cloud);

	// Fill in the cloud data
	cloudPCL->width = 10;
	cloudPCL->height = 1;
	cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

	//д���ĵ�
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

	//���ļ���
	std::cout << "���ļ���" << std::endl;

	double theData;
	std::ifstream instr("E:\\out.txt");
	while (instr >> theData)
		std::cout << theData << std::endl;

	
	/*
	//���߹��ƶ���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//�洢���Ƶķ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//����kd��ָ��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//��cloud����tree����
	tree->setInputCloud(cloud);
	//Ϊ���߹��ƶ������õ���
	n.setInputCloud(cloud);
	//������������
	n.setSearchMethod(tree);
	//����k������kֵΪ20
	n.setKSearch(20);
	n.compute(*normals);

	//����������
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	//�洢������
	pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
	*/
	//PCL����ļ��ĸ�ʽ

	//��CGAL����
	
	/*
	//��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//���õ��ƹ���������
	tree2->setInputCloud(cloudWithNormals);
	//�������ǻ�����
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//�洢�������ǻ�������ģ��
	pcl::PolygonMesh triangles;
	//�������ӵ�֮��������루��Ϊ���������߳�)
	gp3.setSearchRadius(0.025);
	//���ø���������ֵ
	//���ñ��������������ڽ����������Ϊ2.5��Ϊ����Ӧ�����ܶȵı仯
	gp3.setMu(2.5);
	//������������������������Ϊ100
	gp3.setMaximumNearestNeighbors(100);
	//����ĳ�㷨�߷���ƫ�������㷨�߷�������Ƕ�Ϊ45��
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	//�������ǻ���õ��������ڽ���С�Ƕ�Ϊ10��
	gp3.setMinimumAngle(M_PI / 18);
	//�������ǻ���õ��������ڽ����Ƕ�Ϊ120��
	gp3.setMaximumAngle(2 * M_PI / 3);
	//���øò�����֤���߳���һ��
	gp3.setNormalConsistency(true);
	//�����������Ϊ�������
	gp3.setInputCloud(cloudWithNormals);
	//����������ʽΪtree2
	gp3.setSearchMethod(tree2);
	//�ؽ���ȡ���ǻ�
	gp3.reconstruct(triangles);
	pcl::io::saveVTKFile("mesh.vtk", triangles);

	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	*/



	/*
	std::cerr << "total number is " << cloud->points.size() << std::endl;
	
	//����һ����������������ɢ��NAN
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	//���÷ָ��ֶ�Ϊz����
	pass.setFilterFieldName("z");
	//���÷ָ���ֵ��ΧΪ��0,1.1������Z�᷽�򳬹��÷�Χ�ĵ㼯���˵�
	pass.setFilterLimits(200, 600);
	pass.filter(*cloudFiltered);
	std::cerr << "pointCloud after filtering has:" << cloudFiltered->points.size() << "data points." << std::endl;

	//inlinersָ��洢���Ʒָ��Ľ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//�����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//�����Ż�ϵ��
	seg.setOptimizeCoefficients(true);
	//���÷ָ�ģ������Ϊƽ��ģ��
	seg.setModelType(pcl::SACMODEL_PLANE);
	//���þ�����ֵΪ0.01
	seg.setDistanceThreshold(0.01);
	//�����������Ϊ�˲������
	seg.setInputCloud(cloudFiltered);
	//ִ�зָ�
	pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients);
	seg.segment(*inliers, *coefficients);

	//���˲���ĵ㼯ͶӰ��ƽ��ģ����
	//����ͶӰ�˲�����
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	//����ͶӰģ��ΪSACMODEL_PLANE
	proj.setModelType(pcl::SACMODEL_PLANE);
	//�����������Ϊ�˲���ĵ���
	proj.setInputCloud(cloudFiltered);
	//�����Ƶõ���ƽ��coefficeints��������ΪͶӰƽ��ģ��ϵ��
	proj.setModelCoefficients(coefficients);
	//�õ�ͶӰ��ĵ���
	proj.filter(*cloudProjected);
	std::cerr << "pointcloud after projected has:" << cloudProjected->points.size() << "data points" << std::endl;

	//�洢��ȡ������ϵĵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull(new pcl::PointCloud<pcl::PointXYZ>);
	//�����������ȡ����
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	//�����������ΪͶӰ�����
	chull.setInputCloud(cloudProjected);
	//����alphaֵΪ0.1
	chull.setAlpha(0.1);
	//�ؽ���ȡ�����������
	chull.reconstruct(*cloudHull);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test.pcd", * cloud) == -1 )
	{
		PCL_ERROR("couldn't read file test.pcd\n");
		//return -1;
	}
	
	//������С����ʵ�ֵĶ���mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	//��������С���˼�������Ҫ���з��߹���
	mls.setComputeNormals(true);
	mls.setPolynomialFit(true);
	pcl::PointCloud<pcl::PointNormal> cloudNormalXYZ;
	//pcl::io::savePCDFile("test-mls.pcd", cloudNormalXYZ); 15�µ�һ������������
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
	//��������
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
	//�洢����ʱ��Ҫnormal����
	pcl::PointCloud<pcl::Normal> nCloudB;
	//�洢����XYZ��normal��ĵ���
	pcl::PointCloud<pcl::PointNormal> pnCloudC;
	//������������
	//������Ϊ�������
	cloudA.height = cloudB.height = nCloudB.height = 1;
	//����cloudA�����Ϊ3
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

	//����A+B=C
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
	
	//��ϵͳʱ���ʼ���������
	srand(time(NULL));
	//��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudkdTree(new pcl::PointCloud<pcl::PointXYZ>);
	//��������
	cloudkdTree->width = 1000;
	//�������
	cloudkdTree->height = 1;
	cloudkdTree->points.resize(cloudkdTree->width * cloudkdTree->height);
	for (size_t i = 0; i < cloudkdTree->points.size(); i++)
	{
		cloudkdTree->points[i].x = 1024.0 * rand() / (RAND_MAX + 1.0f);
		cloudkdTree->points[i].y = 1024.0 * rand() / (RAND_MAX + 1.0f);
		cloudkdTree->points[i].z = 1024.0 * rand() / (RAND_MAX + 1.0f);

	}
	//����kdTreeFLANN����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//���������ռ�
	kdtree.setInputCloud(cloudkdTree);
	//�����ѯ�㲢�����ֵ
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0 * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0 * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0 * rand() / (RAND_MAX + 1.0f);

	//k��������
	int k = 10;

	std::cout << "k nearest neighbor search at ("
		<< searchPoint.x
		<< searchPoint.y
		<< searchPoint.z
		<< ") with k = " << k << std::endl;

	//�洢��ѯ���������
	std::vector<int> pointIdxNKNSearch(k);
	//�洢���ڵ��Ӧ����ƽ��
	std::vector<float> pointNKNSquaredDistance(k);
	
	//ִ��k��������
	kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);

	//��ӡ���н�������
	for (size_t i = 0; i < pointIdxNKNSearch.size();i++)
	{
		std::cout << " " << cloudkdTree->points[pointIdxNKNSearch[i]].x
			<< " " << cloudkdTree->points[pointIdxNKNSearch[i]].y
			<< " " << cloudkdTree->points[pointIdxNKNSearch[i]].z
			<< "(squared distance:" << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	//�뾶r�ڽ���������ʽ

	//�洢��ѯ���������
	std::vector<int> pointIdxRadiusSearch;
	//�洢���ڵ��Ӧ����ƽ��
	std::vector<float> pointRadiusSquaredDistance;
	//������ɰ뾶
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	//ִ��r�뾶����
	kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	std::cout << "radius search" << std::endl;
	//��ӡ���н�������
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