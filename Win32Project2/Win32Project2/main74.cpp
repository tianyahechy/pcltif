#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/narf.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //����fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/range_image/range_image.h>  //���ͼ��ͷ�ļ�

#include "AfxUtil.h"

/*
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::Narf36> narfFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//������
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	est_normal.compute(*point_normal);
	//fpfh ����
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4); //ָ��4�˼���
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}

//Narf����

narfFeature::Ptr compute_narf_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//������
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	est_normal.compute(*point_normal);
	//fpfh ����
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4); //ָ��4�˼���
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}
*/
int main(int argc, char **argv)
{
	/*
	clock_t start, end, time;
	start = clock();
	pcl::PCDReader reader;
	std::string srcFileName = "E:\\DEM-2013.pcd";
	std::string targetFileName = "E:\\DEM-2016.pcd";
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);

	reader.read(srcFileName.c_str(), *source);
	reader.read(targetFileName.c_str(), *target);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

	//����(ռ���˴󲿷�����ʱ��)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target);
	sac_ia.setTargetFeatures(target_fpfh);
	pointcloud::Ptr align(new pointcloud);
	//  sac_ia.setNumberOfSamples(20);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
	//sac_ia.setCorrespondenceRandomness(6); //���ü���Э����ʱѡ����ٽ��ڵ㣬��ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
	sac_ia.align(*align);
	end = clock();
	cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

	pcl::io::savePCDFile("E:\\crou_output.pcd", *align);
	//  pcl::io::savePCDFile ("final_align.pcd", *final);
	*/
	/*
	//������ƶ���
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	for (float y = -0.5f; y <= 0.5f; y+= 0.01f)
	{
		for (float z = -0.5f; z <= 0.5f; z++)
		{
			pcl::PointXYZ point;
			point.x = 2.0f - y;
			point.y = y;
			point.z = z;
			pointCloud.points.push_back(point);
		}

	}
	
	pointCloud.width = (uint32_t)pointCloud.points.size();
	pointCloud.height = 1;
	*/
	pcl::PCDReader reader;
	std::string strFileName = "E:\\DEM-2013.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(strFileName, *cloud);
	Pt3 midPoint(0, 0, 0);
	double maxDistance = 0;
	double minZ = 0;
	double maxZ = 0;
	double xResolution = 1;
	double yResolution = -1;
	int xSize = 0;
	int ySize = 0;
	util::getPCLMidPointAndDistance(cloud, xResolution, yResolution, midPoint, maxDistance, minZ, maxZ, xSize, ySize);
	
	double midZ = midPoint.z() + maxDistance;
	Pt3 cameraPos(midPoint.x(), midPoint.y(), midZ);

	std::cout << " minZ= " << minZ << ", maxZ = " << maxZ << std::endl;
	
	//����(1)
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));
	//����(360)
	float maxAngleWidth = angularResolution * 360.0f;
	//����(180)
	float maxAngleHeight = angularResolution * 180.0f;
	//�ɼ�λ��
	Eigen::Affine3f sensorPose = (Eigen::Affine3f) Eigen::Translation3f(cameraPos.x(), cameraPos.y(), cameraPos.z());
	//���ͼ����ѭ������ϵͳ
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0;
	float minRange = 0;
	int borderSize = 1;
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << rangeImage << std::endl;

	std::cout << "���ͼ����:" << std::endl;

	//д����
	pcl::PointCloud<pcl::PointXYZI> cloudOutput;
	cloudOutput.clear();

	int imageWidth = rangeImage.width;
	int imageHeight = rangeImage.height;
	for (size_t j = 0; j < imageHeight; j++)
	{
		for (size_t i = 0; i < imageWidth; i++)
		{
			pcl::PointWithRange  thePoint = rangeImage.at(i, j);
			double x = thePoint.x;
			double y = thePoint.y;
			double z = thePoint.z;
			double range = thePoint.range;

			pcl::PointXYZI thePt;
			thePt.x = x;
			thePt.y = y;
			thePt.z = z;
			thePt.intensity = range;
			cloudOutput.push_back(thePt);
		}

	}
	cloudOutput.width = cloudOutput.size();
	cloudOutput.height = 1;
	cloudOutput.is_dense = false;
	cloudOutput.resize(cloudOutput.width * cloudOutput.height);
	std::string strOutPutPointCloudName = "E:\\depthImage.pcd";
	pcl::io::savePCDFileASCII(strOutPutPointCloudName, cloudOutput);
	
	return 0;
}
