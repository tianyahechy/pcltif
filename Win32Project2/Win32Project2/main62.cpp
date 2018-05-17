
#define _CRT_SECURE_NO_WARNINGS
#include "common.h"
#include <pcl/keypoints/harris_2d.h>

int main(int argc, char *argv[])
{
	std::string	strFileName = "e:\\test.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr convertedCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCDReader reader;
	reader.read(strFileName, *originCloud);

	int totalSize = originCloud->points.size();
	std::cout << "×ÜÊý:" << totalSize << std::endl;

	for (size_t i = 0; i < totalSize; i++)	
	{
		double x = originCloud->points[i].x;
		double y = originCloud->points[i].y;
		double z = originCloud->points[i].z;
		double intensity = originCloud->points[i].z;
		pcl::PointXYZI thePt;
		thePt.x = x;
		thePt.y = y;
		thePt.z = z;
		thePt.intensity = intensity;
		convertedCloud->push_back(thePt);
	}
	

	convertedCloud->is_dense = true;
	convertedCloud->height = 2;
	convertedCloud->width = totalSize / 2;

	int winWidth = 3;
	int winHeight = 3;
	int minDistance = 5;
	float threshold = 0;
	bool suppression = true;
	int method = 1;
	float radius = 10;

	for (int i = 0, ie = 50; i < ie; ++i)
	{
		pcl::PointCloud<pcl::PointXYZI> * out = new pcl::PointCloud<pcl::PointXYZI>;
		pcl::HarrisKeypoint2D<pcl::PointXYZI, pcl::PointXYZI> harris((pcl::HarrisKeypoint2D<pcl::PointXYZI, pcl::PointXYZI>::ResponseMethod)method, winWidth, winHeight, minDistance, threshold);
		harris.setRadiusSearch(radius);
		harris.setNumberOfThreads(1);
		harris.setInputCloud(convertedCloud);
		harris.setNonMaxSupression(suppression);
		harris.setRefine(false);
		harris.compute(*out);
		printf("run: %i, in size: %lu, out size :%lu \n", i, convertedCloud->size(), out->size());
		//delete out; //The result cloud is not deleted as this will increase the non-deterministic behaviour. 
	}
}