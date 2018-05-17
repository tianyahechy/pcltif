#include "common.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

typedef pcl::PointXYZ	PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal	PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

struct PCD
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::string f_name;
	PCD() : cloud(new pcl::PointCloud<pcl::PointXYZ>){}
};

struct PCDComparator
{
	bool operator() (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};

//��<x,y,z,curvature>��ʽ����һ���µĵ��ʾ
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
	MyPointRepresentation()
	{
		//������ά��
		nr_dimensions_ = 4;
	}
	//����copyToFloatArray����������ת��Ϊ��ά����
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

//ƥ��һ�Ե������ݼ����ҷ������
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downSample = false)
{
	//Ϊ��һ���Ժ͸��ٵ��²���
	//ע�⣺Ϊ�˴����ݼ���Ҫ��������

	PointCloud::Ptr src(new PointCloud);	//�洢�˲����Դ����
	PointCloud::Ptr tgt(new PointCloud);	//�洢�˲����Ŀ�����
	pcl::VoxelGrid<PointT> grid;			//�˲��������
	if ( downSample )
	{
		grid.setLeafSize(0.05, 0.05, 0.05);	//�����˲�����ʱ���õ����ش�С
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	//������Ʒ���
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;	//���Ʒ��߹��ƶ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	norm_est.setSearchMethod(tree);	//���ù��ƶ�����õ���������
	norm_est.setKSearch(30);		//���ù���ʱ����������k��

	//�ֱ����Դ��Ŀ����Ʒ���
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//��׼
	MyPointRepresentation point_representation;
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;	//��׼����
	reg.setTransformationEpsilon(1e-6);  //���������ж�������ԽС����Խ������ҲԽ��
	
	//�������еĶ�Ӧ���֮���(src<->tgt)����������Ϊ10cm,���ڴ�ֵ�ĵ�Բ�����
	//ע�⣺�����û������ݼ���С������
	reg.setMaxCorrespondenceDistance(0.1);
	//���õ��ʾ
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
	//���õ��ʾ
	reg.setInputCloud(points_with_normals_src);
	//����Ŀ�����
	reg.setInputTarget(points_with_normals_tgt);
	//��һ��ѭ����������ͬ�����Ż�����ʹ������ӻ�
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	//��������������
	reg.setMaximumIterations(2);

	for (size_t i = 0; i < 30; i++)
	{
		//Ϊ�˿��ӻ���Ŀ�ı������
		points_with_normals_src = reg_result;
		//����
		reg.setInputCloud(points_with_normals_src);
		reg.align(*reg_result);
		//��ÿһ������֮���ۻ�ת��
		Ti = reg.getFinalTransformation() * Ti;

		//������ת����֮ǰת��֮��Ĳ���С����ֵ
		//��ͨ����С����Ӧ���������Ƴ���
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
		{
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		}
		prev = reg.getLastIncrementalTransformation();
	}
	//�õ�Ŀ����Ƶ�Դ���Ƶı任
	targetToSource = Ti.inverse();

	//��Ŀ�����ת����Դ���
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	
	//���Դ���Ƶ�ת��Ŀ��
	*output += *cloud_src;
	final_transform = targetToSource;

}
int main()
{
	pcl::PCDReader reader;
	std::string strInputFileName1 = "E:\\DEM-2013.pcd";
	std::string strInputFileName2 = "E:\\DEM-2016.pcd";
	PointCloud::Ptr cloud1(new PointCloud);
	PointCloud::Ptr cloud2(new PointCloud);
	reader.read(strInputFileName1, *cloud1);
	reader.read(strInputFileName2, *cloud2);

	std::cout << cloud1->points.size() << "," << cloud2->points.size() << std::endl;

	PointCloud::Ptr temp(new PointCloud);
	PointCloud::Ptr result(new PointCloud);
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f pairTransform;
	pairAlign(cloud1, cloud2, temp, pairTransform, true);
	//�ѵ�ǰ���������ת����ȫ�ֱ任
	pcl::transformPointCloud(*temp, *result, GlobalTransform);

	int sizeofResult = result->points.size();
	std::cout << sizeofResult << std::endl;
	return 0;
}