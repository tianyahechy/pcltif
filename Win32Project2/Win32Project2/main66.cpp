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

//以<x,y,z,curvature>形式定义一个新的点表示
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
	MyPointRepresentation()
	{
		//定义点的维度
		nr_dimensions_ = 4;
	}
	//重载copyToFloatArray方法来将点转化为四维数组
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

//匹配一对点云数据集并且返还结果
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downSample = false)
{
	//为了一致性和高速的下采样
	//注意：为了大数据集需要允许这项

	PointCloud::Ptr src(new PointCloud);	//存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);	//存储滤波后的目标点云
	pcl::VoxelGrid<PointT> grid;			//滤波处理对象
	if ( downSample )
	{
		grid.setLeafSize(0.05, 0.05, 0.05);	//设置滤波处理时采用的体素大小
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

	//计算点云法线
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;	//点云法线估计对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	norm_est.setSearchMethod(tree);	//设置估计对象采用的搜索对象
	norm_est.setKSearch(30);		//设置估计时进行搜索的k数

	//分别估计源和目标点云法线
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//配准
	MyPointRepresentation point_representation;
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;	//配准对象
	reg.setTransformationEpsilon(1e-6);  //设置收敛判断条件，越小精度越大，收敛也越慢
	
	//将点云中的对应点对之间的(src<->tgt)最大距离设置为10cm,大于此值的点对不考虑
	//注意：根据用户的数据集大小来调整
	reg.setMaxCorrespondenceDistance(0.1);
	//设置点表示
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
	//设置点表示
	reg.setInputCloud(points_with_normals_src);
	//设置目标点云
	reg.setInputTarget(points_with_normals_tgt);
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	point
	//设置最大迭代次数
	reg.setMaximumIterations(2);

	for (size_t i = 0; i < 30; i++)
	{
		//为了可视化的目的保存点云
		points_with_normals_src = reg_result
	}

}
int main()
{
	pcl::PCDReader reader;
	std::string strInputFileName1 = "E:\\capture0001.pcd";
	std::string strInputFileName2 = "E:\\capture0002.pcd";
	PointCloud::Ptr cloud1(new PointCloud);
	PointCloud::Ptr cloud2(new PointCloud);
	reader.read(strInputFileName1, *cloud1);
	reader.read(strInputFileName2, *cloud2);

	std::cout << cloud1->points.size() << "," << cloud2->points.size() << std::endl;

	PointCloud::Ptr temp(new PointCloud);

	return 0;
}