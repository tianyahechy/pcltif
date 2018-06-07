#pragma once

#include "common.h"
#include "AfxUtil.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <pcl/correspondence.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/range_image/range_image.h>  //深度图像头文件


//sift处理
class siftProcess
{
public:
	siftProcess(int widthRoil, int heightRoil,
		std::string strInputTifName1,
		std::string strInputTifName2,
		std::string strInputPCDName1,
		std::string strInputPCDName2);
	~siftProcess();

public:
	//处理所有步骤
	void processAll();

	//从8位像素序列得到opencv图像中  （是否可以直接用opencv读取8位.tif)
	cv::Mat getOpenCVImgFrom8bitVec(std::vector<uchar> rastervec8Bit, zone theZone);
	//计算opencv图像中的SIFT特征及匹配,得出对应点对序号的vector,
	void getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, cv::Mat srcImage2, 
		std::vector<cv::Point2f>& corlinerPointVec1InOpenCVOfTheZone,
		std::vector<cv::Point2f>& corlinerPointVec2InOpenCVOfTheZone);
	//计算最小点云配对
	boost::shared_ptr<pcl::Correspondences> computeMiniCorrisponces( int sizeofColiners);
	//opencv二维序号坐标vector转PCL三维坐标vector(局部区域）
	std::vector<Pt3> convertOpenCV2DVecToPCL3DVec(
		zone theZone,
		tifParameter tifParam,
		std::vector<cv::Point2f> point2dVec, 
		std::vector<float> pixel32BitVec);
	//根据id和宽度取得像素值
	double getPixelFromID(int xID, int yID, int widthRoi, std::vector<float> pixel32BitVec);
	//通过三维坐标序列转换为点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<Pt3> vec3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<std::vector<Pt3>> vec3);
	//通过点云和偏移坐标转换为新点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFromPointCloudAndDeltaXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud,
		double deltaX, double deltaY, double deltaZ);

	//得出粗配准矩阵
	void computeRoughMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
		boost::shared_ptr<pcl::Correspondences> cor);
	//求差值点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr getDiffPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

	//根据粗配准矩阵得出粗配准点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr getRoughPointCloudFromMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
		Eigen::Matrix4f matrix);
	//对应点对从局部坐标得到指定tif的全局坐标和相应序号
	std::map<int, Pt3> getlinerSetFromTifAndLocalCoordinate(std::string strTifName,
		std::vector<cv::Point2f> localCoordinateVector,
		int xRoil, int yRoil);
	//将32位像素序列转换为8像素序列。
	std::vector<uchar> convert32bitPixelVectorTo8bitPixelVector(std::vector<float> pixelVector32bit);
	//根据都减去最小值，来计算出进行粗配准的点云，以尽可能消除坐标大小在矩阵中乘积的影响
	void ajustVecByMinXYZ(std::vector<Pt3> inputVec1,
		std::vector<Pt3> inputVec2,
		std::vector<Pt3>& outputVec1,
		std::vector<Pt3>& outputVec2,
		double& minX,
		double& minY,
		double& minZ);
	//放缩后形成新的序列用于粗配准矩阵调整
	std::vector<Pt3> adjustVecByScale(std::vector<Pt3> inputVec, double scaleX, double scaleY, double scaleZ);
	//根据都减去中间值，来计算出进行粗配准的点云，以尽可能消除坐标大小在矩阵中乘积的影响
	void ajustVecByMidXYZ(std::vector<Pt3> inputVec1,
		std::vector<Pt3> inputVec2,
		std::vector<Pt3>& outputVec1,
		std::vector<Pt3>& outputVec2,
		double& midX,
		double& midY,
		double& midZ);
	//计算出给定vector的最大最小x,y,z值
	void getMinXYZFromVec(std::vector<Pt3> vec, double& minX, double& minY, double& minZ);
	//计算调整后的序列
	std::vector<Pt3> getAjustVecFromVecAndDelta(std::vector<Pt3> vec, double minX,double minY,double minZ);
	//得到精配准矩阵
	Eigen::Matrix4f ICPRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud);
	//点云平移回来
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudFromAdjustCloudAndMinXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud, 
		double minX, double minY, double minZ);
	//分区，根据图像的（xSize,ySize,widthRoi,HeightRoi)确定(xroi,yroi,widthRoitrue,heightRoitrue)
	std::vector<std::vector<zone>> getZoneVecVec(int xSize, int ySize, int widthRoi, int heightRoi);
	std::vector<std::vector<zone>> getZoneVecVec(int xSize, int ySize, float ratioX, float ratioY, 
		int xStart, int yStart, int widthRoi, int heightRoi);
	//返回第一块区域对应的对应内点三维坐标序列
	void getColinersFromZone(zone theZone1, 
		tifParameter tif1, 
		tifParameter tif2, 
		std::vector<Pt3>& colinersOfTheZone1Vec, 
		std::vector<Pt3>& colinersOfTheZoneVec2);
	//选取一部分点云进行处理
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud, float ratioOfDataSize = 0.4 );
	//从点云中得到序列
	std::vector<Pt3> getVecFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	//将目前点对按照比例获取新点对。
	void filterColiner(std::vector<Pt3> inputVec1, std::vector<Pt3> inputVec2, 
		std::vector<Pt3>& outputVec1, std::vector<Pt3>& outputVec2, 
		float ratioFilter);
	//根据比例计算新的过滤后的vec
	std::vector<diffVec> getSortDiffVec(std::vector<diffVec> inputDiffVec, float ratioFilter);
private:

	std::string _strImageFile1Name32bit;				//32位图像1名称
	std::string _strImageFile2Name32bit;				//32位图像2名称
	tifParameter _tifParameter1;						//第一幅.tif图像的参数
	tifParameter _tifParameter2;						//第二幅.tif图像的参数
	int _widthRoi;										//指定分块的宽度
	int _heightRoi;										//指定分块的高度

	//sift处理部分
	boost::shared_ptr<pcl::Correspondences> _cor_inliers_ptr; //对应点对，用以粗配准
	//pcl处理部分

	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud1;	//第一个内点点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud2;	//第二个内点点云
	std::vector<Pt3> _corlinerPointVec1InPCL;			// 第一幅源图像的PCL三维内点序列
	std::vector<Pt3> _corlinerPointVec2InPCL;			// 第二幅源图像的PCL三维内点序列
	Eigen::Matrix4f _roughMatrix;

};

