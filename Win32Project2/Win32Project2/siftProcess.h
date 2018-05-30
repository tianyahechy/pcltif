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
	siftProcess(int xRoil, int yRoil, int widthRoil, int heightRoil, 
		std::string strImageFile1Name8bitForSift, 
		std::string strImageFile2Name8bitForSift,
		std::string strImageFile1Name32bitForPCL,
		std::string strImageFile2Name32bitForPCL);
	~siftProcess();

public:
	//处理所有步骤
	void processAll();

	//从8位像素序列得到opencv图像中  （是否可以直接用opencv读取8位.tif)
	cv::Mat getOpenCVImgFrom8bitVec(std::vector<uchar> rastervec8Bit);
	//计算opencv图像中的SIFT特征及匹配,得出对应点对序号的vector,
	cv::Mat getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, cv::Mat srcImage2 );
	//根据opencv内点局部坐标对计算粗配准点对
	boost::shared_ptr<pcl::Correspondences> getCorrisponcesByOpenCVlocalCoordinate(std::vector<cv::Point2f> vec1,
		std::vector<cv::Point2f> vec2);
	//opencv转到pcl点对
	boost::shared_ptr<pcl::Correspondences> getCorrisponcesByOpenCVMatcher(std::vector<cv::DMatch>  matchesVectorInOpenCV);
	//计算最小点云配对
	boost::shared_ptr<pcl::Correspondences> computeMiniCorrisponces(std::vector<cv::Point2f> firstIDVector, 
		std::vector<cv::Point2f> secondIDVector);
	//opencv二维序号坐标vector转PCL三维坐标vector
	std::vector<Pt3> convertOpenCV2DVecToPCL3DVec(std::vector<cv::Point2f> point2dVec, std::vector<std::vector<Pt3>> segVec3d);
	//通过三维坐标序列转换为点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<Pt3> vec3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<std::vector<Pt3>> vec3);

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
	//opencv二维序号坐标转PCL三维坐标
	Pt3 convertOpenCV2DToPCL3D(cv::Point2f point2d, std::vector<std::vector<Pt3>> segVec3d);
	//对应点对从局部坐标得到指定tif的全局坐标和相应序号
	std::map<int, Pt3> getlinerSetFromTifAndLocalCoordinate(std::string strTifName,
		std::vector<cv::Point2f> localCoordinateVector,
		int xRoil, int yRoil);
	//将32位像素序列转换为8像素序列。
	std::vector<uchar> convert32bitPixelVectorTo8bitPixelVector(std::vector<float> pixelVector32bit);
	//计算出进行粗配准的点云，以尽可能消除坐标大小在矩阵中乘积的影响
	void ajustVec(std::vector<Pt3> inputVec1, 
		std::vector<Pt3> inputVec2,
		std::vector<Pt3>& outputVec1,
		std::vector<Pt3>& outputVec2,
		double& minX,
		double& minY,
		double& minZ);
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

private:
	int _xRoi1;			//第一幅图的左上角x方向ID
	int _yRoi1;			//第一幅图的左上角y方向ID
	int _widthRoi;		//第一幅图截取的宽度	（第二副图可能相等，也可能不等）
	int _heightRoi;		//第一幅图截取的高度

	//sift处理部分
	std::string _strImageFile1Name8bitForSift;			//sift处理用的8bit图像文件1名称
	std::string _strImageFile2Name8bitForSift;			//sift处理用的8bit图像文件2名称
	std::vector<uchar> _rasterID8bitVecForSift1;		//sift处理的第一幅8bit序列
	std::vector<uchar> _rasterID8bitVecForSift2;		//sift处理的第二幅8bit序列
	std::vector<float> _rasterID32bitVecForSift1;		//sift处理的第一幅32bit序列
	std::vector<float> _rasterID32bitVecForSift2;		//sift处理的第二幅32bit序列
	std::vector<cv::Point2f> _corlinerPointVec1InOpenCV;// 第一幅源图像的OpenCV二维内点序列
	std::vector<cv::Point2f> _corlinerPointVec2InOpenCV;// 第二幅源图像的OpenCV二维内点序列
	std::vector<cv::DMatch>  _colinerVectorInOpenCV;	//sift内点点对序列OpenCV
	boost::shared_ptr<pcl::Correspondences> _cor_inliers_ptr; //对应点对，用以粗配准
	//pcl处理部分
	std::string _strImageFile1Name32bitForPCL;			//pcl处理的32位图像1名称
	std::string _strImageFile2Name32bitForPCL;			//pcl处理的32位图像2名称
	std::vector<std::vector<Pt3>> _seg1Vector32bitForPCL;//pcl处理的32位图像1序列
	std::vector<std::vector<Pt3>> _seg2Vector32bitForPCL;//pcl处理的32位图像1序列
	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud1;	//第一个内点点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud2;	//第二个内点点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr _segCloud1;	//第一个截取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr _segCloud2;	//第二个截取点云
	std::vector<Pt3> _corlinerPointVec1InPCL;			// 第一幅源图像的PCL三维内点序列
	std::vector<Pt3> _corlinerPointVec2InPCL;			// 第二幅源图像的PCL三维内点序列
	Eigen::Matrix4f _roughMatrix;

};

