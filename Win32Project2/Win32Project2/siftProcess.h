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
#include <pcl/range_image/range_image.h>  //���ͼ��ͷ�ļ�

//sift����
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
	//�������в���
	void processAll();

	//��8λ�������еõ�opencvͼ����  ���Ƿ����ֱ����opencv��ȡ8λ.tif)
	cv::Mat getOpenCVImgFrom8bitVec(std::vector<uchar> rastervec8Bit);
	//����opencvͼ���е�SIFT������ƥ��,�ó���Ӧ�����ŵ�vector,
	cv::Mat getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, cv::Mat srcImage2 );
	//����opencv�ڵ�ֲ�����Լ������׼���
	boost::shared_ptr<pcl::Correspondences> getCorrisponcesByOpenCVlocalCoordinate(std::vector<cv::Point2f> vec1,
		std::vector<cv::Point2f> vec2);
	//opencvת��pcl���
	boost::shared_ptr<pcl::Correspondences> getCorrisponcesByOpenCVMatcher(std::vector<cv::DMatch>  matchesVectorInOpenCV);
	//������С�������
	boost::shared_ptr<pcl::Correspondences> computeMiniCorrisponces(std::vector<cv::Point2f> firstIDVector, 
		std::vector<cv::Point2f> secondIDVector);
	//opencv��ά�������vectorתPCL��ά����vector
	std::vector<Pt3> convertOpenCV2DVecToPCL3DVec(std::vector<cv::Point2f> point2dVec, std::vector<std::vector<Pt3>> segVec3d);
	//ͨ����ά��������ת��Ϊ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<Pt3> vec3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<std::vector<Pt3>> vec3);

	//�ó�����׼����
	void computeRoughMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
		boost::shared_ptr<pcl::Correspondences> cor);
	//���ֵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr getDiffPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

	//���ݴ���׼����ó�����׼����
	pcl::PointCloud<pcl::PointXYZ>::Ptr getRoughPointCloudFromMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
		Eigen::Matrix4f matrix);
	//opencv��ά�������תPCL��ά����
	Pt3 convertOpenCV2DToPCL3D(cv::Point2f point2d, std::vector<std::vector<Pt3>> segVec3d);
	//��Ӧ��ԴӾֲ�����õ�ָ��tif��ȫ���������Ӧ���
	std::map<int, Pt3> getlinerSetFromTifAndLocalCoordinate(std::string strTifName,
		std::vector<cv::Point2f> localCoordinateVector,
		int xRoil, int yRoil);
	//��32λ��������ת��Ϊ8�������С�
	std::vector<uchar> convert32bitPixelVectorTo8bitPixelVector(std::vector<float> pixelVector32bit);
	//��������д���׼�ĵ��ƣ��Ծ��������������С�ھ����г˻���Ӱ��
	void ajustVec(std::vector<Pt3> inputVec1, 
		std::vector<Pt3> inputVec2,
		std::vector<Pt3>& outputVec1,
		std::vector<Pt3>& outputVec2,
		double& minX,
		double& minY,
		double& minZ);
	//���������vector�������Сx,y,zֵ
	void getMinXYZFromVec(std::vector<Pt3> vec, double& minX, double& minY, double& minZ);
	//��������������
	std::vector<Pt3> getAjustVecFromVecAndDelta(std::vector<Pt3> vec, double minX,double minY,double minZ);
	//�õ�����׼����
	Eigen::Matrix4f ICPRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud);
	//����ƽ�ƻ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudFromAdjustCloudAndMinXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud, 
		double minX, double minY, double minZ);

private:
	int _xRoi1;			//��һ��ͼ�����Ͻ�x����ID
	int _yRoi1;			//��һ��ͼ�����Ͻ�y����ID
	int _widthRoi;		//��һ��ͼ��ȡ�Ŀ��	���ڶ���ͼ������ȣ�Ҳ���ܲ��ȣ�
	int _heightRoi;		//��һ��ͼ��ȡ�ĸ߶�

	//sift������
	std::string _strImageFile1Name8bitForSift;			//sift�����õ�8bitͼ���ļ�1����
	std::string _strImageFile2Name8bitForSift;			//sift�����õ�8bitͼ���ļ�2����
	std::vector<uchar> _rasterID8bitVecForSift1;		//sift����ĵ�һ��8bit����
	std::vector<uchar> _rasterID8bitVecForSift2;		//sift����ĵڶ���8bit����
	std::vector<float> _rasterID32bitVecForSift1;		//sift����ĵ�һ��32bit����
	std::vector<float> _rasterID32bitVecForSift2;		//sift����ĵڶ���32bit����
	std::vector<cv::Point2f> _corlinerPointVec1InOpenCV;// ��һ��Դͼ���OpenCV��ά�ڵ�����
	std::vector<cv::Point2f> _corlinerPointVec2InOpenCV;// �ڶ���Դͼ���OpenCV��ά�ڵ�����
	std::vector<cv::DMatch>  _colinerVectorInOpenCV;	//sift�ڵ�������OpenCV
	boost::shared_ptr<pcl::Correspondences> _cor_inliers_ptr; //��Ӧ��ԣ����Դ���׼
	//pcl������
	std::string _strImageFile1Name32bitForPCL;			//pcl�����32λͼ��1����
	std::string _strImageFile2Name32bitForPCL;			//pcl�����32λͼ��2����
	std::vector<std::vector<Pt3>> _seg1Vector32bitForPCL;//pcl�����32λͼ��1����
	std::vector<std::vector<Pt3>> _seg2Vector32bitForPCL;//pcl�����32λͼ��1����
	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud1;	//��һ���ڵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud2;	//�ڶ����ڵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr _segCloud1;	//��һ����ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr _segCloud2;	//�ڶ�����ȡ����
	std::vector<Pt3> _corlinerPointVec1InPCL;			// ��һ��Դͼ���PCL��ά�ڵ�����
	std::vector<Pt3> _corlinerPointVec2InPCL;			// �ڶ���Դͼ���PCL��ά�ڵ�����
	Eigen::Matrix4f _roughMatrix;

};

