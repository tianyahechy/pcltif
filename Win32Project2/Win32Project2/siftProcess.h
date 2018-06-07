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
	siftProcess(int widthRoil, int heightRoil,
		std::string strInputTifName1,
		std::string strInputTifName2,
		std::string strInputPCDName1,
		std::string strInputPCDName2);
	~siftProcess();

public:
	//�������в���
	void processAll();

	//��8λ�������еõ�opencvͼ����  ���Ƿ����ֱ����opencv��ȡ8λ.tif)
	cv::Mat getOpenCVImgFrom8bitVec(std::vector<uchar> rastervec8Bit, zone theZone);
	//����opencvͼ���е�SIFT������ƥ��,�ó���Ӧ�����ŵ�vector,
	void getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, cv::Mat srcImage2, 
		std::vector<cv::Point2f>& corlinerPointVec1InOpenCVOfTheZone,
		std::vector<cv::Point2f>& corlinerPointVec2InOpenCVOfTheZone);
	//������С�������
	boost::shared_ptr<pcl::Correspondences> computeMiniCorrisponces( int sizeofColiners);
	//opencv��ά�������vectorתPCL��ά����vector(�ֲ�����
	std::vector<Pt3> convertOpenCV2DVecToPCL3DVec(
		zone theZone,
		tifParameter tifParam,
		std::vector<cv::Point2f> point2dVec, 
		std::vector<float> pixel32BitVec);
	//����id�Ϳ��ȡ������ֵ
	double getPixelFromID(int xID, int yID, int widthRoi, std::vector<float> pixel32BitVec);
	//ͨ����ά��������ת��Ϊ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<Pt3> vec3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFrom3dVec(std::vector<std::vector<Pt3>> vec3);
	//ͨ�����ƺ�ƫ������ת��Ϊ�µ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFromPointCloudAndDeltaXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud,
		double deltaX, double deltaY, double deltaZ);

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
	//��Ӧ��ԴӾֲ�����õ�ָ��tif��ȫ���������Ӧ���
	std::map<int, Pt3> getlinerSetFromTifAndLocalCoordinate(std::string strTifName,
		std::vector<cv::Point2f> localCoordinateVector,
		int xRoil, int yRoil);
	//��32λ��������ת��Ϊ8�������С�
	std::vector<uchar> convert32bitPixelVectorTo8bitPixelVector(std::vector<float> pixelVector32bit);
	//���ݶ���ȥ��Сֵ������������д���׼�ĵ��ƣ��Ծ��������������С�ھ����г˻���Ӱ��
	void ajustVecByMinXYZ(std::vector<Pt3> inputVec1,
		std::vector<Pt3> inputVec2,
		std::vector<Pt3>& outputVec1,
		std::vector<Pt3>& outputVec2,
		double& minX,
		double& minY,
		double& minZ);
	//�������γ��µ��������ڴ���׼�������
	std::vector<Pt3> adjustVecByScale(std::vector<Pt3> inputVec, double scaleX, double scaleY, double scaleZ);
	//���ݶ���ȥ�м�ֵ������������д���׼�ĵ��ƣ��Ծ��������������С�ھ����г˻���Ӱ��
	void ajustVecByMidXYZ(std::vector<Pt3> inputVec1,
		std::vector<Pt3> inputVec2,
		std::vector<Pt3>& outputVec1,
		std::vector<Pt3>& outputVec2,
		double& midX,
		double& midY,
		double& midZ);
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
	//����������ͼ��ģ�xSize,ySize,widthRoi,HeightRoi)ȷ��(xroi,yroi,widthRoitrue,heightRoitrue)
	std::vector<std::vector<zone>> getZoneVecVec(int xSize, int ySize, int widthRoi, int heightRoi);
	std::vector<std::vector<zone>> getZoneVecVec(int xSize, int ySize, float ratioX, float ratioY, 
		int xStart, int yStart, int widthRoi, int heightRoi);
	//���ص�һ�������Ӧ�Ķ�Ӧ�ڵ���ά��������
	void getColinersFromZone(zone theZone1, 
		tifParameter tif1, 
		tifParameter tif2, 
		std::vector<Pt3>& colinersOfTheZone1Vec, 
		std::vector<Pt3>& colinersOfTheZoneVec2);
	//ѡȡһ���ֵ��ƽ��д���
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud, float ratioOfDataSize = 0.4 );
	//�ӵ����еõ�����
	std::vector<Pt3> getVecFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	//��Ŀǰ��԰��ձ�����ȡ�µ�ԡ�
	void filterColiner(std::vector<Pt3> inputVec1, std::vector<Pt3> inputVec2, 
		std::vector<Pt3>& outputVec1, std::vector<Pt3>& outputVec2, 
		float ratioFilter);
	//���ݱ��������µĹ��˺��vec
	std::vector<diffVec> getSortDiffVec(std::vector<diffVec> inputDiffVec, float ratioFilter);
private:

	std::string _strImageFile1Name32bit;				//32λͼ��1����
	std::string _strImageFile2Name32bit;				//32λͼ��2����
	tifParameter _tifParameter1;						//��һ��.tifͼ��Ĳ���
	tifParameter _tifParameter2;						//�ڶ���.tifͼ��Ĳ���
	int _widthRoi;										//ָ���ֿ�Ŀ��
	int _heightRoi;										//ָ���ֿ�ĸ߶�

	//sift������
	boost::shared_ptr<pcl::Correspondences> _cor_inliers_ptr; //��Ӧ��ԣ����Դ���׼
	//pcl������

	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud1;	//��һ���ڵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr _colinerCloud2;	//�ڶ����ڵ����
	std::vector<Pt3> _corlinerPointVec1InPCL;			// ��һ��Դͼ���PCL��ά�ڵ�����
	std::vector<Pt3> _corlinerPointVec2InPCL;			// �ڶ���Դͼ���PCL��ά�ڵ�����
	Eigen::Matrix4f _roughMatrix;

};

