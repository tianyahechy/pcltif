
#define _CRT_SECURE_NO_WARNINGS
#include "common.h"
#include <pcl/keypoints/harris_2d.h>

class HarrisDetector
{
private:
	//32λ�������͵Ľǵ�ǿ��ͼ��
	cv::Mat cornerStrength;
	//32λ�������͵���ֵ���ǵ�ͼ��
	cv::Mat cornerTh;
	//�ֲ����ֵͼ��(�ڲ�)
	cv::Mat localMax;
	//ƽ������������ߴ�
	int neighbourhood;
	//�ݶȼ���Ŀھ�
	int aperture;
	//Harris����
	double k;
	//��ֵ��������ǿ��
	double maxStrength;
	//����õ�����ֵ(�ڲ���
	double threshold;
	//�����ֵ���Ƶ�����ߴ�
	int nonMaxSize;
	//�����ֵ���Ƶ��ں�
	cv::Mat kernel;

public:
	HarrisDetector();
	~HarrisDetector();

	void setLocalMaxWindowSize(int size);
	//����harris�ǵ�
	void detect(const cv::Mat& image);
	//��Harrisֵ�õ��ǵ�ֲ�ͼ
	cv::Mat getCornerMap(double qualityLevel);
	//��Harrisֵ�õ��ǵ�ֲ�ͼ
	void getCorners(std::vector<cv::Point> & points, double qualityLevel);
	
};

HarrisDetector::HarrisDetector()
{
	neighbourhood = 3;
	aperture = 3;
	k = 0.01;
	maxStrength = 0;
	threshold = 0.01;
	nonMaxSize = 3;

	//�������ڷ����ֵ���Ƶ��ں�
	//setlocalwindo

}

HarrisDetector::~HarrisDetector()
{
}

void HarrisDetector::setLocalMaxWindowSize(int size)
{
	nonMaxSize = size;
	kernel.create(nonMaxSize, nonMaxSize, CV_8U);
}

//����Harris�ǵ�
void HarrisDetector::detect(const cv::Mat& image)
{
	//Harris����
	cv::cornerHarris(image, cornerStrength, neighbourhood, aperture, k);
	//�ڲ���ֵ����
	cv::minMaxLoc(cornerStrength, 0, &maxStrength);
	//����ֲ����ֵ
	cv::Mat dilated;	//��ʱͼ��
	cv::dilate(cornerStrength, dilated, cv::Mat());
	cv::compare(cornerStrength, dilated, localMax, cv::CMP_EQ);

}

//��Harrisֵ�õ��ǵ�ֲ�ͼ
cv::Mat HarrisDetector::getCornerMap(double qualityLevel)
{
	cv::Mat cornerMap;
	//�Խǵ�ǿ����ֵ��
	threshold = qualityLevel * maxStrength;
	cv::threshold(cornerStrength, cornerTh, threshold, 255, cv::THRESH_BINARY);

	//ת����8λͼ��
	cornerTh.convertTo(cornerMap, CV_8U);

	//�����ֵ����
	cv::bitwise_and(cornerMap, localMax, cornerMap);

	return cornerMap;

}

//��Harrisֵ�õ��ǵ�ֲ�ͼ
void HarrisDetector::getCorners(std::vector<cv::Point> & points, double qualityLevel)
{
	//��ýǵ�ֲ�ͼ
	cv::Mat cornerMap = this->getCornerMap(qualityLevel);
	//��ýǵ�
	this->getCorners()
}
int main(int argc, char *argv[])
{
	//���Harris�ǵ�
	cv::Mat inputImageMat = cv::imread("E:\\logo.scale-100.png");
	//ת��Ϊ�Ҷ�ͼ��
	cv::Mat srcGray;
	cv::cvtColor(inputImageMat, srcGray, CV_32FC1);
	cv::Mat cornerStrength;
	cv::cornerHarris(srcGray, cornerStrength, 3, 3, 0.01);
	cv::imshow("original", inputImageMat);
	cv::imshow("corner", cornerStrength);
	return 0;
}