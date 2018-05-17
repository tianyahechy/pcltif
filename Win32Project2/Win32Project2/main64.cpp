
#define _CRT_SECURE_NO_WARNINGS
#include "common.h"
#include <pcl/keypoints/harris_2d.h>

class HarrisDetector
{
private:
	//32位浮点数型的角点强度图像
	cv::Mat cornerStrength;
	//32位浮点数型的阙值化角点图像
	cv::Mat cornerTh;
	//局部最大值图像(内部)
	cv::Mat localMax;
	//平滑导数的邻域尺寸
	int neighbourhood;
	//梯度计算的口径
	int aperture;
	//Harris参数
	double k;
	//阙值计算的最大强度
	double maxStrength;
	//计算得到的阈值(内部）
	double threshold;
	//非最大值抑制的邻域尺寸
	int nonMaxSize;
	//非最大值抑制的内核
	cv::Mat kernel;

public:
	HarrisDetector();
	~HarrisDetector();

	void setLocalMaxWindowSize(int size);
	//计算harris角点
	void detect(const cv::Mat& image);
	//用Harris值得到角点分布图
	cv::Mat getCornerMap(double qualityLevel);
	//用Harris值得到角点分布图
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

	//创建用于非最大值抑制的内核
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

//计算Harris角点
void HarrisDetector::detect(const cv::Mat& image)
{
	//Harris计算
	cv::cornerHarris(image, cornerStrength, neighbourhood, aperture, k);
	//内部阈值计算
	cv::minMaxLoc(cornerStrength, 0, &maxStrength);
	//计算局部最大值
	cv::Mat dilated;	//临时图像
	cv::dilate(cornerStrength, dilated, cv::Mat());
	cv::compare(cornerStrength, dilated, localMax, cv::CMP_EQ);

}

//用Harris值得到角点分布图
cv::Mat HarrisDetector::getCornerMap(double qualityLevel)
{
	cv::Mat cornerMap;
	//对角点强度阈值化
	threshold = qualityLevel * maxStrength;
	cv::threshold(cornerStrength, cornerTh, threshold, 255, cv::THRESH_BINARY);

	//转换成8位图像
	cornerTh.convertTo(cornerMap, CV_8U);

	//非最大值抑制
	cv::bitwise_and(cornerMap, localMax, cornerMap);

	return cornerMap;

}

//用Harris值得到角点分布图
void HarrisDetector::getCorners(std::vector<cv::Point> & points, double qualityLevel)
{
	//获得角点分布图
	cv::Mat cornerMap = this->getCornerMap(qualityLevel);
	//获得角点
	this->getCorners()
}
int main(int argc, char *argv[])
{
	//检测Harris角点
	cv::Mat inputImageMat = cv::imread("E:\\logo.scale-100.png");
	//转换为灰度图像
	cv::Mat srcGray;
	cv::cvtColor(inputImageMat, srcGray, CV_32FC1);
	cv::Mat cornerStrength;
	cv::cornerHarris(srcGray, cornerStrength, 3, 3, 0.01);
	cv::imshow("original", inputImageMat);
	cv::imshow("corner", cornerStrength);
	return 0;
}