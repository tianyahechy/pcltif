#include "siftProcess.h"
#include "math.h"
#include <algorithm>
siftProcess::siftProcess(int widthRoil, int heightRoil,
	std::string strInputTifName1,
	std::string strInputTifName2,
	std::string strInputPCDName1,
	std::string strInputPCDName2)
{
	_widthRoi = widthRoil;
	_heightRoi = heightRoil;
	_strImageFile1Name32bit = strInputTifName1;
	_strImageFile2Name32bit = strInputTifName2;
	_strInputPCDName1 = strInputPCDName1;
	_strInputPCDName2 = strInputPCDName2;
	util::getTifParameterFromTifName(_strImageFile1Name32bit, _tifParameter1);
	util::getTifParameterFromTifName(_strImageFile2Name32bit, _tifParameter2);

	_roughMatrix.Identity();

	_corlinerPointVec1InPCL.clear();
	_corlinerPointVec2InPCL.clear();
	if (_colinerCloud1)
	{
		_colinerCloud1->clear();
	}
	if (_colinerCloud2)
	{
		_colinerCloud2->clear();
	}
}

siftProcess::~siftProcess()
{
	if (_colinerCloud1)
	{
		_colinerCloud1->clear();
	}
	if (_colinerCloud2)
	{
		_colinerCloud2->clear();
	}
	_corlinerPointVec1InPCL.clear();
	_corlinerPointVec2InPCL.clear();
}

//处理所有步骤
void siftProcess::processAll()
{
	time_t startTime, endTime;
	time(&startTime);
	//0，根据.tif名称得到该.tif的各要素
	int xSize1 = _tifParameter1.xSize;
	int ySize1 = _tifParameter1.ySize;
	float ratioX = 0.5;
	float ratioY = 0.5;
	int startX = 0.3 * xSize1;
	int startY = 0.3 * ySize1;

	//1,分区，根据图像的（xSize,ySize,widthRoi,HeightRoi)确定(xroi,yroi,widthRoitrue,heightRoitrue)
	std::vector<std::vector<zone>> zoneVecVec1 = this->getZoneVecVec(xSize1, ySize1, ratioX, ratioY, startX, startY, _widthRoi, _heightRoi);
	//对于每个区域，根据第一幅图的位置计算第二幅图像的起点和宽高
	std::vector<Pt3> coliners1InOpenCV;
	coliners1InOpenCV.clear();
	std::vector<Pt3> coliners2InOpenCV;
	coliners2InOpenCV.clear();
	//核心粗配准矩阵Vector
	std::vector<Eigen::Matrix4f> coreRoughMatrixVec;
	coreRoughMatrixVec.clear();

	for (size_t j = 0; j < zoneVecVec1.size(); j++)
	{
		for (size_t i = 0; i < zoneVecVec1[j].size(); i++)
		{
			zone theZone1 = zoneVecVec1[j][i];
			
			//返回第一幅图像各块的对应点对的三维坐标
			std::vector<Pt3> colinersOfTheZone1Vec;
			colinersOfTheZone1Vec.clear();
			std::vector<Pt3> colinersOfTheZone2Vec;
			colinersOfTheZone2Vec.clear();

			this->getColinersFromZone(theZone1, _tifParameter1, _tifParameter2, colinersOfTheZone1Vec, colinersOfTheZone2Vec);

			//如果点对大小为0，则跳出
			int sizeofColiner1Vec = colinersOfTheZone1Vec.size();
			int sizeofColiner2Vec = colinersOfTheZone2Vec.size();
			if (sizeofColiner1Vec == 0 ||
				sizeofColiner2Vec == 0 )
			{
				continue;
			}

			//将所有内点的三维坐标加入对应点对中(第一条路，先内点集合再计算矩阵）
			for (size_t k = 0; k < colinersOfTheZone1Vec.size(); k++)
			{
				coliners1InOpenCV.push_back(colinersOfTheZone1Vec[k]);
				coliners2InOpenCV.push_back(colinersOfTheZone2Vec[k]);
			}

			//第二条路，先计算矩阵再拟和
			//1,根据比例再过滤一次点对
			float ratioFilter = 0.3;
			std::vector<Pt3>filterColinerPt3Vec1;
			filterColinerPt3Vec1.clear();
			std::vector<Pt3> filterColinerPt3Vec2;
			filterColinerPt3Vec2.clear();
			this->filterColiner(colinersOfTheZone1Vec, colinersOfTheZone2Vec, filterColinerPt3Vec1, filterColinerPt3Vec2, ratioFilter);
			colinersOfTheZone1Vec.clear();
			colinersOfTheZone2Vec.clear();
			//根据点对来计算核心粗配准矩阵
			Eigen::Matrix4f coreRoughMatrix = this->getCoreRoughMatrixFromColinerVec(filterColinerPt3Vec1, filterColinerPt3Vec2);
			coreRoughMatrixVec.push_back(coreRoughMatrix);

		}
	}

	//1,根据比例再过滤一次点对
	float ratioFilter = 0.3;
	std::vector<Pt3>filterColinerPt3Vec1;
	filterColinerPt3Vec1.clear();
	std::vector<Pt3> filterColinerPt3Vec2;
	filterColinerPt3Vec2.clear();
	this->filterColiner(coliners1InOpenCV, coliners2InOpenCV, filterColinerPt3Vec1, filterColinerPt3Vec2, ratioFilter);
	//第一种办法计算核心粗配准矩阵
	Eigen::Matrix4f firstRoughMatrix = this->getCoreRoughMatrixFromColinerVec(filterColinerPt3Vec1, filterColinerPt3Vec2);
	//第二种计算拟和矩阵
	Eigen::Matrix4f secondRoughMatrix = this->getAverageMatrix(coreRoughMatrixVec);
	//对两个点云粗配准，并计算出差点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud = this->getPointCloudFrom3dVec(filterColinerPt3Vec1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr destCloud = this->getPointCloudFrom3dVec(filterColinerPt3Vec2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr firstDiffCloud = this->getDiffCloudFromSrcCloudAndDestCloudAndCoreRoughMatrix(srcCloud,
		destCloud,
		firstRoughMatrix);
	pcl::io::savePCDFile("E:\\test\\secondDiffCloud.pcd", *firstDiffCloud);
	firstDiffCloud->clear();

	pcl::PointCloud<pcl::PointXYZ>::Ptr secondDiffCloud = this->getDiffCloudFromSrcCloudAndDestCloudAndCoreRoughMatrix(srcCloud,
		destCloud,
		secondRoughMatrix);
	pcl::io::savePCDFile("E:\\test\\secondDiffCloud.pcd", *secondDiffCloud);
	secondDiffCloud->clear();

	//输出粗配准矩阵
	std::cout << "第一种粗配准核心矩阵" << std::endl;
	std::cout << firstRoughMatrix << std::endl;
	std::cout << "第二种粗配准核心矩阵" << std::endl;
	std::cout << secondRoughMatrix << std::endl;
	// 测试核矩阵
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read(_strInputPCDName1, *cloud1);
	reader.read(_strInputPCDName2, *cloud2);

	//8，清理
	cloud1->clear();
	cloud2->clear();
	coreRoughMatrixVec.clear();
}

//对两个点云粗配准，并计算出差点云
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getDiffCloudFromSrcCloudAndDestCloudAndCoreRoughMatrix(
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr destCloud,
	Eigen::Matrix4f coreRoughMatrix)
{
	std::vector<Pt3> srcCloudVec = this->getVecFromCloud(srcCloud);
	std::vector<Pt3> destCloudVec = this->getVecFromCloud(destCloud);
	//将源中心点至原点，目标平移同样距离
	//计算源内点点云中心点
	double midSrcCloudCoordX = 0;
	double midSrcCloudCoordY = 0;
	double midSrcCloudCoordZ = 0;
	this->getMidPointOfTheVec(srcCloudVec, midSrcCloudCoordX, midSrcCloudCoordY, midSrcCloudCoordZ);

	//平移源点云至原点
	std::vector<Pt3> transformSrcCloudVec = this->getAjustVecFromVecAndDelta(srcCloudVec, midSrcCloudCoordX, midSrcCloudCoordY, midSrcCloudCoordZ);
	std::vector<Pt3> transformDestCloudVec = this->getAjustVecFromVecAndDelta(destCloudVec, midSrcCloudCoordX, midSrcCloudCoordY, midSrcCloudCoordZ);
	//从序列计算得到两个点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformSrcCloud = this->getPointCloudFrom3dVec(transformSrcCloudVec);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformDestCloud = this->getPointCloudFrom3dVec(transformDestCloudVec);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformRoughCloud = this->getRoughPointCloudFromMatrix(transformSrcCloud, coreRoughMatrix);
	//平移回去
	pcl::PointCloud<pcl::PointXYZ>::Ptr roughCloud = this->getCloudFromAdjustCloudAndMinXYZ(transformRoughCloud,
		midSrcCloudCoordX,
		midSrcCloudCoordY,
		midSrcCloudCoordZ);
	//计算差值点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr diffCloud = this->getDiffPointCloud(roughCloud, destCloud);
	srcCloudVec.clear();
	destCloudVec.clear();
	transformSrcCloudVec.clear();
	transformDestCloudVec.clear();
	transformSrcCloud->clear();
	transformDestCloud->clear();
	transformRoughCloud->clear();
	roughCloud->clear();

	return diffCloud;

}
//计算拟和矩阵
Eigen::Matrix4f siftProcess::getAverageMatrix(std::vector<Eigen::Matrix4f> matrixVector)
{
	Eigen::Matrix4f averageMatrix;
	averageMatrix.setZero();
	Eigen::Matrix4f totalMatrix;
	totalMatrix.setZero();
	//1，判断矩阵是否为空，空则返回0
	int sizeOfMatrixVector = matrixVector.size();
	if (sizeOfMatrixVector == 0 )
	{
		averageMatrix.setZero();
		return averageMatrix;
	}
	//2，遍历各个矩阵，相加
	for (size_t i = 0; i < sizeOfMatrixVector; i++)
	{
		Eigen::Matrix4f theMatrix = matrixVector[i];
		totalMatrix += theMatrix;
	}
	//3，平均
	averageMatrix = totalMatrix / sizeOfMatrixVector;
	//4，返回
	return averageMatrix;
}
//根据点对来计算核心粗配准矩阵
Eigen::Matrix4f siftProcess::getCoreRoughMatrixFromColinerVec(std::vector<Pt3> colinerPt3Vec1,
	std::vector<Pt3> colinerPt3Vec2 )
{
	//1,计算点对
	int sizeofColiner = colinerPt3Vec1.size();
	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr = this->computeMiniCorrisponces(sizeofColiner);
	std::cout << "cor_inliers_ptr个数:" << cor_inliers_ptr->size() << std::endl;
	//2，使用pcl的算法得出粗配准矩阵
	//将源中心点至原点，目标平移同样距离，计算核旋转矩阵，（此时同名点对基本重合）再平移源中点原点这么多距离，到目标点云的位置
	//计算源内点点云中心点
	double midCoordX1 = 0;
	double midCoordY1 = 0;
	double midCoordZ1 = 0;
	this->getWeightedmeanMidPointOfTheVec(colinerPt3Vec1, midCoordX1, midCoordY1, midCoordZ1);
	//5将源中心点至原点，目标平移同样距离
	std::vector<Pt3> transformColiner1Vec = this->getAjustVecFromVecAndDelta(colinerPt3Vec1, midCoordX1, midCoordY1, midCoordZ1);
	std::vector<Pt3> transformColiner2Vec = this->getAjustVecFromVecAndDelta(colinerPt3Vec2, midCoordX1, midCoordY1, midCoordZ1);
	//6从序列计算得到两个点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformColinerCloud1 = this->getPointCloudFrom3dVec(transformColiner1Vec);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformColinerCloud2 = this->getPointCloudFrom3dVec(transformColiner2Vec);
	colinerPt3Vec1.clear();
	colinerPt3Vec2.clear();
	//7计算两个内点序列中心坐标，计算核心矩阵
	Eigen::Matrix4f coreRoughtMatrix = this->computeRoughMatrix(transformColinerCloud1, transformColinerCloud2, cor_inliers_ptr);
	cor_inliers_ptr->clear();
	transformColinerCloud1->clear();
	transformColinerCloud2->clear();
	transformColiner1Vec.clear();
	transformColiner2Vec.clear();

	return coreRoughtMatrix;
}
//根据粗配准矩阵得出粗配准点云
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getRoughPointCloudFromMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	Eigen::Matrix4f transformation_matrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = inputCloud->points.size();
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Raw转换前个数：" << cloud_in_filtered_Raw->size() << std::endl;
	pcl::transformPointCloud(*inputCloud, *cloud_in_filtered_Raw, transformation_matrix);
	std::cout << "cloud_in_filtered_Raw转换后个数：" << cloud_in_filtered_Raw->size() << std::endl;

	return cloud_in_filtered_Raw;
}

//二维序号坐标vector转三维坐标vector(局部区域）
std::vector<Pt3> siftProcess::convertOpenCV2DVecToPCL3DVec(
	zone theZone, 
	tifParameter tifParam,
	std::vector<cv::Point2f> point2dVec, 
	std::vector<float> pixel32BitVec)
{
	std::vector<Pt3> pcl3DVec;
	pcl3DVec.clear();
	//1,首先判断二维序列和三维序列是否为空
	if (point2dVec.size() == 0 || pixel32BitVec.size() == 0)
	{
		pcl3DVec.clear();
		return pcl3DVec;
	}

	//2，将二维序列中的数据转化为pt3
	int xRoi = theZone.xRoi;
	int yRoi = theZone.yRoi;
	int widthRoi = theZone.widthRoi;
	double leftTopX = tifParam.leftTopX;
	double leftTopY = tifParam.leftTopY;
	double xResolution = tifParam.xResolution;
	double yResolution = tifParam.yResolution;
	for (int i = 0; i < point2dVec.size(); i++)
	{
		//将每个cv::Point2f转换为Pt3
		int deltaXID = point2dVec[i].x;
		int deltaYID = point2dVec[i].y;
		int xID = xRoi + deltaXID;
		int yID = yRoi + deltaYID;
		double x = leftTopX + xID * xResolution;
		double y = leftTopY + yID * yResolution;
		double z = this->getPixelFromID(deltaXID, deltaYID, widthRoi, pixel32BitVec);
		
		Pt3 thePoint3(x, y, z);
		pcl3DVec.push_back(thePoint3);
	}

	return pcl3DVec;
}

//计算图像中的SIFT特征及匹配,得出对应点对序号的vector,
void siftProcess::getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, 
	cv::Mat srcImage2,
	std::vector<cv::Point2f>& corlinerPointVec1InOpenCVOfTheZone,
	std::vector<cv::Point2f>& corlinerPointVec2InOpenCVOfTheZone)
{
	//定义sift描述子
	//cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create(0, 6, 0.08, 15, 1.0);
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

	//查找关键点
	std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
	f2d->detect(srcImage1, keyPoints_1);
	f2d->detect(srcImage2, keyPoints_2);

	//如果关键点个数为0，则不匹配 
	int sizeofKeyPoint1 = keyPoints_1.size();
	int sizeofKeyPoint2 = keyPoints_2.size();
	if (sizeofKeyPoint1 == 0 || sizeofKeyPoint2 == 0 )
	{
		return;
	}
	
	//计算描述子
	cv::Mat descriptor_1, descriptor_2;
	f2d->compute(srcImage1, keyPoints_1, descriptor_1);
	f2d->compute(srcImage2, keyPoints_2, descriptor_2);

	//特征点匹配
	cv::FlannBasedMatcher matcher;
	std::vector<std::vector<cv::DMatch>> matchesVectorVector;
	//matcher.match(descriptor_1, descriptor_2, matchesVector);
	//测试一定距离的匹配
	int k = 2;
	matcher.knnMatch(descriptor_1, descriptor_2, matchesVectorVector, k);
	std::vector<cv::DMatch>  matchesVectorInOpenCV;
	//double ratio = 0.9;
	std::vector<std::vector<cv::DMatch>>::iterator
		iterCur = matchesVectorVector.begin(),
		iterEnd = matchesVectorVector.end();
	for (; iterCur != iterEnd; iterCur++)
	{
		std::vector<cv::DMatch> theMatch = *iterCur;
		double minDistance = theMatch[0].distance;
		int minID = 0;
		for (size_t i = 0; i < k; i++)
		{
			double theDistance = theMatch[i].distance;
			if ( theDistance < minDistance)
			{
				minID = i;
			}		
		}
		matchesVectorInOpenCV.push_back(theMatch[minID]);
	}
	//计算最大最小值
	double minDist = matchesVectorInOpenCV[0].distance;
	double maxDist = matchesVectorInOpenCV[0].distance;
	for (size_t i = 0; i < matchesVectorInOpenCV.size(); i++)
	{
		double theDistance = matchesVectorInOpenCV[i].distance;
		if (theDistance < minDist)
		{
			minDist = theDistance;
		}
		if ( theDistance > maxDist)
		{
			maxDist = theDistance;
		}
	}

	std::cout << "mindist = " << minDist << ",maxDist=" << maxDist << std::endl;

	double ratio = 0.9;
	double standardDist = minDist + (maxDist - minDist) * ratio;
	//输出匹配结果
	for (size_t i = 0; i < matchesVectorInOpenCV.size(); i++)
	{
		int idFirst = matchesVectorInOpenCV[i].queryIdx;
		int idSecond = matchesVectorInOpenCV[i].trainIdx;
		cv::Point2f ptFirst = keyPoints_1[idFirst].pt;
		float firstPtX = ptFirst.x;
		float firstPtY = ptFirst.y;
		cv::Point2f ptSecond = keyPoints_2[idSecond].pt;
		float secondPtX = ptSecond.x;
		float secondPtY = ptSecond.y;
		float diffX = ptSecond.x - ptFirst.x;
		float diffY = ptSecond.y - ptFirst.y;
		//如果相似度<最大相似度距离的1/3,则输出sift点
		float theDistance = matchesVectorInOpenCV[i].distance;
		bool bDistance = theDistance < standardDist;  //判断相似度是否合适
		bool bWindowSizeFitX = (diffX < -22) && (diffX > -32 );		//判断过滤窗口x大小是否合适
		bool bWindowSizeFitY = (diffY < 2) && (diffY > -8);		//判断过滤窗口y大小是否合适

		if (bDistance && bWindowSizeFitX && bWindowSizeFitY)
		{	
			//内点
			corlinerPointVec1InOpenCVOfTheZone.push_back(ptFirst);
			corlinerPointVec2InOpenCVOfTheZone.push_back(ptSecond);
			
		}
	
	}


}
//对应点对从局部坐标得到指定tif的全局坐标和相应序号
std::map<int, Pt3> siftProcess::getlinerSetFromTifAndLocalCoordinate(std::string strTifName, std::vector<cv::Point2f> localCoordinateVector, int xRoil, int yRoil)
{
	std::vector<Pt2> globalIDVector;
	globalIDVector.clear();

	for (size_t i = 0; i < localCoordinateVector.size(); i++)
	{
		int deltaXID = localCoordinateVector[i].x;
		int deltaYID = localCoordinateVector[i].y;
		int xID = xRoil + deltaXID;
		int yID = yRoil + deltaYID;
		Pt2 thePt(xID, yID);
		globalIDVector.push_back(thePt);
	}
	//从.tif文件得到相应的vector，看看
	int xSize = 0;
	int ySize = 0;
	double xResolution = 0;
	double yResolution = 0;
	double topLeftX = 0;
	double topLeftY = 0;
	std::vector<std::vector<Pt3>> rastervec = util::getRasterVecVecFromTif(strTifName.c_str(),
		xSize, ySize,
		xResolution, yResolution,
		topLeftX, topLeftY);

	//赋值（序号，坐标，高程）
	std::map<int, Pt3> selectTifSet;
	selectTifSet.clear();
	for (size_t i = 0; i < globalIDVector.size(); i++)
	{
		int xID = globalIDVector[i].x();
		int yID = globalIDVector[i].y();
		int ID = xID + yID * xSize;
		double x = rastervec[yID][xID].x();
		double y = rastervec[yID][xID].y();
		double z = rastervec[yID][xID].z();
		Pt3 thePt(x, y, z);
		selectTifSet.insert(std::pair<int, Pt3>(ID, thePt));
	}

	globalIDVector.clear();
	rastervec.clear();
	return selectTifSet;
}
//从8位像素序列得到opencv图像中  （是否可以直接用opencv读取8位.tif?)
cv::Mat siftProcess::getOpenCVImgFrom8bitVec(std::vector<uchar> rastervec8Bit, zone theZone)
{
	int heightRoi = theZone.heightRoi;
	int widthRoi = theZone.widthRoi;
	cv::Mat srcImage(heightRoi, widthRoi, CV_8UC1);;
	for (int i = 0; i < heightRoi; i++)
	{
		for (int j = 0; j < widthRoi; j++)
		{
			srcImage.at<uchar>(i, j) = rastervec8Bit[i * widthRoi + j];
		}
	}
	return srcImage;
}
//计算粗配准矩阵
Eigen::Matrix4f siftProcess::computeRoughMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, 
	boost::shared_ptr<pcl::Correspondences> cor)
{
	Eigen::Matrix4f theMatrix;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_east;
	trans_east.estimateRigidTransformation(*cloud1, *cloud2, *cor, theMatrix);
	return theMatrix;
}

//通过三维坐标序列转换为点云
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getPointCloudFrom3dVec(std::vector<Pt3> vec3)
{
	//写点云(局部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->clear();
	cloud->width = vec3.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);

	for (size_t i = 0; i < vec3.size(); i++)
	{
	
		Pt3 thetifPt = vec3[i];
		double x = thetifPt.x();
		double y = thetifPt.y();
		double z = thetifPt.z();

		pcl::PointXYZ thePt;
		thePt.x = x;
		thePt.y = y;
		thePt.z = z;
		cloud->points[i] = thePt;

	}
	return cloud;
}
//通过点云和偏移坐标转换为新点云
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getPointCloudFromPointCloudAndDeltaXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud,
	double deltaX, double deltaY, double deltaZ)
{
	//写点云(局部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->clear();
	cloud->width = originalCloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->size(); i++)
	{

		double x = originalCloud->points[i].x - deltaX;
		double y = originalCloud->points[i].y - deltaY;
		double z = originalCloud->points[i].z - deltaZ;

		pcl::PointXYZ thePt;
		thePt.x = x;
		thePt.y = y;
		thePt.z = z;
		cloud->points[i] = thePt;

	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getPointCloudFrom3dVec(std::vector<std::vector<Pt3>> vec3)
{
	//写点云(局部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->clear();
	cloud->width = vec3.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);

	for (size_t j = 0; j < vec3.size(); j++)
	{
		for (int i = 0; i < vec3[j].size(); i++)
		{
			Pt3 thetifPt = vec3[j][i];
			double x = thetifPt.x();
			double y = thetifPt.y();
			double z = thetifPt.z();

			pcl::PointXYZ thePt;
			thePt.x = x;
			thePt.y = y;
			thePt.z = z;
			cloud->push_back(thePt);
		}
	}
	return cloud;
}

//计算最小点云配对
boost::shared_ptr<pcl::Correspondences> siftProcess::computeMiniCorrisponces(int sizeofColiner)
{
	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);
	//赋值给内点
	for (size_t i = 0; i < sizeofColiner; i++)
	{
		pcl::Correspondence theCorr;
		theCorr.index_query = i;
		theCorr.index_match = i; 
		cor_inliers_ptr->push_back(theCorr);
	}
	return cor_inliers_ptr;
}

//求差值点云
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getDiffPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
	int minSize = cloud1->points.size();
	if ( minSize > cloud2->points.size() )
	{
		minSize = cloud2->points.size();
	}
	//写点云(局部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->clear();
	cloud->width = minSize;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);

	for (size_t i = 0; i < minSize; i++)
	{

		pcl::PointXYZ thePt;
		thePt.x = cloud1->points[i].x - cloud2->points[i].x;
		thePt.y = cloud1->points[i].y - cloud2->points[i].y;
		thePt.z = cloud1->points[i].z - cloud2->points[i].z;
		cloud->points[i] = thePt;

	}
	return cloud;

}

//将32位像素序列转换为8像素序列。
std::vector<uchar> siftProcess::convert32bitPixelVectorTo8bitPixelVector(std::vector<float> pixelVector32bit)
{
	//0,初始化一个空的8位像素序列
	std::vector<uchar> pixelVector8bit;
	pixelVector8bit.clear();

	//1，判断32位像素序列是否为空，空则返回
	int sizeof32Bit = pixelVector32bit.size();
	if (sizeof32Bit == 0)
	{
		pixelVector8bit.clear();
		return pixelVector8bit;
	}

	//2，计算32位像素序列的最大值和最小值
	double maxPixel32bit = pixelVector32bit[0];
	double minPixel32bit = pixelVector32bit[0];
	for (int i = 0; i < sizeof32Bit; i++)
	{
		double thePixel32bit = pixelVector32bit[i];
		if (minPixel32bit > thePixel32bit)
		{
			minPixel32bit = thePixel32bit;
		}

		if (maxPixel32bit < thePixel32bit)
		{
			maxPixel32bit = thePixel32bit;
		}
	}
	std::cout << "minPixel32bit = " << minPixel32bit << ",maxPixel32bit = " << maxPixel32bit << std::endl;
	//3，如果最小值和最大值相等，则设定为原值
	if (minPixel32bit == maxPixel32bit)
	{
		int thePixel8bit = minPixel32bit;
		if ( thePixel8bit > 255 )
		{
			thePixel8bit = 255;
		}
		for (size_t i = 0; i < sizeof32Bit; i++)
		{
			pixelVector8bit.push_back(thePixel8bit);
		}
		return pixelVector8bit;
	}

	//4，将32位像素序列的最大值和最小值范围映射到0-255
	double lengthOf32bit = maxPixel32bit - minPixel32bit;

	//5，将32位像素序列的所有值在0-255按比例取值
	for (size_t i = 0; i < sizeof32Bit; i++)
	{
		double thePixel32bit = pixelVector32bit[i];
		//32位偏移量是
		double deltaLength32bit = thePixel32bit - minPixel32bit;
		//计算换算出的8bit像素
		int thePixel8bit = 255 * deltaLength32bit/lengthOf32bit;
		if (thePixel8bit > 255)
		{
			thePixel8bit = 255;
		}

		//5，将得到的各个8位像素放入序列
		pixelVector8bit.push_back(thePixel8bit);
	}
	//6，返回8位像素序列
	return pixelVector8bit;

}

//放缩后形成新的序列用于粗配准矩阵调整
std::vector<Pt3> siftProcess::adjustVecByScale(std::vector<Pt3> inputVec, double scaleX, double scaleY, double scaleZ)
{
	std::vector<Pt3> scaleVec;
	scaleVec.clear();
	for (size_t i = 0; i < inputVec.size(); i++)
	{
		Pt3 thePt = inputVec[i];
		float theX = thePt.x() * scaleX;
		float theY = thePt.y() * scaleY;
		float theZ = thePt.z() * scaleZ;
		Pt3 newPt(theX, theY, theZ);
		scaleVec.push_back(newPt);

	}
	return scaleVec;
}
//根据都减去最小值，来计算出进行粗配准的点云，以尽可能消除坐标大小在矩阵中乘积的影响
void siftProcess::ajustVecByMinXYZ(std::vector<Pt3> inputVec1,
	std::vector<Pt3> inputVec2,
	std::vector<Pt3>& adjustVec1,
	std::vector<Pt3>& adjustVec2,
	double& minX,
	double& minY,
	double& minZ)
{
	//0，先判断两个输入序列是否为空
	int sizeofVec1 = inputVec1.size();
	int sizeofVec2 = inputVec2.size();
	if ( sizeofVec1 == 0 || sizeofVec2 == 0)
	{
		return;
	}

	//1,分别计算两个序列中的最小值
	double minX1 = 0;
	double minY1 = 0;
	double minZ1 = 0;
	this->getMinXYZFromVec(inputVec1, minX1, minY1, minZ1);

	double minX2 = 0;
	double minY2 = 0;
	double minZ2 = 0;
	this->getMinXYZFromVec(inputVec2, minX2, minY2, minZ2);
	//2,从两个序列中的最小值得出更小的值
	minX = minX1;
	if (minX2 < minX)
	{
		minX = minX2;
	}
	minY = minY1;
	if (minY2 < minY)
	{
		minY = minY2;
	}
	minZ = minZ1;
	if ( minZ2 < minZ)
	{
		minZ = minZ2;
	}
	//3,得出调整后的序列
	adjustVec1 = this->getAjustVecFromVecAndDelta(inputVec1, minX, minY, minZ);
	adjustVec2 = this->getAjustVecFromVecAndDelta(inputVec2, minX, minY, minZ);
	
}

//根据都减去中间值，来计算出进行粗配准的点云，以尽可能消除坐标大小在矩阵中乘积的影响
void siftProcess::ajustVecByMidXYZ(std::vector<Pt3> inputVec1,
	std::vector<Pt3> inputVec2,
	std::vector<Pt3>& adjustVec1,
	std::vector<Pt3>& adjustVec2,
	double& midX,
	double& midY,
	double& midZ)
{
	//0，先判断两个输入序列是否为空
	int sizeofVec1 = inputVec1.size();
	int sizeofVec2 = inputVec2.size();
	if (sizeofVec1 == 0 || sizeofVec2 == 0)
	{
		return;
	}

	//1,分别计算两个序列中的最小值
	int midID1 = sizeofVec1 / 2;
	Pt3 midPt1 = inputVec1[midID1];
	double midX1 = midPt1.x();
	double midY1 = midPt1.y();
	double midZ1 = midPt1.z();

	int midID2 = sizeofVec2 / 2;
	Pt3 midPt2 = inputVec2[midID2];
	double midX2 = midPt2.x();
	double midY2 = midPt2.y();
	double midZ2 = midPt2.z();

	//2,从两个序列中的中间值计算中值
	midX = (midX1 + midX2) / 2;
	midY = (midY1 + midY2) / 2;
	midZ = (midZ1 + midZ2) / 2;

	//3,得出调整后的序列
	adjustVec1 = this->getAjustVecFromVecAndDelta(inputVec1, midX, midY, midZ);
	adjustVec2 = this->getAjustVecFromVecAndDelta(inputVec2, midX, midY, midZ);

}
//计算出给定vector的最大最小x,y,z值
void siftProcess::getMinXYZFromVec(std::vector<Pt3> vec, double& minX, double& minY, double& minZ)
{
	//1,先判断输入的vec是否为空，空则直接返回
	int sizeOfVec = vec.size();
	if (sizeOfVec == 0)
	{
		return;
	}

	//2，逐个进行计算。最小值赋初值为第一个
	minX = vec[0].x();
	minY = vec[0].y();
	minZ = vec[0].z();
	for (int i = 0; i < sizeOfVec; i++)
	{
		Pt3 thePt = vec[i];
		double theX = thePt.x();
		double theY = thePt.y();
		double theZ = thePt.z();
		if (minX > theX)
		{
			minX = theX;
		}
		if (minY > theY)
		{
			minY = theY;
		}
		if (minZ > theZ )
		{
			minZ = theZ;
		}
	}
}

//计算调整后的序列
std::vector<Pt3> siftProcess::getAjustVecFromVecAndDelta(std::vector<Pt3> vec, double minX, double minY, double minZ)
{
	std::vector<Pt3> adjustVec;
	adjustVec.clear();

	//1,判断输入的序列是否为空，空则返回空
	int sizeofInputVec = vec.size();
	if (sizeofInputVec == 0)
	{
		adjustVec.clear();
		return adjustVec;
	}

	//2,不为空时逐个赋值
	adjustVec.resize(sizeofInputVec);
	for (size_t i = 0; i < sizeofInputVec; i++)
	{
		Pt3 thePt = vec[i];
		double x = thePt.x() - minX;
		double y = thePt.y() - minY;
		double z = thePt.z() - minZ;

		Pt3 outputPt(x, y, z);
		adjustVec[i] = outputPt;
	}

	return adjustVec;
}
  
//精配准
Eigen::Matrix4f siftProcess::ICPRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud)
{
	//准备ICP精配准
	//是使用之前要至少set三个参数： 
	//1,setMaximumIterations， 最大迭代次数，icp是一个迭代的方法，最多迭代这些次；

	//2,setTransformationEpsilon， 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件；

	//3,setEuclideanFitnessEpsilon， 还有一条收敛条件是均方误差和小于阈值， 停止迭代。

	double correspondenceDistance = 5000; //10000
	int ICPmaximumIterations = 2000;//200
	double ICPTransformationEpsilon = 100;//100
	//double ICPEuclideanFitnessEpsilon = 0.5;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalAdjustPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(sourceCloud);
	icp.setInputTarget(targetCloud);
	icp.setMaxCorrespondenceDistance(correspondenceDistance);
	icp.setMaximumIterations(ICPmaximumIterations);
	icp.setTransformationEpsilon(ICPTransformationEpsilon);
	//icp.setEuclideanFitnessEpsilon(ICPEuclideanFitnessEpsilon);
	icp.align(*finalAdjustPointCloud);

	std::string strFinalPointCloud = "E:\\test\\finalAdjustPointCloud.pcd";
	pcl::io::savePCDFile(strFinalPointCloud, *finalAdjustPointCloud);

	//差值点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr diffCloud(new pcl::PointCloud<pcl::PointXYZ>);
	diffCloud->clear();
	diffCloud->height = 1;
	diffCloud->width = finalAdjustPointCloud->size();
	diffCloud->resize(diffCloud->width * diffCloud->height);
	for (size_t i = 0; i < diffCloud->size(); i++)
	{
		diffCloud->points[i].x = finalAdjustPointCloud->points[i].x - targetCloud->points[i].x;
		diffCloud->points[i].y = finalAdjustPointCloud->points[i].y - targetCloud->points[i].y;
		diffCloud->points[i].z = finalAdjustPointCloud->points[i].z - targetCloud->points[i].z;

	}

	std::string strDiffCloud = "E:\\test\\diffCloud.pcd";
	pcl::io::savePCDFile(strDiffCloud, *diffCloud);

	finalAdjustPointCloud->clear();
	diffCloud->clear();
	//返回最终矩阵
	Eigen::Matrix4f finalMatrix = icp.getFinalTransformation();
	return finalMatrix;

}

//点云平移回来
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getCloudFromAdjustCloudAndMinXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud,
	double minX, double minY, double minZ)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	originalCloud->clear();

	//1,判断输入点云个数，0则返回空
	int sizeOfInputCloud = adjustCloud->points.size();
	if ( sizeOfInputCloud == 0)
	{
		originalCloud->points.clear();
		return originalCloud;
	}
	//2，对点云每个值都进行+(minX,minY,minZ),并赋值给新点云

	originalCloud->height = 1;
	originalCloud->width = sizeOfInputCloud;
	originalCloud->resize(originalCloud->width * originalCloud->height);
	for (size_t i = 0; i < originalCloud->size(); i++)
	{
		originalCloud->points[i].x = adjustCloud->points[i].x + minX;
		originalCloud->points[i].y = adjustCloud->points[i].y + minY;
		originalCloud->points[i].z = adjustCloud->points[i].z + minZ;

	}

	//3,返回新点云
	return originalCloud;
}

//根据id和宽度取得
double siftProcess::getPixelFromID(int xID, int yID, int widthRoi, std::vector<float> pixel32BitVec)
{
	int theID = xID + yID * widthRoi;
	double thePixel = pixel32BitVec[theID];
	return thePixel;
}
//分区，根据图像的（xSize,ySize,widthRoi,HeightRoi)确定(xroi,yroi,widthRoitrue,heightRoitrue)
std::vector<std::vector<zone>> siftProcess::getZoneVecVec(int xSize, int ySize, int widthRoi, int heightRoi)
{
	std::vector<std::vector<zone>> zoneVecVec;
	zoneVecVec.clear();

	//分块
	int zoneX = xSize / widthRoi;
	int zoneY = ySize / heightRoi;
	zoneVecVec.resize(zoneY);
	for (size_t j = 0; j < zoneY; j++)
	{
		zoneVecVec[j].resize(zoneX);
	}

	for (size_t j = 0; j < zoneY; j++)
	{
		for (size_t i = 0; i < zoneX; i++)
		{
			int roix = i * widthRoi;
			int roiy = j * heightRoi;
			int theWidthRoi = widthRoi;
			int theHeightRoi = heightRoi;
			if ( roix + theWidthRoi > xSize )
			{
				theWidthRoi = xSize - roix;
			}

			if (roiy + theHeightRoi > ySize)
			{
				theHeightRoi = ySize - roiy;
			}
			
			zone theZone;
			theZone.xRoi = roix;
			theZone.yRoi = roiy;
			theZone.widthRoi = theWidthRoi;
			theZone.heightRoi = theHeightRoi;
			zoneVecVec[j][i] = theZone;
		}

	}
	return zoneVecVec;

}

std::vector<std::vector<zone>> siftProcess::getZoneVecVec(int xSize, int ySize, float ratioX, float ratioY, 
	int xStart, int yStart, int widthRoi, int heightRoi)
{
	std::vector<std::vector<zone>> zoneVecVec;
	zoneVecVec.clear();

	//分块
	int zoneX = xSize / widthRoi;
	int zoneY = ySize / heightRoi;

	int minZoneX = xStart / widthRoi;
	int minZoneY = yStart / heightRoi;
	int maxZoneX = minZoneX + ratioX * zoneX;
	int maxZoneY = minZoneY + ratioY * zoneY;
	if (maxZoneX > zoneX)
	{
		maxZoneX = zoneX;
	}
	if ( maxZoneY > zoneY )
	{
		maxZoneY = zoneY;
	}
	std::cout << "minZoneX = " << minZoneX << ", minZoneY = " << minZoneY << std::endl
		<< "maxZoneX = " << maxZoneX << ",maxZoneY = " << maxZoneY << std::endl;
	zoneVecVec.resize( maxZoneY - minZoneY );
	for (size_t j = 0; j < maxZoneY - minZoneY; j++)
	{
		zoneVecVec[j].resize(maxZoneX - minZoneX);
	}

	for (size_t j = minZoneY; j < maxZoneY; j++)
	{
		for (size_t i = minZoneX; i < maxZoneX; i++)
		{
			int roix = i * widthRoi;
			int roiy = j * heightRoi;
			int theWidthRoi = widthRoi;
			int theHeightRoi = heightRoi;
			if (roix + theWidthRoi > xSize)
			{
				theWidthRoi = xSize - roix;
			}

			if (roiy + theHeightRoi > ySize)
			{
				theHeightRoi = ySize - roiy;
			}

			zone theZone;
			theZone.xRoi = roix;
			theZone.yRoi = roiy;
			theZone.widthRoi = theWidthRoi;
			theZone.heightRoi = theHeightRoi;
			zoneVecVec[j - minZoneY][i - minZoneX] = theZone;
		}

	}
	return zoneVecVec;

}

//返回第一块区域对应的对应内点三维坐标序列
void siftProcess::getColinersFromZone(zone theZone1, tifParameter tif1, tifParameter tif2, 
	std::vector<Pt3>& colinersOfTheZoneVec1, std::vector<Pt3>& colinersOfTheZoneVec2)
{
	//zone1的区域最小id不能为0
	int xID1 = theZone1.xRoi;
	int yID1 = theZone1.yRoi;
	if (xID1 < 0 || yID1 < 0)
	{
		return;
	}
	//1,根据第一幅图的每块分区确定第二副图像的分区
	zone zone2;
	util::getZone2FromZone1(theZone1, tif1, tif2, zone2);
	//zone2的区域最小id不能为0
	int xID2 = zone2.xRoi;
	int yID2 = zone2.yRoi;
	if (xID2 < 0 || yID2 < 0)
	{
		return;
	}

	//2,计算.tif相应位置相应区域的32位像素集合
	//计算得到相应截取区域的像素序列
	std::vector<float> rasterID32bitVecForSift1 = util::getSegRasterVecVecFromTif_32bit(_strImageFile1Name32bit.c_str(), theZone1);
	std::vector<float> rasterID32bitVecForSift2 = util::getSegRasterVecVecFromTif_32bit(_strImageFile2Name32bit.c_str(), zone2);

	//像素序列空则返回
	int sizeof32BitSift1 = rasterID32bitVecForSift1.size();
	int sizeof32BitSift2 = rasterID32bitVecForSift2.size();
	if (sizeof32BitSift1 == 0 || sizeof32BitSift2 == 0)
	{
		return;
	}
	std::cout << "32bit size:(" << rasterID32bitVecForSift1.size() << "," << rasterID32bitVecForSift2.size() << ")" << std::endl;
	//3，将32位像素序列转换为8像素序列。
	std::vector<uchar> rasterID8bitVecForSift1 = this->convert32bitPixelVectorTo8bitPixelVector(rasterID32bitVecForSift1);
	std::vector<uchar> rasterID8bitVecForSift2 = this->convert32bitPixelVectorTo8bitPixelVector(rasterID32bitVecForSift2);
	//像素序列空则返回
	int sizeof8BitSift1 = rasterID8bitVecForSift1.size();
	int sizeof8BitSift2 = rasterID8bitVecForSift2.size();
	if (sizeof8BitSift1 == 0 || sizeof8BitSift2 == 0)
	{
		return;
	}
	std::cout << "8bit size:(" << rasterID8bitVecForSift1.size() << "," << rasterID8bitVecForSift2.size() << ")" << std::endl;
	//4，将像素序列填充到opencv图像
	cv::Mat srcImage8BitForSift1 = this->getOpenCVImgFrom8bitVec(rasterID8bitVecForSift1,theZone1);
	cv::Mat srcImage8BitForSift2 = this->getOpenCVImgFrom8bitVec(rasterID8bitVecForSift2, zone2);

	//5,均衡直方图
	cv::Mat heqResult1;
	cv::equalizeHist(srcImage8BitForSift1, heqResult1);
	cv::Mat heqResult2;
	cv::equalizeHist(srcImage8BitForSift2, heqResult2);

	//6，用opencv的sift算法提取出两幅图像的sift序列和内点序列（都是局部坐标序号）
	// 第一幅源图像的OpenCV二维内点序列
	std::vector<cv::Point2f> corlinerPointVec1InOpenCVOfTheZone;
	corlinerPointVec1InOpenCVOfTheZone.clear();
	// 第二幅源图像的OpenCV二维内点序列
	std::vector<cv::Point2f> corlinerPointVec2InOpenCVOfTheZone;
	corlinerPointVec2InOpenCVOfTheZone.clear();

	this->getSIFTFeatureFromOpenCVImage8bit(srcImage8BitForSift1, 
		srcImage8BitForSift2, 
		corlinerPointVec1InOpenCVOfTheZone, 
		corlinerPointVec2InOpenCVOfTheZone);
	//this->getSIFTFeatureFromOpenCVImage8bit(heqResult1, heqResult2);
	//7，将各个局部坐标序列转到三维坐标
	colinersOfTheZoneVec1 = this->convertOpenCV2DVecToPCL3DVec(
		theZone1, 
		tif1,
		corlinerPointVec1InOpenCVOfTheZone, 
		rasterID32bitVecForSift1);
	colinersOfTheZoneVec2 = this->convertOpenCV2DVecToPCL3DVec(
		zone2, 
		tif2,
		corlinerPointVec2InOpenCVOfTheZone, 
		rasterID32bitVecForSift2);

	//8，清空临时序列
	rasterID32bitVecForSift1.clear();
	rasterID32bitVecForSift2.clear();
	rasterID8bitVecForSift1.clear();
	rasterID8bitVecForSift2.clear();

}
//选取一部分点云进行处理
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud, float ratioOfDataSize )
{
	//写点云(局部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->clear();
	cloud->width = originalCloud->points.size() * ratioOfDataSize;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->size(); i++)
	{

		double x = originalCloud->points[i].x;
		double y = originalCloud->points[i].y;
		double z = originalCloud->points[i].z;

		pcl::PointXYZ thePt;
		thePt.x = x;
		thePt.y = y;
		thePt.z = z;
		cloud->points[i] = thePt;

	}
	return cloud;
}
//从点云中得到序列
std::vector<Pt3> siftProcess::getVecFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::vector<Pt3> cloudVector;
	cloudVector.clear();
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		Pt3 thePt(x, y, z);
		cloudVector.push_back(thePt);
	}
	return cloudVector;
}

//将目前点对按照比例获取新点对。
void siftProcess::filterColiner(std::vector<Pt3> inputVec1, std::vector<Pt3> inputVec2,
	std::vector<Pt3>& outputVec1, std::vector<Pt3>& outputVec2,
	float ratioFilter)
{
	//1,如果输入序列大小为空，则返回
	int sizeofVec1 = inputVec1.size();
	int sizeofVec2 = inputVec2.size();
	if ( sizeofVec1 == 0 || sizeofVec2 == 0)
	{
		return;
	}
	//2，计算差值，形成一个序列vecdiff，(vec1,vec2,vecdiff)
	std::vector<diffVec> diffVecVec;
	diffVecVec.clear();
	for (size_t i = 0; i < sizeofVec1; i++)
	{
		float diffX = inputVec2[i].x() - inputVec1[i].x();
		float diffY = inputVec2[i].y() - inputVec1[i].y();
		float diffZ = inputVec2[i].z() - inputVec1[i].z();
		Pt3 diffPoint(diffX, diffY, diffZ);
		Pt3 inputPoint1 = inputVec1[i];
		Pt3 inputPoint2 = inputVec2[i];
		diffVec thePoint;
		thePoint.inputPt1 = inputPoint1;
		thePoint.inputPt2 = inputPoint2;
		thePoint.diffPt = diffPoint;
		diffVecVec.push_back(thePoint);
	}
	//2，将vecdiff按照x值排序，筛掉序号倍数ratioFilter以外的值，前后两端
	std::sort(diffVecVec.begin(), diffVecVec.end(), util::greaterSortX);
	std::vector<diffVec> diffVecVecX = this->getSortDiffVec(diffVecVec, ratioFilter);
	diffVecVec.clear();
	//3，将vecdiff按照y值排序，筛掉序号倍数ratioFilter以外的值，前后两端
	std::sort(diffVecVecX.begin(), diffVecVecX.end(), util::greaterSortY);
	std::vector<diffVec> diffVecVecY = this->getSortDiffVec(diffVecVecX, ratioFilter);
	diffVecVecX.clear();

	//4，将vecdiff按照z值排序，筛掉序号倍数ratioFilter以外的值，前后两端
	std::sort(diffVecVecY.begin(), diffVecVecY.end(), util::greaterSortZ);
	std::vector<diffVec> diffVecVecZ = this->getSortDiffVec(diffVecVecY, ratioFilter);
	diffVecVecY.clear();

	//5，返回过滤后的点对坐标
	outputVec1.clear();
	outputVec2.clear();
	for (size_t i = 0; i < diffVecVecZ.size(); i++)
	{
		diffVec theVec = diffVecVecZ[i];
		Pt3 input1Pt = theVec.inputPt1;
		Pt3 input2Pt = theVec.inputPt2;
		outputVec1.push_back(input1Pt);
		outputVec2.push_back(input2Pt);
	}
	diffVecVecZ.clear();
}

//根据比例计算新的过滤后的vec
std::vector<diffVec> siftProcess::getSortDiffVec(std::vector<diffVec> inputDiffVec, float ratioFilter)
{
	std::vector<diffVec> filterDiffVec;
	filterDiffVec.clear();
	int filterSize = inputDiffVec.size() * ratioFilter;
	int beginID = (inputDiffVec.size() - filterSize) / 2;
	int endID = beginID + filterSize;
	for (size_t i = beginID; i < endID; i++)
	{
		diffVec theDiff = inputDiffVec[i];
		filterDiffVec.push_back(theDiff);
	}
	return filterDiffVec;
}


//计算序列的中心点(几何平均）
void siftProcess::getMidPointOfTheVec(std::vector<Pt3> inputVec, double &midCoordX1, double &midCoordY1, double &midCoordZ1)
{
	//判断输入序列是否为0，0则返回
	int theSize = inputVec.size();
	if ( theSize == 0 )
	{
		return;
	}
	//遍历输入序列的各个坐标点，计算出坐标xyz最大值和最小值
	double minX = inputVec[0].x();
	double minY = inputVec[0].y();
	double minZ = inputVec[0].z();
	double maxX = inputVec[0].x();
	double maxY = inputVec[0].y();
	double maxZ = inputVec[0].z();
	for (size_t i = 0; i < theSize; i++)
	{
		double theX = inputVec[i].x();
		double theY = inputVec[i].y();
		double theZ = inputVec[i].z();
		if (theX > maxX)
		{
			maxX = theX;
		}
		if (theX < minX)
		{
			minX = theX;
		}
		if (theY > maxY)
		{
			maxY = theY;
		}
		if (theY < minY)
		{
			minY = theY;
		}
		if (theZ > maxZ)
		{
			maxZ = theZ;
		}
		if (theZ < minZ)
		{
			minZ = theZ;
		}
	}
	//根据最小值和最大值，平均得到中值
	midCoordX1 = (minX + maxX) / 2;
	midCoordY1 = (minY + maxY) / 2;
	midCoordZ1 = (minZ + maxZ) / 2;

}

//计算序列的中心点(加权平均）
void siftProcess::getWeightedmeanMidPointOfTheVec(std::vector<Pt3> inputVec, double &midCoordX1, double &midCoordY1, double &midCoordZ1)
{
	//判断输入序列是否为0，0则返回
	int theSize = inputVec.size();
	if (theSize == 0)
	{
		return;
	}

	//计算总和再平均
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;
	for (size_t i = 0; i < theSize; i++)
	{
		double theX = inputVec[i].x();
		double theY = inputVec[i].y();
		double theZ = inputVec[i].z();

		sumX += theX;
		sumY += theY;
		sumZ += theZ;
		
	}
	//根据最小值和最大值，平均得到中值
	midCoordX1 = sumX / theSize;
	midCoordY1 = sumY / theSize;
	midCoordZ1 = sumZ / theSize;
	std::cout << "midCoordX1 = " << midCoordX1 << ",midCoordY1 = " << midCoordY1 << ",midCoordZ1 = " << midCoordZ1 << std::endl;

}


//计算序列的最大最小点
void siftProcess::getMinMaxPointOfTheVec(std::vector<Pt3> inputVec, 
	double &minX, double &maxX, 
	double &minY, double &maxY,
	double &minZ, double &maxZ)
{
	//判断输入序列是否为0，0则返回
	int theSize = inputVec.size();
	if (theSize == 0)
	{
		return;
	}
	minX = inputVec[0].x();
	minY = inputVec[0].y();
	minZ = inputVec[0].z();
	maxX = inputVec[0].x();
	maxY = inputVec[0].y();
	maxZ = inputVec[0].z();
	//遍历输入序列的各个坐标点，计算出坐标xyz最大值和最小值
	for (size_t i = 0; i < theSize; i++)
	{
		double theX = inputVec[i].x();
		double theY = inputVec[i].y();
		double theZ = inputVec[i].z();
		if (theX > maxX)
		{
			maxX = theX;
		}
		if (theX < minX)
		{
			minX = theX;
		}
		if (theY > maxY)
		{
			maxY = theY;
		}
		if (theY < minY)
		{
			minY = theY;
		}
		if (theZ > maxZ)
		{
			maxZ = theZ;
		}
		if (theZ < minZ)
		{
			minZ = theZ;
		}
	}

}
//计算序列的中心点（标准差）
void siftProcess::getSigmaFromTheVec(std::vector<Pt3> inputVec, 
	double midCoordX1, double midCoordY1, double midCoordZ1,
	double &sigmaMidX, double &sigmaMidY, double &sigmaMidZ )
{
	//判断输入队列是否有值，没有则空
	int theSize = inputVec.size();
	if ( theSize == 0)
	{
		return;
	}
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;
	for (size_t i = 0; i < theSize; i++)
	{
		Pt3 thePt = inputVec[i];
		double theX = thePt.x();
		double theY = thePt.y();
		double theZ = thePt.z();
		
		double distanceX2 = (theX - midCoordX1) * (theX - midCoordX1);
		double distnaceY2 = (theY - midCoordY1) * (theY - midCoordY1);
		double distnaceZ2 = (theZ - midCoordZ1) * (theZ - midCoordZ1);
		sumX += distanceX2;
		sumY += distnaceY2;
		sumZ += distnaceZ2;
	}

	//判断
	sigmaMidX = sqrt(sumX / theSize);
	sigmaMidY = sqrt(sumY / theSize);
	sigmaMidZ = sqrt(sumZ / theSize);

}

//计算标准差中心点和相应序列
void siftProcess::getSigmaMidPointAndVecFromVec(std::vector<Pt3> inputVec,
	std::vector<Pt3>& outputVec,
	double &midCoordX,
	double &midCoordY,
	double &midCoordZ)
{
	//1,先判断输入是否为空，空则返回
	int sizeofInputVec = inputVec.size();
	if (sizeofInputVec == 0 )
	{
		return;
	}
	
	//2，计算总和的加权平均值，计算偏向几何平均值的左侧或右侧，即大小 
	double weightMidX = 0;
	double weightMidY = 0;
	double weightMidZ = 0;
	this->getWeightedmeanMidPointOfTheVec(inputVec, weightMidX, weightMidY, weightMidZ );
	std::cout << "weightMidX =" << weightMidX << ",weightMidY = " << weightMidY << ",weightMidZ = " << weightMidZ << std::endl;
	//3，计算标准差
	double sigmaX = 0;
	double sigmaY = 0;
	double sigmaZ = 0;
	this->getSigmaFromTheVec(inputVec,
		weightMidX, weightMidY, weightMidZ,
		sigmaX, sigmaY, sigmaZ);
	std::cout << "sigmaX =" << sigmaX << ",sigmaY = " << sigmaY << ",sigmaZ = " << sigmaZ << std::endl;

	//4,得到取值范围
	double minSigMaX = weightMidX - sigmaX;
	double minSigMaY = weightMidY - sigmaY;
	double minSigMaZ = weightMidZ - sigmaZ;
	double maxSigMaX = weightMidX + sigmaX;
	double maxSigMaY = weightMidY + sigmaY;
	double maxSigMaZ = weightMidZ + sigmaZ;

	//5，将输入序列中，加权平均值正负标准差之外的去掉，
	outputVec = this->filterSigmaVec(inputVec,
		minSigMaX, minSigMaY, minSigMaZ, 
		maxSigMaX, maxSigMaY, maxSigMaZ);
	//6,再计算加权平均。
	this->getMidPointOfTheVec(outputVec, midCoordX, midCoordY, midCoordZ);

}
//得到sigma过滤的序列
std::vector<Pt3> siftProcess::filterSigmaVec(std::vector<Pt3> inputVec,
	double minSigMaX, double minSigMaY, double minSigMaZ,
	double maxSigMaX, double maxSigMaY, double maxSigMaZ)
{
	std::vector<Pt3> filterVec;
	filterVec.clear();
	//判断是否为空，空则返回
	int theSize = inputVec.size();
	if ( theSize == 0 )
	{
		filterVec.clear();
		return filterVec;
	}

	//遍历，符合条件的加入
	for (size_t i = 0; i < theSize; i++)
	{
		Pt3 thePt = inputVec[i];
		double theX = thePt.x();
		double theY = thePt.y();
		double theZ = thePt.z();
		bool bX = (theX <= maxSigMaX && theX >= minSigMaX);
		bool bY = (theY <= maxSigMaY && theY >= minSigMaY);
		bool bZ = (theZ <= maxSigMaZ && theZ >= minSigMaZ);

		if ( bX && bY && bZ )
		{
			filterVec.push_back(thePt);
		}
	}

	return filterVec;
}