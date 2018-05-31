#include "siftProcess.h"
#include "math.h"
siftProcess::siftProcess(int xRoil, int yRoil, int widthRoil, int heightRoil,
	std::string strImageFile1Name8bitForSift, 
	std::string strImageFile2Name8bitForSift,
	std::string strImageFile1Name32bitForPCL,
	std::string strImageFile2Name32bitForPCL)
{
	_xRoi1 = xRoil;
	_yRoi1 = yRoil;
	_widthRoi = widthRoil;
	_heightRoi = heightRoil;
	_strImageFile1Name8bitForSift = strImageFile1Name8bitForSift;
	_strImageFile2Name8bitForSift = strImageFile2Name8bitForSift;
	_strImageFile1Name32bitForPCL = strImageFile1Name32bitForPCL;
	_strImageFile2Name32bitForPCL = strImageFile2Name32bitForPCL;
	_roughMatrix.Identity();

	_rasterID8bitVecForSift1.clear();
	_rasterID8bitVecForSift2.clear();
	_rasterID32bitVecForSift1.clear();
	_rasterID32bitVecForSift2.clear();
	_corlinerPointVec1InOpenCV.clear();
	_corlinerPointVec2InOpenCV.clear();
	_colinerVectorInOpenCV.clear();
	_corlinerPointVec1InPCL.clear();
	_corlinerPointVec2InPCL.clear();
	_seg1Vector32bitForPCL.clear();
	_seg2Vector32bitForPCL.clear();
	if (_colinerCloud1)
	{
		_colinerCloud1->clear();
	}
	if (_colinerCloud2)
	{
		_colinerCloud2->clear();
	}
	if (_segCloud1)
	{
		_segCloud1->clear();
	}
	if (_segCloud2)
	{
		_segCloud2->clear();
	}
}

siftProcess::~siftProcess()
{
	_rasterID8bitVecForSift1.clear();
	_rasterID8bitVecForSift2.clear();
	_rasterID32bitVecForSift1.clear();
	_rasterID32bitVecForSift2.clear();
	_corlinerPointVec1InOpenCV.clear();
	_corlinerPointVec2InOpenCV.clear();
	_colinerVectorInOpenCV.clear();
	_corlinerPointVec1InPCL.clear();
	_corlinerPointVec2InPCL.clear();
	_seg1Vector32bitForPCL.clear();
	_seg2Vector32bitForPCL.clear();
	if (_segCloud1)
	{
		_segCloud1->clear();
	}
	if (_segCloud2)
	{
		_segCloud2->clear();
	}

	if (_colinerCloud1)
	{
		_colinerCloud1->clear();
	}
	if (_colinerCloud2)
	{
		_colinerCloud2->clear();
	}
	_cor_inliers_ptr->clear();
}

//处理所有步骤
void siftProcess::processAll()
{
	//0,计算出两幅图像的参数,以及全图像序列
	int xSize1 = 0;
	int ySize1 = 0;
	double xResolution1 = 0;
	double yResolution1 = 0;
	double topLeftX1 = 0;
	double topLeftY1 = 0;
	std::vector<std::vector<Pt3>> rastervecvec1 = util::getRasterVecVecFromTif(_strImageFile1Name32bitForPCL.c_str(),
		xSize1, ySize1,
		xResolution1, yResolution1,
		topLeftX1, topLeftY1);

	int xSize2 = 0;
	int ySize2 = 0;
	double xResolution2 = 0;
	double yResolution2 = 0;
	double topLeftX2 = 0;
	double topLeftY2 = 0;
	std::vector<std::vector<Pt3>> rastervecvec2 = util::getRasterVecVecFromTif(_strImageFile2Name32bitForPCL.c_str(),
		xSize2, ySize2,
		xResolution2, yResolution2,
		topLeftX2, topLeftY2);

	//1，从32位图像中提取像素序列
	_seg1Vector32bitForPCL = util::getSegRasterVecVecFromVecVec2(rastervecvec1, _xRoi1, _yRoi1, _widthRoi, _heightRoi);
	//从第一幅图截取范围的左上角(xRoi,yRoi)计算出另一幅图像的截取范围的左上角(xRoi2,yRoi2)
	double inputTopLeftX = _seg1Vector32bitForPCL[0][0].x();
	double inputTopLeftY = _seg1Vector32bitForPCL[0][0].y();

	int xRoi2 = 0;
	int yRoi2 = 0;
	util::getRoil2FromRoi1AndTif(_xRoi1, _yRoi1, _widthRoi, _heightRoi, inputTopLeftX, inputTopLeftY,
		xResolution2, yResolution2, topLeftX2, topLeftY2,  xSize2,  ySize2,
		 xRoi2,  yRoi2);
	//第二幅图的序列
	_seg2Vector32bitForPCL = util::getSegRasterVecVecFromVecVec2(rastervecvec2, xRoi2, yRoi2, _widthRoi, _heightRoi);

	//得到像素点序列
	_rasterID32bitVecForSift1 = util::getPixel32bitFromTifVecVec(_seg1Vector32bitForPCL);
	_rasterID32bitVecForSift2 = util::getPixel32bitFromTifVecVec(_seg2Vector32bitForPCL);

	//2，将32位像素序列转换为8像素序列。
	_rasterID8bitVecForSift1 = this->convert32bitPixelVectorTo8bitPixelVector(_rasterID32bitVecForSift1);
	_rasterID8bitVecForSift2 = this->convert32bitPixelVectorTo8bitPixelVector(_rasterID32bitVecForSift2);
	//2，将像素序列填充到opencv图像
	cv::Mat srcImage8BitForSift1 = this->getOpenCVImgFrom8bitVec(_rasterID8bitVecForSift1);
	cv::Mat srcImage8BitForSift2 = this->getOpenCVImgFrom8bitVec(_rasterID8bitVecForSift2);
	cv::imwrite("e:\\test\\srcImage8BitForSift1.jpg", srcImage8BitForSift1);
	cv::imwrite("e:\\test\\srcImage8BitForSift2.jpg", srcImage8BitForSift2);

	//3,均衡直方图
	cv::Mat heqResult1;
	cv::equalizeHist(srcImage8BitForSift1, heqResult1);
	cv::Mat heqResult2;
	cv::equalizeHist(srcImage8BitForSift2, heqResult2);

	//3，用opencv的sift算法提取出两幅图像的sift序列和内点序列（都是局部坐标序号）
	//this->getSIFTFeatureFromOpenCVImage8bit(srcImage8BitForSift1, srcImage8BitForSift2);
	this->getSIFTFeatureFromOpenCVImage8bit(heqResult1, heqResult2);
	//4，得出粗配准点对
	_cor_inliers_ptr = this->computeMiniCorrisponces(_corlinerPointVec1InOpenCV, _corlinerPointVec2InOpenCV);
	std::cout << "cor_inliers_ptr个数:" << _cor_inliers_ptr->size() << std::endl;
	FILE *fp_cor_inliers_ptr;
	fp_cor_inliers_ptr = fopen("E:\\test\\fp_cor_inliers_ptr2.txt", "w");
	for (size_t i = 0; i < _cor_inliers_ptr->size(); i++)
	{
		int firstID = _cor_inliers_ptr->at(i).index_query;
		int secondID = _cor_inliers_ptr->at(i).index_match;
		double distance = _cor_inliers_ptr->at(i).distance;
		double weight = _cor_inliers_ptr->at(i).weight;

		fprintf(fp_cor_inliers_ptr, "(<%d,%d>,%f,%f,)\n",
			firstID, secondID, distance, weight);

		if (i % 10 == 0)
		{
			std::cout << std::endl;
		}
	}
	fclose(fp_cor_inliers_ptr);
	
	//6，将各个局部坐标序列转到三维坐标
	_corlinerPointVec1InPCL = this->convertOpenCV2DVecToPCL3DVec(_corlinerPointVec1InOpenCV, _seg1Vector32bitForPCL);
	_corlinerPointVec2InPCL = this->convertOpenCV2DVecToPCL3DVec(_corlinerPointVec2InOpenCV, _seg2Vector32bitForPCL);

	//7,从三维坐标转换为点云

	_colinerCloud1 = this->getPointCloudFrom3dVec(_corlinerPointVec1InPCL);
	_colinerCloud2 = this->getPointCloudFrom3dVec(_corlinerPointVec2InPCL);
	_segCloud1 = this->getPointCloudFrom3dVec(_seg1Vector32bitForPCL);
	_segCloud2 = this->getPointCloudFrom3dVec(_seg1Vector32bitForPCL);
	pcl::io::savePCDFile("e:\\test\\_colinerCloud1.pcd", *_colinerCloud1);
	pcl::io::savePCDFile("e:\\test\\_colinerCloud2.pcd", *_colinerCloud2);

	//8，计算出进行粗配准的点云，尽可能消除坐标大小在矩阵中乘积的影响
	double minX = 0;
	double minY = 0;
	double minZ = 0;
	//先计算调整后的序列
	std::vector<Pt3> adjustVec1;
	adjustVec1.clear();
	std::vector<Pt3> adjustVec2;
	adjustVec2.clear();
	this->ajustVec(_corlinerPointVec1InPCL, _corlinerPointVec2InPCL, adjustVec1, adjustVec2,minX, minY, minZ);
	//根据调整后的序列得到调整后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud1 = this->getPointCloudFrom3dVec(adjustVec1);
	pcl::io::savePCDFile("e:\\test\\adjustCloud1.pcd", *adjustCloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud2 = this->getPointCloudFrom3dVec(adjustVec2);
	pcl::io::savePCDFile("e:\\test\\adjustCloud2.pcd", *adjustCloud2);

	//9，使用pcl的算法得出粗配准矩阵
	this->computeRoughMatrix(adjustCloud1, adjustCloud2,_cor_inliers_ptr);
	//输出粗配准矩阵
	std::cout << "输出粗配准矩阵" << std::endl;
	std::cout << _roughMatrix << std::endl;
	//根据粗配准矩阵得出粗配准点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustRoughCloud = this->getRoughPointCloudFromMatrix(adjustCloud1, _roughMatrix);
	pcl::io::savePCDFile("e:\\test\\adjustRoughCloud.pcd", *adjustRoughCloud);

	//10，用icp得出精配准矩阵
	Eigen::Matrix4f detailMatrix = this->ICPRegistration(adjustRoughCloud, adjustCloud2);
	//输出最终矩阵
	std::cout << "输出精配准矩阵" << std::endl;
	std::cout << detailMatrix << std::endl;

	//11,点云平移回来
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud = this->getCloudFromAdjustCloudAndMinXYZ(adjustRoughCloud, minX, minY, minZ);
	pcl::io::savePCDFile("e:\\test\\finalCloud.pcd", *finalCloud);
	
}

//根据粗配准矩阵得出粗配准点云
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getRoughPointCloudFromMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	Eigen::Matrix4f transformation_matrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Raw转换前个数：" << cloud_in_filtered_Raw->size() << std::endl;
	pcl::transformPointCloud(*inputCloud, *cloud_in_filtered_Raw, transformation_matrix);
	std::cout << "cloud_in_filtered_Raw转换后个数：" << cloud_in_filtered_Raw->size() << std::endl;

	return cloud_in_filtered_Raw;
}
//二维序号坐标转三维坐标
Pt3 siftProcess::convertOpenCV2DToPCL3D(cv::Point2f point2d, std::vector<std::vector<Pt3>> segVec3d)
{
	//将二维坐标的i,j转换为三维中的坐标
	int xID = point2d.x;
	int yID = point2d.y;
	Pt3 thePt = segVec3d[yID][xID];
	return thePt;

}
//二维序号坐标vector转三维坐标vector
std::vector<Pt3> siftProcess::convertOpenCV2DVecToPCL3DVec(std::vector<cv::Point2f> point2dVec, std::vector<std::vector<Pt3>> segVec3d)
{
	std::vector<Pt3> pcl3DVec;
	pcl3DVec.clear();
	//1,首先判断二维序列和三维序列是否为空
	if (point2dVec.size() == 0 || segVec3d.size() == 0)
	{
		pcl3DVec.clear();
		return pcl3DVec;
	}
	//2，将二维序列中的数据转化为pt3
	std::vector<cv::Point2f>::iterator
		iterCurVec = point2dVec.begin(),
		iterEndVec = point2dVec.end();
	for (; iterCurVec != iterEndVec; iterCurVec++)
	{
		//将每个cv::Point2f转换为Pt3
		cv::Point2f thePoint2 = *iterCurVec;
		Pt3 thePoint3 = convertOpenCV2DToPCL3D(thePoint2, segVec3d);
		pcl3DVec.push_back(thePoint3);
	}

	return pcl3DVec;
}
//计算图像中的SIFT特征及匹配,得出对应点对序号的vector,
cv::Mat siftProcess::getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, cv::Mat srcImage2 )
{
	//定义sift描述子
	//cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create(0, 6, 0.08, 15, 1.0);
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

	//查找关键点
	std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
	f2d->detect(srcImage1, keyPoints_1);
	f2d->detect(srcImage2, keyPoints_2);

	//计算描述子
	cv::Mat descriptor_1, descriptor_2;
	f2d->compute(srcImage1, keyPoints_1, descriptor_1);
	f2d->compute(srcImage2, keyPoints_2, descriptor_2);

	//特征点匹配
	cv::BFMatcher matcher;
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
		for (size_t i = 0; i < theMatch.size(); i++)
		{
			matchesVectorInOpenCV.push_back(theMatch[i]);
		}

	}
	//输出匹配结果
	FILE * fp = fopen("e:\\test\\affineSift.txt", "w");
	fprintf(fp, "firstID        secondID                第一个坐标                       第二个坐标                            坐标差值\n");

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

		//内点
		_colinerVectorInOpenCV.push_back(matchesVectorInOpenCV[i]);
		_corlinerPointVec1InOpenCV.push_back(ptFirst);
		_corlinerPointVec2InOpenCV.push_back(ptSecond);
		fprintf(fp, "    %-15d%-15d%-10.3f%-25.3f%-10.3f%-25.3f%-10.3f%-25.3f%-25.3f\n", idFirst, idSecond, firstPtX, firstPtY, secondPtX, secondPtY, diffX, diffY, theDistance);

	}

	fclose(fp);
	//得到转换矩阵
	cv::Mat matchMat;
	cv::drawMatches(srcImage1, keyPoints_1, srcImage2, keyPoints_2, matchesVectorVector, matchMat);

	//剔除不一致
	cv::Mat finalMat = cv::findHomography(_corlinerPointVec1InOpenCV, _corlinerPointVec2InOpenCV, cv::RANSAC);
	std::cout << finalMat << std::endl;
	return matchMat;

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
cv::Mat siftProcess::getOpenCVImgFrom8bitVec(std::vector<uchar> rastervec8Bit)
{
	cv::Mat srcImage(_heightRoi, _widthRoi, CV_8UC1);;
	for (int i = 0; i < _heightRoi; i++)
	{
		for (int j = 0; j < _widthRoi; j++)
		{
			srcImage.at<uchar>(i, j) = rastervec8Bit[i * _widthRoi + j];
		}
	}
	return srcImage;
}
//计算粗配准矩阵
void siftProcess::computeRoughMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, 
	boost::shared_ptr<pcl::Correspondences> cor)
{
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_east;
	//trans_east.estimateRigidTransformation(*_colinerCloud1, *_colinerCloud2, *_cor_inliers_ptr, _roughMatrix);
	trans_east.estimateRigidTransformation(*cloud1, *cloud2, *cor, _roughMatrix);
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
boost::shared_ptr<pcl::Correspondences> siftProcess::computeMiniCorrisponces(std::vector<cv::Point2f> firstIDVector, std::vector<cv::Point2f> secondIDVector)
{
	//计算最小个数
	int minSize = firstIDVector.size();
	if (minSize > secondIDVector.size())
	{
		minSize = secondIDVector.size();
	}

	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);

	//赋值给内点
	for (size_t i = 0; i < minSize; i++)
	{
		pcl::Correspondence theCorr;
		theCorr.index_query = i;
		theCorr.index_match = i; 
		//theCorr.distance = _colinerVectorInOpenCV[i].distance;
		cor_inliers_ptr->push_back(theCorr);
	}
	return cor_inliers_ptr;
}

//根据opencv二维坐标得到对应点对
boost::shared_ptr<pcl::Correspondences> siftProcess::getCorrisponcesByOpenCVlocalCoordinate(std::vector<cv::Point2f> firstIDVector, std::vector<cv::Point2f> secondIDVector)
{
	//计算最小个数
	int minSize = firstIDVector.size();
	if (minSize > secondIDVector.size())
	{
		minSize = secondIDVector.size();
	}

	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);

	//赋值给内点
	for (size_t i = 0; i < minSize; i++)
	{
		int xID1 = firstIDVector[i].x;
		int yID1 = firstIDVector[i].y;
		int ID1 = xID1 + yID1 * _widthRoi;

		int xID2 = secondIDVector[i].x;
		int yID2 = secondIDVector[i].y;
		int ID2 = xID2 + yID2 * _widthRoi;

		pcl::Correspondence theCorr;
		theCorr.index_query = ID1;
		theCorr.index_match = ID2;
		theCorr.distance = _colinerVectorInOpenCV[i].distance;
		cor_inliers_ptr->push_back(theCorr);
	}
	return cor_inliers_ptr;
}

//opencv转到pcl点对
boost::shared_ptr<pcl::Correspondences> siftProcess::getCorrisponcesByOpenCVMatcher(std::vector<cv::DMatch> matchesVectorInOpenCV)
{
	int theSize = matchesVectorInOpenCV.size();
	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);

	//赋值给内点
	for (size_t i = 0; i < theSize; i++)
	{
		pcl::Correspondence theCorr;
		theCorr.index_query = matchesVectorInOpenCV[i].queryIdx;
		theCorr.index_match = matchesVectorInOpenCV[i].trainIdx;
		theCorr.distance = matchesVectorInOpenCV[i].distance;
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
	
	//3，如果最小值和最大值相等，则返回空
	if (minPixel32bit == maxPixel32bit)
	{
		pixelVector8bit.clear();
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

//计算出进行粗配准的点云，尽可能消除坐标大小在矩阵中乘积的影响
void siftProcess::ajustVec(std::vector<Pt3> inputVec1,
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

	double correspondenceDistance = 0.5;
	int ICPmaximumIterations = 200;
	double ICPTransformationEpsilon = 1e-6;
	double ICPEuclideanFitnessEpsilon = 0.5;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalAdjustPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(sourceCloud);
	icp.setInputTarget(targetCloud);
	icp.setMaxCorrespondenceDistance(correspondenceDistance);
	icp.setMaximumIterations(ICPmaximumIterations);
	icp.setTransformationEpsilon(ICPTransformationEpsilon);
	icp.setEuclideanFitnessEpsilon(ICPEuclideanFitnessEpsilon);
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