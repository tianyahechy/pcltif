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

//�������в���
void siftProcess::processAll()
{
	time_t startTime, endTime;
	time(&startTime);
	//0������.tif���Ƶõ���.tif�ĸ�Ҫ��
	int xSize1 = _tifParameter1.xSize;
	int ySize1 = _tifParameter1.ySize;
	float ratioX = 0.5;
	float ratioY = 0.5;
	int startX = 0.3 * xSize1;
	int startY = 0.3 * ySize1;

	//1,����������ͼ��ģ�xSize,ySize,widthRoi,HeightRoi)ȷ��(xroi,yroi,widthRoitrue,heightRoitrue)
	std::vector<std::vector<zone>> zoneVecVec1 = this->getZoneVecVec(xSize1, ySize1, ratioX, ratioY, startX, startY, _widthRoi, _heightRoi);
	//����ÿ�����򣬸��ݵ�һ��ͼ��λ�ü���ڶ���ͼ������Ϳ��
	std::vector<Pt3> coliners1InOpenCV;
	coliners1InOpenCV.clear();
	std::vector<Pt3> coliners2InOpenCV;
	coliners2InOpenCV.clear();


	for (size_t j = 0; j < zoneVecVec1.size(); j++)
	{
		for (size_t i = 0; i < zoneVecVec1[j].size(); i++)
		{
			zone theZone1 = zoneVecVec1[j][i];
			
			//���ص�һ��ͼ�����Ķ�Ӧ��Ե���ά����
			std::vector<Pt3> colinersOfTheZone1Vec;
			colinersOfTheZone1Vec.clear();
			std::vector<Pt3> colinersOfTheZone2Vec;
			colinersOfTheZone2Vec.clear();

			this->getColinersFromZone(theZone1, _tifParameter1, _tifParameter2, colinersOfTheZone1Vec, colinersOfTheZone2Vec);

			//�����Դ�СΪ0��������
			int sizeofColiner1Vec = colinersOfTheZone1Vec.size();
			int sizeofColiner2Vec = colinersOfTheZone2Vec.size();
			if (sizeofColiner1Vec == 0 ||
				sizeofColiner2Vec == 0 )
			{
				continue;
			}

			//�������ڵ����ά��������Ӧ�����
			for (size_t k = 0; k < colinersOfTheZone1Vec.size(); k++)
			{
				coliners1InOpenCV.push_back(colinersOfTheZone1Vec[k]);
				coliners2InOpenCV.push_back(colinersOfTheZone2Vec[k]);
			}

			colinersOfTheZone1Vec.clear();
			colinersOfTheZone2Vec.clear();

		}
	}
	//2.0�ٹ���һ�ε��
	float ratioFilter = 0.9;
	_corlinerPointVec1InPCL.clear();
	_corlinerPointVec2InPCL.clear();
	this->filterColiner(coliners1InOpenCV, coliners2InOpenCV, _corlinerPointVec1InPCL, _corlinerPointVec2InPCL, ratioFilter);
	coliners1InOpenCV.clear();
	coliners2InOpenCV.clear();

	//2������
	
	int sizeofColiner = _corlinerPointVec1InPCL.size();
	_cor_inliers_ptr = this->computeMiniCorrisponces(sizeofColiner);
	std::cout << "cor_inliers_ptr����:" << _cor_inliers_ptr->size() << std::endl;
	FILE *fp_cor_inliers_ptr;
	fp_cor_inliers_ptr = fopen("E:\\test\\fp_cor_inliers_ptr2.txt", "w");
	fprintf(fp_cor_inliers_ptr, "������                         ����1                            ����2                            ��ֵ\n");

	for (size_t i = 0; i < _cor_inliers_ptr->size(); i++)
	{
		int firstID = _cor_inliers_ptr->at(i).index_query;
		int secondID = _cor_inliers_ptr->at(i).index_match;
		Pt3 pt1 = _corlinerPointVec1InPCL[i];
		float x1 = pt1.x();
		float y1 = pt1.y();
		float z1 = pt1.z();
		Pt3 pt2 = _corlinerPointVec2InPCL[i];
		float x2 = pt2.x();
		float y2 = pt2.y();
		float z2 = pt2.z();
		
		float diffX = x2 - x1;
		float diffY = y2 - y1;
		float diffZ = z2 - z1;
		fprintf(fp_cor_inliers_ptr, "��%d,%d,)        %0.6f,%0.6f,%-10.6f    %0.6f,%0.6f,%-10.6f    %0.6f,%0.6f,%-10.6f  \n",
			firstID, secondID,x1,y1,z1,x2,y2,z2,diffX,diffY, diffZ);

	}
	fclose(fp_cor_inliers_ptr);

	//3,����ά����ת��Ϊ����
	_colinerCloud1 = this->getPointCloudFrom3dVec(_corlinerPointVec1InPCL);
	_colinerCloud2 = this->getPointCloudFrom3dVec(_corlinerPointVec2InPCL);
	pcl::io::savePCDFile("e:\\test\\_colinerCloud1.pcd", *_colinerCloud1);
	pcl::io::savePCDFile("e:\\test\\_colinerCloud2.pcd", *_colinerCloud2);

	//5��ʹ��pcl���㷨�ó�����׼����
	this->computeRoughMatrix(_colinerCloud1, _colinerCloud2, _cor_inliers_ptr);
	//�������׼����
	std::cout << "�������׼����" << std::endl;
	std::cout << _roughMatrix << std::endl;
	//���ݴ���׼����ó�����׼����

	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustColinerRoughCloud1 = this->getRoughPointCloudFromMatrix(_colinerCloud1, _roughMatrix);
	std::vector<Pt3> adjustColinerVec1 = this->getVecFromCloud(adjustColinerRoughCloud1);
	FILE *fpAdjustColiner;
	fpAdjustColiner = fopen("E:\\test\\AdjustColiner.txt", "w");
	fprintf(fpAdjustColiner, "������                         ����1                            ����2                            ��ֵ\n");

	for (size_t i = 0; i < adjustColinerVec1.size(); i++)
	{
		Pt3 pt1 = adjustColinerVec1[i];
		float x1 = pt1.x();
		float y1 = pt1.y();
		float z1 = pt1.z();
		Pt3 pt2 = _corlinerPointVec2InPCL[i];
		float x2 = pt2.x();
		float y2 = pt2.y();
		float z2 = pt2.z();

		float diffX = x2 - x1 ;
		float diffY = y2 - y1;
		float diffZ = z2 - z1;
		fprintf(fpAdjustColiner, "       %0.6f,%0.6f,%-10.6f    %0.6f,%0.6f,%-10.6f    %0.6f,%0.6f,%-10.6f  \n",
			 x1, y1, z1, x2, y2, z2, diffX, diffY, diffZ);

	}
	fclose(fpAdjustColiner);

	time(&endTime);
	double comsumeTime = difftime(endTime, startTime);
	std::cout << "�ܹ��ķ�ʱ�䣺" << comsumeTime << std::endl;

	//8������
	adjustColinerVec1.clear();
	
}

//���ݴ���׼����ó�����׼����
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getRoughPointCloudFromMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	Eigen::Matrix4f transformation_matrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = inputCloud->points.size();
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Rawת��ǰ������" << cloud_in_filtered_Raw->size() << std::endl;
	pcl::transformPointCloud(*inputCloud, *cloud_in_filtered_Raw, transformation_matrix);
	std::cout << "cloud_in_filtered_Rawת���������" << cloud_in_filtered_Raw->size() << std::endl;

	return cloud_in_filtered_Raw;
}

//��ά�������vectorת��ά����vector(�ֲ�����
std::vector<Pt3> siftProcess::convertOpenCV2DVecToPCL3DVec(
	zone theZone, 
	tifParameter tifParam,
	std::vector<cv::Point2f> point2dVec, 
	std::vector<float> pixel32BitVec)
{
	std::vector<Pt3> pcl3DVec;
	pcl3DVec.clear();
	//1,�����ж϶�ά���к���ά�����Ƿ�Ϊ��
	if (point2dVec.size() == 0 || pixel32BitVec.size() == 0)
	{
		pcl3DVec.clear();
		return pcl3DVec;
	}

	//2������ά�����е�����ת��Ϊpt3
	int xRoi = theZone.xRoi;
	int yRoi = theZone.yRoi;
	int widthRoi = theZone.widthRoi;
	double leftTopX = tifParam.leftTopX;
	double leftTopY = tifParam.leftTopY;
	double xResolution = tifParam.xResolution;
	double yResolution = tifParam.yResolution;
	for (int i = 0; i < point2dVec.size(); i++)
	{
		//��ÿ��cv::Point2fת��ΪPt3
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

//����ͼ���е�SIFT������ƥ��,�ó���Ӧ�����ŵ�vector,
void siftProcess::getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, 
	cv::Mat srcImage2,
	std::vector<cv::Point2f>& corlinerPointVec1InOpenCVOfTheZone,
	std::vector<cv::Point2f>& corlinerPointVec2InOpenCVOfTheZone)
{
	//����sift������
	//cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create(0, 6, 0.08, 15, 1.0);
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

	//���ҹؼ���
	std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
	f2d->detect(srcImage1, keyPoints_1);
	f2d->detect(srcImage2, keyPoints_2);

	//����ؼ������Ϊ0����ƥ�� 
	int sizeofKeyPoint1 = keyPoints_1.size();
	int sizeofKeyPoint2 = keyPoints_2.size();
	if (sizeofKeyPoint1 == 0 || sizeofKeyPoint2 == 0 )
	{
		return;
	}
	
	//����������
	cv::Mat descriptor_1, descriptor_2;
	f2d->compute(srcImage1, keyPoints_1, descriptor_1);
	f2d->compute(srcImage2, keyPoints_2, descriptor_2);

	//������ƥ��
	cv::FlannBasedMatcher matcher;
	std::vector<std::vector<cv::DMatch>> matchesVectorVector;
	//matcher.match(descriptor_1, descriptor_2, matchesVector);
	//����һ�������ƥ��
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
	//���������Сֵ
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

	double ratio = 0.1;
	double standardDist = minDist + (maxDist - minDist) * ratio;
	//���ƥ����
	FILE * fp = fopen("e:\\test\\affineSift.txt", "w");
	fprintf(fp, "firstID        secondID                ��һ������                       �ڶ�������                            �����ֵ\n");

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
		//������ƶ�<������ƶȾ����1/3,�����sift��
		float theDistance = matchesVectorInOpenCV[i].distance;
		bool bDistance = theDistance < standardDist;  //�ж����ƶ��Ƿ����
		bool bWindowSizeFitX = (diffX < -17) && (diffX > -37 );		//�жϹ��˴���x��С�Ƿ����
		bool bWindowSizeFitY = (diffY < 10) && (diffY > -10);		//�жϹ��˴���y��С�Ƿ����

		if (bDistance && bWindowSizeFitX && bWindowSizeFitY)
		{	
			//�ڵ�
			corlinerPointVec1InOpenCVOfTheZone.push_back(ptFirst);
			corlinerPointVec2InOpenCVOfTheZone.push_back(ptSecond);
			fprintf(fp, "    %-15d%-15d%-10.3f%-25.3f%-10.3f%-25.3f%-10.3f%-25.3f%-25.3f\n",
				idFirst, idSecond, firstPtX, firstPtY, secondPtX, secondPtY, diffX, diffY, theDistance);
		}
	
	}

	fclose(fp);

}
//��Ӧ��ԴӾֲ�����õ�ָ��tif��ȫ���������Ӧ���
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
	//��.tif�ļ��õ���Ӧ��vector������
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

	//��ֵ����ţ����꣬�̣߳�
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
//��8λ�������еõ�opencvͼ����  ���Ƿ����ֱ����opencv��ȡ8λ.tif?)
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
//�������׼����
void siftProcess::computeRoughMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, 
	boost::shared_ptr<pcl::Correspondences> cor)
{
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_east;
	//trans_east.estimateRigidTransformation(*_colinerCloud1, *_colinerCloud2, *_cor_inliers_ptr, _roughMatrix);
	trans_east.estimateRigidTransformation(*cloud1, *cloud2, *cor, _roughMatrix);
}

//ͨ����ά��������ת��Ϊ����
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getPointCloudFrom3dVec(std::vector<Pt3> vec3)
{
	//д����(�ֲ���
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
//ͨ�����ƺ�ƫ������ת��Ϊ�µ���
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getPointCloudFromPointCloudAndDeltaXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud,
	double deltaX, double deltaY, double deltaZ)
{
	//д����(�ֲ���
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
	//д����(�ֲ���
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

//������С�������
boost::shared_ptr<pcl::Correspondences> siftProcess::computeMiniCorrisponces(int sizeofColiner)
{
	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);
	//��ֵ���ڵ�
	for (size_t i = 0; i < sizeofColiner; i++)
	{
		pcl::Correspondence theCorr;
		theCorr.index_query = i;
		theCorr.index_match = i; 
		cor_inliers_ptr->push_back(theCorr);
	}
	return cor_inliers_ptr;
}

//���ֵ����
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getDiffPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
	int minSize = cloud1->points.size();
	if ( minSize > cloud2->points.size() )
	{
		minSize = cloud2->points.size();
	}
	//д����(�ֲ���
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

//��32λ��������ת��Ϊ8�������С�
std::vector<uchar> siftProcess::convert32bitPixelVectorTo8bitPixelVector(std::vector<float> pixelVector32bit)
{
	//0,��ʼ��һ���յ�8λ��������
	std::vector<uchar> pixelVector8bit;
	pixelVector8bit.clear();

	//1���ж�32λ���������Ƿ�Ϊ�գ����򷵻�
	int sizeof32Bit = pixelVector32bit.size();
	if (sizeof32Bit == 0)
	{
		pixelVector8bit.clear();
		return pixelVector8bit;
	}

	//2������32λ�������е����ֵ����Сֵ
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
	//3�������Сֵ�����ֵ��ȣ����趨Ϊԭֵ
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

	//4����32λ�������е����ֵ����Сֵ��Χӳ�䵽0-255
	double lengthOf32bit = maxPixel32bit - minPixel32bit;

	//5����32λ�������е�����ֵ��0-255������ȡֵ
	for (size_t i = 0; i < sizeof32Bit; i++)
	{
		double thePixel32bit = pixelVector32bit[i];
		//32λƫ������
		double deltaLength32bit = thePixel32bit - minPixel32bit;
		//���㻻�����8bit����
		int thePixel8bit = 255 * deltaLength32bit/lengthOf32bit;
		if (thePixel8bit > 255)
		{
			thePixel8bit = 255;
		}

		//5�����õ��ĸ���8λ���ط�������
		pixelVector8bit.push_back(thePixel8bit);
	}
	//6������8λ��������
	return pixelVector8bit;

}

//�������γ��µ��������ڴ���׼�������
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
//���ݶ���ȥ��Сֵ������������д���׼�ĵ��ƣ��Ծ��������������С�ھ����г˻���Ӱ��
void siftProcess::ajustVecByMinXYZ(std::vector<Pt3> inputVec1,
	std::vector<Pt3> inputVec2,
	std::vector<Pt3>& adjustVec1,
	std::vector<Pt3>& adjustVec2,
	double& minX,
	double& minY,
	double& minZ)
{
	//0�����ж��������������Ƿ�Ϊ��
	int sizeofVec1 = inputVec1.size();
	int sizeofVec2 = inputVec2.size();
	if ( sizeofVec1 == 0 || sizeofVec2 == 0)
	{
		return;
	}

	//1,�ֱ�������������е���Сֵ
	double minX1 = 0;
	double minY1 = 0;
	double minZ1 = 0;
	this->getMinXYZFromVec(inputVec1, minX1, minY1, minZ1);

	double minX2 = 0;
	double minY2 = 0;
	double minZ2 = 0;
	this->getMinXYZFromVec(inputVec2, minX2, minY2, minZ2);
	//2,�����������е���Сֵ�ó���С��ֵ
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
	//3,�ó������������
	adjustVec1 = this->getAjustVecFromVecAndDelta(inputVec1, minX, minY, minZ);
	adjustVec2 = this->getAjustVecFromVecAndDelta(inputVec2, minX, minY, minZ);
	
}

//���ݶ���ȥ�м�ֵ������������д���׼�ĵ��ƣ��Ծ��������������С�ھ����г˻���Ӱ��
void siftProcess::ajustVecByMidXYZ(std::vector<Pt3> inputVec1,
	std::vector<Pt3> inputVec2,
	std::vector<Pt3>& adjustVec1,
	std::vector<Pt3>& adjustVec2,
	double& midX,
	double& midY,
	double& midZ)
{
	//0�����ж��������������Ƿ�Ϊ��
	int sizeofVec1 = inputVec1.size();
	int sizeofVec2 = inputVec2.size();
	if (sizeofVec1 == 0 || sizeofVec2 == 0)
	{
		return;
	}

	//1,�ֱ�������������е���Сֵ
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

	//2,�����������е��м�ֵ������ֵ
	midX = (midX1 + midX2) / 2;
	midY = (midY1 + midY2) / 2;
	midZ = (midZ1 + midZ2) / 2;

	//3,�ó������������
	adjustVec1 = this->getAjustVecFromVecAndDelta(inputVec1, midX, midY, midZ);
	adjustVec2 = this->getAjustVecFromVecAndDelta(inputVec2, midX, midY, midZ);

}
//���������vector�������Сx,y,zֵ
void siftProcess::getMinXYZFromVec(std::vector<Pt3> vec, double& minX, double& minY, double& minZ)
{
	//1,���ж������vec�Ƿ�Ϊ�գ�����ֱ�ӷ���
	int sizeOfVec = vec.size();
	if (sizeOfVec == 0)
	{
		return;
	}

	//2��������м��㡣��Сֵ����ֵΪ��һ��
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

//��������������
std::vector<Pt3> siftProcess::getAjustVecFromVecAndDelta(std::vector<Pt3> vec, double minX, double minY, double minZ)
{
	std::vector<Pt3> adjustVec;
	adjustVec.clear();

	//1,�ж�����������Ƿ�Ϊ�գ����򷵻ؿ�
	int sizeofInputVec = vec.size();
	if (sizeofInputVec == 0)
	{
		adjustVec.clear();
		return adjustVec;
	}

	//2,��Ϊ��ʱ�����ֵ
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
  
//����׼
Eigen::Matrix4f siftProcess::ICPRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud)
{
	//׼��ICP����׼
	//��ʹ��֮ǰҪ����set���������� 
	//1,setMaximumIterations�� ������������icp��һ�������ķ�������������Щ�Σ�

	//2,setTransformationEpsilon�� ǰһ���任����͵�ǰ�任����Ĳ���С����ֵʱ������Ϊ�Ѿ������ˣ���һ������������

	//3,setEuclideanFitnessEpsilon�� ����һ�����������Ǿ�������С����ֵ�� ֹͣ������

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

	//��ֵ����
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
	//�������վ���
	Eigen::Matrix4f finalMatrix = icp.getFinalTransformation();
	return finalMatrix;

}

//����ƽ�ƻ���
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getCloudFromAdjustCloudAndMinXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud,
	double minX, double minY, double minZ)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	originalCloud->clear();

	//1,�ж�������Ƹ�����0�򷵻ؿ�
	int sizeOfInputCloud = adjustCloud->points.size();
	if ( sizeOfInputCloud == 0)
	{
		originalCloud->points.clear();
		return originalCloud;
	}
	//2���Ե���ÿ��ֵ������+(minX,minY,minZ),����ֵ���µ���

	originalCloud->height = 1;
	originalCloud->width = sizeOfInputCloud;
	originalCloud->resize(originalCloud->width * originalCloud->height);
	for (size_t i = 0; i < originalCloud->size(); i++)
	{
		originalCloud->points[i].x = adjustCloud->points[i].x + minX;
		originalCloud->points[i].y = adjustCloud->points[i].y + minY;
		originalCloud->points[i].z = adjustCloud->points[i].z + minZ;

	}

	//3,�����µ���
	return originalCloud;
}

//����id�Ϳ��ȡ��
double siftProcess::getPixelFromID(int xID, int yID, int widthRoi, std::vector<float> pixel32BitVec)
{
	int theID = xID + yID * widthRoi;
	double thePixel = pixel32BitVec[theID];
	return thePixel;
}
//����������ͼ��ģ�xSize,ySize,widthRoi,HeightRoi)ȷ��(xroi,yroi,widthRoitrue,heightRoitrue)
std::vector<std::vector<zone>> siftProcess::getZoneVecVec(int xSize, int ySize, int widthRoi, int heightRoi)
{
	std::vector<std::vector<zone>> zoneVecVec;
	zoneVecVec.clear();

	//�ֿ�
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

	//�ֿ�
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

//���ص�һ�������Ӧ�Ķ�Ӧ�ڵ���ά��������
void siftProcess::getColinersFromZone(zone theZone1, tifParameter tif1, tifParameter tif2, 
	std::vector<Pt3>& colinersOfTheZoneVec1, std::vector<Pt3>& colinersOfTheZoneVec2)
{
	//zone1��������Сid����Ϊ0
	int xID1 = theZone1.xRoi;
	int yID1 = theZone1.yRoi;
	if (xID1 < 0 || yID1 < 0)
	{
		return;
	}
	//1,���ݵ�һ��ͼ��ÿ�����ȷ���ڶ���ͼ��ķ���
	zone zone2;
	util::getZone2FromZone1(theZone1, tif1, tif2, zone2);
	//zone2��������Сid����Ϊ0
	int xID2 = zone2.xRoi;
	int yID2 = zone2.yRoi;
	if (xID2 < 0 || yID2 < 0)
	{
		return;
	}

	//2,����.tif��Ӧλ����Ӧ�����32λ���ؼ���
	//����õ���Ӧ��ȡ�������������
	std::vector<float> rasterID32bitVecForSift1 = util::getSegRasterVecVecFromTif_32bit(_strImageFile1Name32bit.c_str(), theZone1);
	std::vector<float> rasterID32bitVecForSift2 = util::getSegRasterVecVecFromTif_32bit(_strImageFile2Name32bit.c_str(), zone2);

	//�������п��򷵻�
	int sizeof32BitSift1 = rasterID32bitVecForSift1.size();
	int sizeof32BitSift2 = rasterID32bitVecForSift2.size();
	if (sizeof32BitSift1 == 0 || sizeof32BitSift2 == 0)
	{
		return;
	}
	std::cout << "32bit size:(" << rasterID32bitVecForSift1.size() << "," << rasterID32bitVecForSift2.size() << ")" << std::endl;
	//3����32λ��������ת��Ϊ8�������С�
	std::vector<uchar> rasterID8bitVecForSift1 = this->convert32bitPixelVectorTo8bitPixelVector(rasterID32bitVecForSift1);
	std::vector<uchar> rasterID8bitVecForSift2 = this->convert32bitPixelVectorTo8bitPixelVector(rasterID32bitVecForSift2);
	//�������п��򷵻�
	int sizeof8BitSift1 = rasterID8bitVecForSift1.size();
	int sizeof8BitSift2 = rasterID8bitVecForSift2.size();
	if (sizeof8BitSift1 == 0 || sizeof8BitSift2 == 0)
	{
		return;
	}
	std::cout << "8bit size:(" << rasterID8bitVecForSift1.size() << "," << rasterID8bitVecForSift2.size() << ")" << std::endl;
	//4��������������䵽opencvͼ��
	cv::Mat srcImage8BitForSift1 = this->getOpenCVImgFrom8bitVec(rasterID8bitVecForSift1,theZone1);
	cv::Mat srcImage8BitForSift2 = this->getOpenCVImgFrom8bitVec(rasterID8bitVecForSift2, zone2);

	//5,����ֱ��ͼ
	cv::Mat heqResult1;
	cv::equalizeHist(srcImage8BitForSift1, heqResult1);
	cv::Mat heqResult2;
	cv::equalizeHist(srcImage8BitForSift2, heqResult2);

	//6����opencv��sift�㷨��ȡ������ͼ���sift���к��ڵ����У����Ǿֲ�������ţ�
	// ��һ��Դͼ���OpenCV��ά�ڵ�����
	std::vector<cv::Point2f> corlinerPointVec1InOpenCVOfTheZone;
	corlinerPointVec1InOpenCVOfTheZone.clear();
	// �ڶ���Դͼ���OpenCV��ά�ڵ�����
	std::vector<cv::Point2f> corlinerPointVec2InOpenCVOfTheZone;
	corlinerPointVec2InOpenCVOfTheZone.clear();

	this->getSIFTFeatureFromOpenCVImage8bit(srcImage8BitForSift1, 
		srcImage8BitForSift2, 
		corlinerPointVec1InOpenCVOfTheZone, 
		corlinerPointVec2InOpenCVOfTheZone);
	//this->getSIFTFeatureFromOpenCVImage8bit(heqResult1, heqResult2);
	//7���������ֲ���������ת����ά����
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

	//8�������ʱ����
	rasterID32bitVecForSift1.clear();
	rasterID32bitVecForSift2.clear();
	rasterID8bitVecForSift1.clear();
	rasterID8bitVecForSift2.clear();

}
//ѡȡһ���ֵ��ƽ��д���
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud, float ratioOfDataSize )
{
	//д����(�ֲ���
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
//�ӵ����еõ�����
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

//��Ŀǰ��԰��ձ�����ȡ�µ�ԡ�
void siftProcess::filterColiner(std::vector<Pt3> inputVec1, std::vector<Pt3> inputVec2,
	std::vector<Pt3>& outputVec1, std::vector<Pt3>& outputVec2,
	float ratioFilter)
{
	//1,����������д�СΪ�գ��򷵻�
	int sizeofVec1 = inputVec1.size();
	int sizeofVec2 = inputVec2.size();
	if ( sizeofVec1 == 0 || sizeofVec2 == 0)
	{
		return;
	}
	//2�������ֵ���γ�һ������vecdiff��(vec1,vec2,vecdiff)
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
	//2����vecdiff����xֵ����ɸ����ű���ratioFilter�����ֵ��ǰ������
	std::sort(diffVecVec.begin(), diffVecVec.end(), util::greaterSortX);
	std::vector<diffVec> diffVecVecX = this->getSortDiffVec(diffVecVec, ratioFilter);
	diffVecVec.clear();
	//3����vecdiff����yֵ����ɸ����ű���ratioFilter�����ֵ��ǰ������
	std::sort(diffVecVecX.begin(), diffVecVecX.end(), util::greaterSortY);
	std::vector<diffVec> diffVecVecY = this->getSortDiffVec(diffVecVecX, ratioFilter);
	diffVecVecX.clear();

	//4����vecdiff����zֵ����ɸ����ű���ratioFilter�����ֵ��ǰ������
	std::sort(diffVecVecY.begin(), diffVecVecY.end(), util::greaterSortZ);
	std::vector<diffVec> diffVecVecZ = this->getSortDiffVec(diffVecVecY, ratioFilter);
	diffVecVecY.clear();

	//5�����ع��˺�ĵ������
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

//���ݱ��������µĹ��˺��vec
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