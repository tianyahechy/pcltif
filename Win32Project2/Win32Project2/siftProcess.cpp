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

//�������в���
void siftProcess::processAll()
{
	//0,���������ͼ��Ĳ���,�Լ�ȫͼ������
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

	//1����32λͼ������ȡ��������
	_seg1Vector32bitForPCL = util::getSegRasterVecVecFromVecVec2(rastervecvec1, _xRoi1, _yRoi1, _widthRoi, _heightRoi);
	//�ӵ�һ��ͼ��ȡ��Χ�����Ͻ�(xRoi,yRoi)�������һ��ͼ��Ľ�ȡ��Χ�����Ͻ�(xRoi2,yRoi2)
	double inputTopLeftX = _seg1Vector32bitForPCL[0][0].x();
	double inputTopLeftY = _seg1Vector32bitForPCL[0][0].y();

	int xRoi2 = 0;
	int yRoi2 = 0;
	util::getRoil2FromRoi1AndTif(_xRoi1, _yRoi1, _widthRoi, _heightRoi, inputTopLeftX, inputTopLeftY,
		xResolution2, yResolution2, topLeftX2, topLeftY2,  xSize2,  ySize2,
		 xRoi2,  yRoi2);
	//�ڶ���ͼ������
	_seg2Vector32bitForPCL = util::getSegRasterVecVecFromVecVec2(rastervecvec2, xRoi2, yRoi2, _widthRoi, _heightRoi);

	//�õ����ص�����
	_rasterID32bitVecForSift1 = util::getPixel32bitFromTifVecVec(_seg1Vector32bitForPCL);
	_rasterID32bitVecForSift2 = util::getPixel32bitFromTifVecVec(_seg2Vector32bitForPCL);

	//2����32λ��������ת��Ϊ8�������С�
	_rasterID8bitVecForSift1 = this->convert32bitPixelVectorTo8bitPixelVector(_rasterID32bitVecForSift1);
	_rasterID8bitVecForSift2 = this->convert32bitPixelVectorTo8bitPixelVector(_rasterID32bitVecForSift2);
	//2��������������䵽opencvͼ��
	cv::Mat srcImage8BitForSift1 = this->getOpenCVImgFrom8bitVec(_rasterID8bitVecForSift1);
	cv::Mat srcImage8BitForSift2 = this->getOpenCVImgFrom8bitVec(_rasterID8bitVecForSift2);
	cv::imwrite("e:\\test\\srcImage8BitForSift1.jpg", srcImage8BitForSift1);
	cv::imwrite("e:\\test\\srcImage8BitForSift2.jpg", srcImage8BitForSift2);

	//3,����ֱ��ͼ
	cv::Mat heqResult1;
	cv::equalizeHist(srcImage8BitForSift1, heqResult1);
	cv::Mat heqResult2;
	cv::equalizeHist(srcImage8BitForSift2, heqResult2);

	//3����opencv��sift�㷨��ȡ������ͼ���sift���к��ڵ����У����Ǿֲ�������ţ�
	//this->getSIFTFeatureFromOpenCVImage8bit(srcImage8BitForSift1, srcImage8BitForSift2);
	this->getSIFTFeatureFromOpenCVImage8bit(heqResult1, heqResult2);
	//4���ó�����׼���
	_cor_inliers_ptr = this->computeMiniCorrisponces(_corlinerPointVec1InOpenCV, _corlinerPointVec2InOpenCV);
	std::cout << "cor_inliers_ptr����:" << _cor_inliers_ptr->size() << std::endl;
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
	
	//6���������ֲ���������ת����ά����
	_corlinerPointVec1InPCL = this->convertOpenCV2DVecToPCL3DVec(_corlinerPointVec1InOpenCV, _seg1Vector32bitForPCL);
	_corlinerPointVec2InPCL = this->convertOpenCV2DVecToPCL3DVec(_corlinerPointVec2InOpenCV, _seg2Vector32bitForPCL);

	//7,����ά����ת��Ϊ����

	_colinerCloud1 = this->getPointCloudFrom3dVec(_corlinerPointVec1InPCL);
	_colinerCloud2 = this->getPointCloudFrom3dVec(_corlinerPointVec2InPCL);
	_segCloud1 = this->getPointCloudFrom3dVec(_seg1Vector32bitForPCL);
	_segCloud2 = this->getPointCloudFrom3dVec(_seg1Vector32bitForPCL);
	pcl::io::savePCDFile("e:\\test\\_colinerCloud1.pcd", *_colinerCloud1);
	pcl::io::savePCDFile("e:\\test\\_colinerCloud2.pcd", *_colinerCloud2);

	//8����������д���׼�ĵ��ƣ����������������С�ھ����г˻���Ӱ��
	double minX = 0;
	double minY = 0;
	double minZ = 0;
	//�ȼ�������������
	std::vector<Pt3> adjustVec1;
	adjustVec1.clear();
	std::vector<Pt3> adjustVec2;
	adjustVec2.clear();
	this->ajustVec(_corlinerPointVec1InPCL, _corlinerPointVec2InPCL, adjustVec1, adjustVec2,minX, minY, minZ);
	//���ݵ���������еõ�������ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud1 = this->getPointCloudFrom3dVec(adjustVec1);
	pcl::io::savePCDFile("e:\\test\\adjustCloud1.pcd", *adjustCloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustCloud2 = this->getPointCloudFrom3dVec(adjustVec2);
	pcl::io::savePCDFile("e:\\test\\adjustCloud2.pcd", *adjustCloud2);

	//9��ʹ��pcl���㷨�ó�����׼����
	this->computeRoughMatrix(adjustCloud1, adjustCloud2,_cor_inliers_ptr);
	//�������׼����
	std::cout << "�������׼����" << std::endl;
	std::cout << _roughMatrix << std::endl;
	//���ݴ���׼����ó�����׼����
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjustRoughCloud = this->getRoughPointCloudFromMatrix(adjustCloud1, _roughMatrix);
	pcl::io::savePCDFile("e:\\test\\adjustRoughCloud.pcd", *adjustRoughCloud);

	//10����icp�ó�����׼����
	Eigen::Matrix4f detailMatrix = this->ICPRegistration(adjustRoughCloud, adjustCloud2);
	//������վ���
	std::cout << "�������׼����" << std::endl;
	std::cout << detailMatrix << std::endl;

	//11,����ƽ�ƻ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud = this->getCloudFromAdjustCloudAndMinXYZ(adjustRoughCloud, minX, minY, minZ);
	pcl::io::savePCDFile("e:\\test\\finalCloud.pcd", *finalCloud);
	
}

//���ݴ���׼����ó�����׼����
pcl::PointCloud<pcl::PointXYZ>::Ptr siftProcess::getRoughPointCloudFromMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	Eigen::Matrix4f transformation_matrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered_Raw(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in_filtered_Raw->height = 1;
	cloud_in_filtered_Raw->width = 100000;
	cloud_in_filtered_Raw->resize(cloud_in_filtered_Raw->width * cloud_in_filtered_Raw->height);
	std::cout << "cloud_in_filtered_Rawת��ǰ������" << cloud_in_filtered_Raw->size() << std::endl;
	pcl::transformPointCloud(*inputCloud, *cloud_in_filtered_Raw, transformation_matrix);
	std::cout << "cloud_in_filtered_Rawת���������" << cloud_in_filtered_Raw->size() << std::endl;

	return cloud_in_filtered_Raw;
}
//��ά�������ת��ά����
Pt3 siftProcess::convertOpenCV2DToPCL3D(cv::Point2f point2d, std::vector<std::vector<Pt3>> segVec3d)
{
	//����ά�����i,jת��Ϊ��ά�е�����
	int xID = point2d.x;
	int yID = point2d.y;
	Pt3 thePt = segVec3d[yID][xID];
	return thePt;

}
//��ά�������vectorת��ά����vector
std::vector<Pt3> siftProcess::convertOpenCV2DVecToPCL3DVec(std::vector<cv::Point2f> point2dVec, std::vector<std::vector<Pt3>> segVec3d)
{
	std::vector<Pt3> pcl3DVec;
	pcl3DVec.clear();
	//1,�����ж϶�ά���к���ά�����Ƿ�Ϊ��
	if (point2dVec.size() == 0 || segVec3d.size() == 0)
	{
		pcl3DVec.clear();
		return pcl3DVec;
	}
	//2������ά�����е�����ת��Ϊpt3
	std::vector<cv::Point2f>::iterator
		iterCurVec = point2dVec.begin(),
		iterEndVec = point2dVec.end();
	for (; iterCurVec != iterEndVec; iterCurVec++)
	{
		//��ÿ��cv::Point2fת��ΪPt3
		cv::Point2f thePoint2 = *iterCurVec;
		Pt3 thePoint3 = convertOpenCV2DToPCL3D(thePoint2, segVec3d);
		pcl3DVec.push_back(thePoint3);
	}

	return pcl3DVec;
}
//����ͼ���е�SIFT������ƥ��,�ó���Ӧ�����ŵ�vector,
cv::Mat siftProcess::getSIFTFeatureFromOpenCVImage8bit(cv::Mat srcImage1, cv::Mat srcImage2 )
{
	//����sift������
	//cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create(0, 6, 0.08, 15, 1.0);
	cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

	//���ҹؼ���
	std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
	f2d->detect(srcImage1, keyPoints_1);
	f2d->detect(srcImage2, keyPoints_2);

	//����������
	cv::Mat descriptor_1, descriptor_2;
	f2d->compute(srcImage1, keyPoints_1, descriptor_1);
	f2d->compute(srcImage2, keyPoints_2, descriptor_2);

	//������ƥ��
	cv::BFMatcher matcher;
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
		for (size_t i = 0; i < theMatch.size(); i++)
		{
			matchesVectorInOpenCV.push_back(theMatch[i]);
		}

	}
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

		//�ڵ�
		_colinerVectorInOpenCV.push_back(matchesVectorInOpenCV[i]);
		_corlinerPointVec1InOpenCV.push_back(ptFirst);
		_corlinerPointVec2InOpenCV.push_back(ptSecond);
		fprintf(fp, "    %-15d%-15d%-10.3f%-25.3f%-10.3f%-25.3f%-10.3f%-25.3f%-25.3f\n", idFirst, idSecond, firstPtX, firstPtY, secondPtX, secondPtY, diffX, diffY, theDistance);

	}

	fclose(fp);
	//�õ�ת������
	cv::Mat matchMat;
	cv::drawMatches(srcImage1, keyPoints_1, srcImage2, keyPoints_2, matchesVectorVector, matchMat);

	//�޳���һ��
	cv::Mat finalMat = cv::findHomography(_corlinerPointVec1InOpenCV, _corlinerPointVec2InOpenCV, cv::RANSAC);
	std::cout << finalMat << std::endl;
	return matchMat;

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
boost::shared_ptr<pcl::Correspondences> siftProcess::computeMiniCorrisponces(std::vector<cv::Point2f> firstIDVector, std::vector<cv::Point2f> secondIDVector)
{
	//������С����
	int minSize = firstIDVector.size();
	if (minSize > secondIDVector.size())
	{
		minSize = secondIDVector.size();
	}

	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);

	//��ֵ���ڵ�
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

//����opencv��ά����õ���Ӧ���
boost::shared_ptr<pcl::Correspondences> siftProcess::getCorrisponcesByOpenCVlocalCoordinate(std::vector<cv::Point2f> firstIDVector, std::vector<cv::Point2f> secondIDVector)
{
	//������С����
	int minSize = firstIDVector.size();
	if (minSize > secondIDVector.size())
	{
		minSize = secondIDVector.size();
	}

	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);

	//��ֵ���ڵ�
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

//opencvת��pcl���
boost::shared_ptr<pcl::Correspondences> siftProcess::getCorrisponcesByOpenCVMatcher(std::vector<cv::DMatch> matchesVectorInOpenCV)
{
	int theSize = matchesVectorInOpenCV.size();
	boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr(new pcl::Correspondences);

	//��ֵ���ڵ�
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
	
	//3�������Сֵ�����ֵ��ȣ��򷵻ؿ�
	if (minPixel32bit == maxPixel32bit)
	{
		pixelVector8bit.clear();
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

//��������д���׼�ĵ��ƣ����������������С�ھ����г˻���Ӱ��
void siftProcess::ajustVec(std::vector<Pt3> inputVec1,
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