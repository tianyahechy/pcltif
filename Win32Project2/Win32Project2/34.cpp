
#include "common2.h"

int main()
{
	//记录开始时的当前时间

	time_t startTime, endTime;
	double diff;
	time(&startTime);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	//读取点云数据
	std::cout << "读取点云数据:" << std::endl;
	reader.read("d:\\1_xyz.pcd", *cloud);
	pt3Set vector3Set;
	vector3Set.clear();
	//for (size_t i = 0; i < cloud->points.size(); i++)
	for (size_t i = 0; i < 20000; i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;
		Pt3 theVector(Pt3(x, y, z));
		vector3Set.insert(pt3Pair(i, theVector));
		//std::cout << "第" << i << "个数据：(" <<theVector.x() << "," << theVector.y() << "," << theVector.z() << ")" << std::endl;
	}

	//根据点云集求点云的细节
	PCLDetail theDetail = getPCLDetail(vector3Set);
	
	//设置X，Y方向的像素分辨率
	double xResolution = 1.0;
	double yResolution = 1.0;

	//求X,Y方向的像素个数,
	double distanceX = theDetail.xDistance;
	double distanceY = theDetail.yDistance;
	double xSizef = distanceX / xResolution;
	double ySizef = distanceY / yResolution;
	//取整
	int xSize = (int)xSizef;
	int ySize = (int)ySizef;

	//设置左上角为minx,maxY
	double minX = theDetail.minX;
	double minY = theDetail.minY;
	
	//根据输入点集合来等分XY间距，得到图像坐标集合vector
	rasterVec2Set imagePointVector2Set = getEqualXYVectorFromDataSet(vector3Set, minX, minY, xSize, ySize,xResolution,yResolution);
	
	//三角化数据集合中的X,Y坐标,重组带Z值的三角形集合
	std::cout << "三角化数据集合中的X,Y坐标,重组带Z值的三角形集合" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);
	rasterVec3Set imageSet = getPt3SetFromDataSetAndPt2Vector(vector3Set, imagePointVector2Set,myTriangleSet);
	
	//创建网格文件
	const char * pszRasterFile = "E:\\PCLOutPut20000Time.tif";
	int bandSize = 1;
	float maxY = theDetail.maxY;
	//设定图像左上角
	float topLeftX = minX;
	float topLeftY = maxY;
	createRasterFile(pszRasterFile, bandSize, xSize, ySize, xResolution, yResolution, topLeftX, topLeftY);
	//更新网格文件,生成.tiff文件
	UpdateRasterFile(pszRasterFile, imageSet);
	
	//记录结束时间
	time(&endTime);
	diff = difftime(endTime, startTime);
	std::cout << "总共花费时间" << diff << "秒" << std::endl;
	/*
	double testZ = -1;
	//得到三角面集合。提取出来，
	//三角化数据集合中的X,Y坐标,重组带Z值的三角形集合
	std::cout << "三角化数据集合中的X,Y坐标,重组带Z值的三角形集合" << std::endl;
	triangleSet myTriangleSet = getTriangleSetFromDataSet(vector3Set);

	//得到图像点的坐标XYZ
	std::cout << "得到图像点的坐标XYZ" << std::endl;

	//输出到文件
	FILE * file = fopen("e:\\outputTriangleAndPoint", "w");

	for (int x = 118; x < 119; x++)
	{
		for (int y = 70; y < 71; y++ )
		{
			double theX = minX + xResolution * x;
			double theY = minY + yResolution * y;
			Pt2 thisPoint = Pt2(theX, theY);
			
			Pt3 thept3(0, 0, 0);
			std::vector<Pt3> pt3Vec;
			pt3Vec.clear();
			getPt3InsertAndPt3VecInOneTriangleFromDataSet(vector3Set, thisPoint, thept3, myTriangleSet, pt3Vec);
			std::cout << "大小为：" << pt3Vec.size() << std::endl;
	
			fprintf( file, "(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f,%0.6f),(%0.6f,%0.6f)\n",
				pt3Vec[0].x(), pt3Vec[0].y(), pt3Vec[0].z(),
				pt3Vec[1].x(), pt3Vec[1].y(), pt3Vec[1].z(),
				pt3Vec[2].x(), pt3Vec[2].y(), pt3Vec[2].z(),
				thept3.x(), thept3.y(), thept3.z(),thisPoint.x(), thisPoint.y());
		}
	}

	fclose(file);
	*/
	return 0;
}