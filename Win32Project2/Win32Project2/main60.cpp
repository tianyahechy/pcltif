

#pragma once
#include "PCLTif.h"
#include "AfxUtil.h"
#include "ogrsf_frmts.h"


bool layerMethod(const char* pszSrcShp, const char * pszMethodShp, const char* pszDstShp, int iType)
{
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	OGRRegisterAll();
	//打开数据
	OGRDataSource * poSrcDS = OGRSFDriverRegistrar::Open(pszSrcShp, FALSE);
	if ( poSrcDS == NULL)
	{
		printf("open file [%s] failed.\n", pszSrcShp);
		return false;
	}

	OGRDataSource * poMethodDS = OGRSFDriverRegistrar::Open(pszMethodShp, FALSE);
	if ( poMethodDS == NULL )
	{
		printf("open file [%s] failed .\n", pszMethodShp);
		OGRDataSource::DestroyDataSource(poSrcDS);
		return false;
	}

	//创建数据
	OGRSFDriver * poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName("ESRI Shapefile");
	if ( poDriver == NULL )
	{
		printf("%s driver not available \n", "ESRI Shapefile");
		OGRDataSource::DestroyDataSource(poSrcDS);
		OGRDataSource::DestroyDataSource(poMethodDS);
		return false;
	}

	OGRDataSource * poDstDs = poDriver->CreateDataSource(pszDstShp, NULL);
	if ( poDstDs == NULL )
	{
		printf("Creation of output file failed\n");
		OGRDataSource::DestroyDataSource(poSrcDS);
		OGRDataSource::DestroyDataSource(poMethodDS);
		return false;
	}

	OGRLayer * poSrcLayer = poSrcDS->GetLayer(0);
	if (poSrcLayer == NULL)
	{
		printf("Creation of layer failed\n");
		OGRDataSource::DestroyDataSource(poSrcDS);
		OGRDataSource::DestroyDataSource(poMethodDS);
		OGRDataSource::DestroyDataSource(poDstDs);
		return false;
	}

	OGRLayer * poMethodLayer = poMethodDS->GetLayer(0);
	if (poMethodLayer == NULL)
	{
		printf("Creation of layer failed\n");
		OGRDataSource::DestroyDataSource(poSrcDS);
		OGRDataSource::DestroyDataSource(poMethodDS);
		OGRDataSource::DestroyDataSource(poDstDs);
		return false;
	}

	OGRSpatialReference * pSRS = poSrcLayer->GetSpatialRef();
	OGRLayer * poDstLayer = poDstDs->CreateLayer("Polygon", pSRS, wkbPolygon, NULL);
	if (poDstLayer == NULL)
	{
		printf("Creation of layer failed\n");
		OGRDataSource::DestroyDataSource(poSrcDS);
		OGRDataSource::DestroyDataSource(poMethodDS);
		OGRDataSource::DestroyDataSource(poDstDs);
		return false;
	}

	switch (iType)
	{
	case 0:
		poSrcLayer->Intersection(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;
	case 1:
		poSrcLayer->Union(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;
	case 2:
		poSrcLayer->SymDifference(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;
	case 3:
		poSrcLayer->Identity(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;
	case 4:
		poSrcLayer->Update(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;
	case 5:
		poSrcLayer->Clip(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;
	case 6:
		poSrcLayer->Erase(poMethodLayer, poDstLayer, NULL, GDALTermProgress, NULL);
		break;

	default:
		break;
	}
	OGRDataSource::DestroyDataSource(poSrcDS);
	OGRDataSource::DestroyDataSource(poMethodDS);
	OGRDataSource::DestroyDataSource(poDstDs);

	return true;
}


void main(int argc, char **argv)
{	
	/*
	//打开las文件
	std::ifstream ifs;
	ifs.open("E:\\DEM-2016.las", std::ios::in | std::ios::binary );
	
	liblas::ReaderFactory readerFactory;
	liblas::Reader reader = readerFactory.CreateWithStream(ifs);

	//写点云
	pcl::PointCloud<pcl::PointXYZ> cloudOutput;
	cloudOutput.clear();

	while ( reader.ReadNextPoint())
	{
		double x = reader.GetPoint().GetX();
		double y = reader.GetPoint().GetY();
		double z = reader.GetPoint().GetZ();

		pcl::PointXYZ thePt(x, y, z);
		cloudOutput.push_back(thePt);

	}

	cloudOutput.width = cloudOutput.size();
	cloudOutput.height = 1;
	cloudOutput.is_dense = false;
	cloudOutput.resize(cloudOutput.width * cloudOutput.height);

	pcl::io::savePCDFileASCII("E:\\DEM-2016_2.pcd", cloudOutput);

	cloudOutput.clear();
	*/
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader pcdreader;
	pcdreader.read("E:\\DEM-2016.pcd", *cloud);

	std::cout << "总数:" << cloud->points.size() << std::endl;
	//写liblas,
	
	std::ios::openmode m = std::ios::out | std::ios::in | std::ios::binary | std::ios::ate;
	
	std::ofstream ofs;
	if (!liblas::Create(ofs, "E:\\DEM-2016_3.las") )
	{
		std::cout << "无法创建.las" << std::endl;
		return;
	}
	ofs.close();
	
	//ofs.open("E:\\DEM-2016_3.las", m);
	std::ofstream * ofs2 = new std::ofstream("E:\\DEM-2016_3.las", m);
	if ( !ofs2->is_open())
	{
		std::cout << "打不开.las" << std::endl;
		return ;
	}
	else
	{
		std::cout << "能打开.las" << std::endl;
		
	}
	
	liblas::Header header;
	liblas::Writer writer(*ofs2, header);
	liblas::Point point(&header);

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;

		point.SetX(x);
		point.SetY(y);
		point.SetZ(z);
		writer.WritePoint(point);
		std::cout << x << "," << y << "," << z << std::endl;
	}
	

	ofs2->close();
	*/

}