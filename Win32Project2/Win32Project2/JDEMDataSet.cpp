/*
#include "JDEMDataSet.h"
#include "JDEMRasterBand.h"

static int JDEMGetField(const char* pszField, int nWidth)
{
	char szWork[32] = {};
	CPLAssert(nWidth < static_cast<int>(sizeof(szWork)));
	strncpy(szWork, pszField, nWidth);
	szWork[nWidth] = '\0';
	return atoi(szWork);
}

static double JDEMGetAngle(const char* pszField)
{
	const int nAngle = JDEMGetField(pszField, 7);
	const int nDegree = nAngle / 10000;
	const int nMin = (nAngle / 100) % 100;
	const int nSec = nAngle % 100;

	return nDegree + nMin / 60.0 + nSec / 3600.0;
}

JDEMDataSet::JDEMDataSet()
{
}

JDEMDataSet::~JDEMDataSet()
{
}

GDALDataset * JDEMDataSet::Open(GDALOpenInfo * poOpenInfo)
{
	if (!Identify(poOpenInfo))
	{
		return NULL;
	}

	//判断该类型是否支持写入
	if (poOpenInfo->eAccess == GA_Update)
	{
		CPLError(CE_Failure, CPLE_NotSupported, "JDEM格式不支持更新模式.\n");
		return NULL;
	}

	//创建GDAL数据集
	JDEMDataSet * poDS = new JDEMDataSet();
	poDS->fp = VSIFOpenL(poOpenInfo->pszFilename, "rb");
	if (poDS->fp == NULL)
	{
		delete poDS;
		return NULL;
	}

	//读取文件头
	VSIFReadL(poDS->abyHeader, 1, 1012, poDS->fp);
	poDS->nRasterXSize = JDEMGetField((char*)poDS->abyHeader + 23, 3);
	poDS->nRasterYSize = JDEMGetField((char*)poDS->abyHeader + 26, 3);

	if ( poDS->nRasterXSize <= 0 || poDS->nRasterYSize <= 0 )
	{
		CPLError(CE_Failure, CPLE_AppDefined, "%d x %d", poDS->nRasterXSize, poDS->nRasterYSize);
		delete poDS;
		return NULL;
	}

	//创建一个波段对象
	poDS->SetBand(1, new JDEMRasterBand(poDS, 1));

	//初始化pam信息
	poDS->SetDescription(poOpenInfo->pszFilename);
	poDS->TryLoadXML();

	poDS->oOvManager.Initialize(poDS, poOpenInfo->pszFilename);

	return poDS;

}
*/