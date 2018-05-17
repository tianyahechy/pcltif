/*
#include "JDEMRasterBand.h"
#include "JDEMDataSet.h"

JDEMRasterBand::JDEMRasterBand(JDEMDataSet * poDS, int nBand)
{
	this->poDS = poDS;
	this->nBand = nBand;
	eDataType = GDT_Float32;

	nBlockXSize = poDS->GetRasterXSize();
	nBlockYSize = 1;

	nRecordSize = nBlockXSize * 5 + 9 + 2;
	pszRecord = NULL;

}

JDEMRasterBand::~JDEMRasterBand()
{
}

CPLErr JDEMRasterBand::IReadBlock(int nBlockXOff, int nBlockYOff, void * pImage)
{
	JDEMDataSet * poGDS = (JDEMDataSet*)poDS;
	int i;
	
	if (pszRecord == NULL)
	{
		if ( nRecordSize < 0 )
		{
			return CE_Failure;
		}

		pszRecord = (char *)VSIMalloc(nRecordSize);
		if ( pszRecord == NULL )
		{
			CPLError(CE_Failure, CPLE_OutOfMemory, "申请一行数据内存错误");
			nRecordSize = -1;
			return CE_Failure;
		}
	}

	VSIFSeekL(poGDS->fp, 1011 + nRecordSize * nBlockYOff, SEEK_SET);
	VSIFRead(pszRecord, 1, nRecordSize, poGDS->fp);

	
}
*/