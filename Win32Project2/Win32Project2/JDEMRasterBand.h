/*
#pragma once
#include "cpl_port.h"
#include "gdal_frmts.h"
#include "gdal_pam.h"
#include <algorithm>
class JDEMDataSet;
class JDEMRasterBand : public GDALPamRasterBand
{
	friend class JDEMDataSet;
	int nRecordSize;
	char * pszRecord;

public:
	JDEMRasterBand(JDEMDataSet*, int);
	~JDEMRasterBand();

	virtual CPLErr IReadBlock(int, int, void *) override;
};

*/