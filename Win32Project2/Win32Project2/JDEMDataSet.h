/*
#pragma once
#include "cpl_port.h"
#include "gdal_frmts.h"
#include "gdal_pam.h"
#include <algorithm>
class JDEMRasterBand;
class JDEMDataSet : public GDALPamDataset
{
public:
	friend class JDEMRasterBand;
	VSILFILE * fp;
	GByte abyHeader[1012];

public:
	JDEMDataSet();
	~JDEMDataSet();

	static GDALDataset * Open(GDALOpenInfo *);
	static int Identify(GDALOpenInfo*);
	CPLErr GetGeoTransform(double * padfTransform) override;
	const char * GetProjectionRef() override;
};

*/