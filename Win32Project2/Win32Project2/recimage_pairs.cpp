#include "recimage_pairs.h"
#include "soe_envelope.h"
#include "soeImgInfo.h"

recimage_pairs::recimage_pairs()
{
	_null_value = 0;
	_xResolution = 0;
	_yResolution = 0;
	_topLeftX = 0;
	_topLeftY = 0;
	_point_clound.clear();
	_pointSet.clear();
	_difftime12 = 0;
	_difftime23 = 0;
	_difftime34 = 0;
}

recimage_pairs::~recimage_pairs()
{
	_point_clound.clear();
	_pointSet.clear();
	_totalRasterVec.clear();
}
//���õ㼯
void recimage_pairs::setPointSet(pt3Set dataSet, int startXID, int startYID, int widthRoi, int heightRoi)
{
	//�����ж������Ƿ�Ϊ�գ����򷵻�
	int theSize = dataSet.size();
	if ( theSize == 0 )
	{
		return;
	}
	//��һ��ֵ
	int id = 0;
	pt3Set::iterator
		iterCurPt = dataSet.begin(),
		iterEndPt = dataSet.end();
	for (; iterCurPt != iterEndPt; iterCurPt++)
	{
		Pt3 thePt = iterCurPt->second;
		float theX = thePt.x();
		float theY = thePt.y();
		float theZ = thePt.z();

		//���id��һ����Χ�ڣ�������
		int xID = (theX - _topLeftX) / _xResolution;
		int yID = (theY - _topLeftY) / _yResolution;
		if ( xID >= startXID && 			
			xID < (startXID + widthRoi) && 
			yID >= startYID &&
			yID < (startYID + heightRoi)
			
			)
		{
			_pointSet.insert(pt3Pair(id, thePt));
			id = id + 1;
		}

	}
}

//�����ܵ�դ������
void recimage_pairs::setTotalRasterVec(std::vector<std::vector<Pt3>> rasterVector)
{
	_totalRasterVec = rasterVector;
}
//���÷ֱ���
void recimage_pairs::setResolution(double xResolution,double yResolution )
{
	_xResolution = xResolution;
	_yResolution = yResolution;
}
//�����ܵ�դ������
std::vector<std::vector<Pt3>> recimage_pairs::getRasterVecVec3()
{
	return _totalRasterVec;
}
//��������դ������Ͻ�����
void recimage_pairs::setTopLeft(double topLeftX, double topLeftY)
{
	_topLeftX = topLeftX;
	_topLeftY = topLeftY;
}
//�ֲ�����
void recimage_pairs::processBySegment(int segmentSize)
{
	//�����ư�������ֿ飬�ֿ鴦��
	std::vector<epi_block> epi_blockVector;
	epi_blockVector.clear();
	_point_clound.clear();
	std::cout << "�ܵ���:" << _pointSet.size() << std::endl;
	time_t splitTimeBefor;
	time(&splitTimeBefor);
	this->splitVectorFromPointCloud(_pointSet, segmentSize, epi_blockVector, _point_clound);
	time_t splitTimeAfter;
	time(&splitTimeAfter);
	double difsplit = difftime(splitTimeAfter, splitTimeBefor);
	std::cout << "�ӵ㼯�ָ���Ҫʱ��" << difsplit << std::endl;
	int sizeofCloudVector = epi_blockVector.size();
	//����ÿ���ֵ���
	for (int i = 0; i < sizeofCloudVector; i++)
	{
		epi_block theEpiBlock = epi_blockVector[i];
		this->dtm_resample(_xResolution, theEpiBlock);
	}
	std::cout << "_difftime12 = " << _difftime12 << std::endl;
	std::cout << "_difftime23 = " << _difftime23 << std::endl;
	std::cout << "_difftime34 = " << _difftime34 << std::endl;
}				  
//�����Ƽ��Ϸָ����Ӧ������
void recimage_pairs::splitVectorFromPointCloud(pt3Set pointCloudDataSet,
	int segmentSize,
	std::vector<epi_block>& epi_blockVector,
	std::vector<std::vector<Pt3>>& cloudVector)
{
	//1,�ж��Ƿ�Ϊ��,���Ϊ�գ�����С�ڷ��������򷵻�
	int theSize = pointCloudDataSet.size();
	if ((theSize == 0) ||
		(theSize < segmentSize)
		)
	{
		return;
	}

	//2,���е��Ʒֿ�
	//����ÿ���С������������򲻱䣬�������������ÿ��Ԫ�ظ���+1
	int sizeOfEachSegment = theSize / segmentSize;
	int modNumber = theSize % segmentSize;
	if (modNumber != 0)
	{
		sizeOfEachSegment = sizeOfEachSegment + 1;
	}

	std::cout << "ÿ���С" << sizeOfEachSegment << std::endl;

	//3,��������ֱ�ֵ
	cloudVector.resize(segmentSize);
	pt3Set::iterator
		iterCurPt = pointCloudDataSet.begin(),
		iterEndPt = pointCloudDataSet.end();
	for (; iterCurPt != iterEndPt; iterCurPt++)
	{
		int id = iterCurPt->first;
		//�ڼ���
		int quadID = id / sizeOfEachSegment;
		Pt3 thePt = iterCurPt->second;
		epi_block theEpi;
		theEpi.block_index = quadID;
		theEpi.block_dem_path = "";
		epi_blockVector.push_back(theEpi);
		cloudVector[quadID].push_back(thePt);

		//std::cout << "id = " << id << std::endl;
		//std::cout << "����= <" << thePt.x() << "," << thePt.y() << "," << thePt.z() << std::endl;
		//std::cout << "��" << quadID << "��" << std::endl;
	}

}

//�������طֱ��ʺ���Ӧ���ֵĵ��ƣ��ɲ�ֳɲ�ͬ��ŵĵ��ƣ�
bool recimage_pairs::dtm_resample(SOE_64F dtm_cell_size, epi_block &block)
{
	time(&time1);
	SOE_32S pt_size = _point_clound[block.block_index].size();
	//std::cout << "��" << block.block_index << "�����Ϊ" << pt_size;
	if (pt_size == 0)
		return false;

	soe_envelope clound_extent;
	clound_extent.init(_point_clound[block.block_index]);

	SOE_64F pt_win_size = dtm_cell_size * 5;

	SOE_32S grid_col = static_cast<SOE_32S>((clound_extent.getMaxX() - clound_extent.getMinX()) / pt_win_size) + 1;
	SOE_32S grid_row = static_cast<SOE_32S>((clound_extent.getMaxY() - clound_extent.getMinY()) / pt_win_size) + 1;
	SOE_32S grid_size = grid_col * grid_row;

	SOE_32S *grid_pt_num = new SOE_32S[grid_size];
	SOE_32S** grid_pt_ids = new SOE_32S*[grid_size];
	memset(grid_pt_num, 0, sizeof(SOE_32S)*grid_size);
	memset(grid_pt_ids, 0, sizeof(SOE_32S *)*grid_size);

	SOE_32S i, j, grid_x, grid_y, grid_pos;
	for (i = 0; i != pt_size; ++i)
	{
		grid_x = static_cast<SOE_32S>((_point_clound[block.block_index][i].x() - clound_extent.getMinX()) / pt_win_size);
		grid_y = static_cast<SOE_32S>((_point_clound[block.block_index][i].y() - clound_extent.getMinY()) / pt_win_size);
		grid_pt_num[grid_y*grid_col + grid_x]++;
	}

	for (i = 0; i != grid_size; ++i)
	{
		grid_pt_ids[i] = new SOE_32S[grid_pt_num[i]];
		memset(grid_pt_ids[i], 0, sizeof(SOE_32S)*grid_pt_num[i]);
	}

	memset(grid_pt_num, 0, sizeof(SOE_32S)*grid_size);
	for (i = 0; i != pt_size; ++i)
	{
		grid_x = static_cast<SOE_32S>((_point_clound[block.block_index][i].x() - clound_extent.getMinX()) / pt_win_size);
		grid_y = static_cast<SOE_32S>((_point_clound[block.block_index][i].y() - clound_extent.getMinY()) / pt_win_size);
		grid_pos = grid_y * grid_col + grid_x;

		grid_pt_ids[grid_pos][grid_pt_num[grid_pos]] = i;
		grid_pt_num[grid_pos]++;
	}

	time(&time2);
	_difftime12 += difftime(time2, time1);

	SOE_32S dtm_width = static_cast<SOE_32S>((clound_extent.getMaxX() - clound_extent.getMinX()) / dtm_cell_size);
	SOE_32S dtm_height = static_cast<SOE_32S>((clound_extent.getMaxY() - clound_extent.getMinY()) / dtm_cell_size);
	soe_envelope dtm_extent;
	dtm_extent.init(clound_extent.getMinX(), clound_extent.getMinX() + dtm_width * dtm_cell_size, clound_extent.getMinY(), clound_extent.getMinY() + dtm_height * dtm_cell_size);

	std::vector<SOE_32S>find_ids;
	SOE_64F x, y;

	//SOE_32F *result_data = new SOE_32F[dtm_width * dtm_height];
	//memset(result_data, _null_value, sizeof(SOE_32F)*dtm_width * dtm_height);

	for (SOE_16U i = 0; i != dtm_height; ++i)
	{
		y = dtm_extent.getMaxY() - i * dtm_cell_size - dtm_cell_size / 2.0;

		int yID = (y - _topLeftY) / _yResolution;
		for (SOE_16U j = 0; j != dtm_width; ++j)
		{
			time(&time2);
			x = dtm_extent.getMinX() + j * dtm_cell_size + dtm_cell_size / 2.0;
			int xID = ( x - _topLeftX) / _xResolution;

			//std::cout << "xID = " << xID << ",yID=" << yID <<std::endl;

			SOE_32S grid_x2 = static_cast<SOE_32S>((x - clound_extent.getMinX()) / pt_win_size);
			SOE_32S grid_y2 = static_cast<SOE_32S>((y - clound_extent.getMinY()) / pt_win_size);

			SOE_32S find_iter = 0;
			find_ids.clear();
			for (SOE_32S m = -1; m != 2; ++m)
			{
				grid_y = grid_y2 + m;
				for (SOE_32S n = -1; n != 2; ++n)
				{
					grid_x = grid_x2 + n;
					grid_pos = grid_y * grid_col + grid_x;

					if (grid_x < 0 || grid_x >= grid_col || grid_y < 0 || grid_y >= grid_row)
						continue;

					for (SOE_32S k = 0; k < grid_pt_num[grid_pos]; k++)
					{
						find_ids.push_back(grid_pt_ids[grid_pos][k]);
						find_iter++;
					}

				}
			}
			//std::cout << "find_iter = " << find_iter << std::endl;
			double x_dis, y_dis, xy_dis[find_win_size];

			if (find_iter < find_win_size)
			{
				//result_data[i * dtm_width + j] = _null_value;
				continue;
			}

			for (SOE_32S m = 0; m != find_win_size; ++m)
			{
				x_dis = (_point_clound[block.block_index][find_ids[m]].x() - x) * (_point_clound[block.block_index][find_ids[m]].x() - x) +
					(_point_clound[block.block_index][find_ids[m]].y() - y) * (_point_clound[block.block_index][find_ids[m]].y() - y);

				for (SOE_32S n = m; n != find_iter; ++n)
				{
					y_dis = (_point_clound[block.block_index][find_ids[n]].x() - x) * (_point_clound[block.block_index][find_ids[n]].x() - x) +
						(_point_clound[block.block_index][find_ids[n]].y() - y) * (_point_clound[block.block_index][find_ids[n]].y() - y);

					if (x_dis > y_dis)
					{
						x_dis = y_dis;
						grid_pos = find_ids[m];
						find_ids[m] = find_ids[n];
						find_ids[n] = grid_pos;
					}
				}

				xy_dis[m] = x_dis;

			}

			time(&time3);
			_difftime23 += difftime(time3, time2);
			
			double result_height = 0.0, dis_total = 0.0;

			if (xy_dis[0] == 0)
			{
				result_height = _point_clound[block.block_index][find_ids[0]].z();
				dis_total = 1;

			}
			else
			{
				result_height = 0.0;
				dis_total = 0.0;
				for (SOE_32S m = 0; m != find_win_size; ++m)
				{

					result_height += _point_clound[block.block_index][find_ids[m]].z() * 1.0 / xy_dis[m];
					dis_total += 1.0 / xy_dis[m];

					//std::cout << "m = " << m << std::endl;
					//std::cout << "xy_dis[" << m << "]=" << xy_dis[m] << std::endl;
					//std::cout << "_point_clound[" <<block.block_index<<"][" <<find_ids[m]<<"].z() = " 
					//	<< _point_clound[block.block_index][find_ids[m]].z() << std::endl;
					//std::cout << "dis_total =" << dis_total << std::endl;
				}

			}
		//	std::cout << "result_height = " << result_height << "," << "dis_total=" << dis_total << std::endl;
			result_height /= dis_total;
			//result_data[i * dtm_width + j] = static_cast<SOE_32F>(result_height);
			float theX = _totalRasterVec[yID][xID].x();
			float theY = _totalRasterVec[yID][xID].y();
			float theZ = static_cast<SOE_32F>(result_height);
			Pt3 thePt(theX, theY, theZ);
			_totalRasterVec[yID][xID] = thePt;

			time(&time4);
			_difftime34 += difftime(time4, time3);

			//std::cout << "_difftime12 = " << _difftime12 << std::endl;
			//std::cout << "_difftime23 = " << _difftime23 << std::endl;
			//std::cout << "_difftime34 = " << _difftime34 << std::endl;
		}
	}

	//dtm_dataset.Close();
	delete[]grid_pt_num;
	for (i = 0; i != grid_size; ++i)
	{
		delete[]grid_pt_ids[i];
	}
	delete[]grid_pt_ids;

	return true;
}


//�õ��㼯
pt3Set recimage_pairs::getPointSet()
{
	return _pointSet;
}