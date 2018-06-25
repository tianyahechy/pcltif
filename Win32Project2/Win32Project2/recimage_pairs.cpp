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
}

recimage_pairs::~recimage_pairs()
{
	_point_clound.clear();
	_pointSet.clear();
	_totalRasterVec.clear();
}
/*
//�������طֱ����Լ��ÿ���ƣ����ò��ֵ������Ϊ.tif
bool recimage_pairs::dtm_resample(SOE_64F dtm_cell_size, epi_block & block)
{
	//�жϵ��ƴ�С�Ƿ�Ϊ��,���򷵻�false
	SOE_32S pt_size = _point_clound[block.block_index].size();
	if ( pt_size == 0 )
	{
		return false;
	}

	//ȷ���ÿ���Ƶķ�Χ
	soe_envelope clound_extent;
	clound_extent.init(_point_clound[block.block_index]);

	//5*5���γ�һ��,����ÿ������طֱ���
	SOE_64F pt_win_size = dtm_cell_size * 5;
	//����ÿ����ÿ��ÿ�и��ж����飬�Լ���������
	SOE_32S grid_col = static_cast<SOE_32S> ((clound_extent.getMaxX() - clound_extent.getMinX()) / pt_win_size) + 1;
	SOE_32S grid_row = static_cast<SOE_32S> ((clound_extent.getMaxX() - clound_extent.getMinX()) / pt_win_size) + 1;
	SOE_32S grid_size = grid_col * grid_row;
	
	//��ÿ��ĵ�����id�����ռ䣬���趨��ֵΪ0
	SOE_32S * grid_pt_num = new SOE_32S[grid_size];
	SOE_32S ** grid_pt_ids = new SOE_32S*[grid_size];
	memset(grid_pt_num, 0, sizeof(SOE_32S) *grid_size);
	memset(grid_pt_ids, 0, sizeof(SOE_32S*)*grid_size);

	SOE_32S grid_x, grid_y, grid_pos;

	//������Ƹ����ڸ����xy�������,�������������ĸ���+1
	for (size_t i = 0; i < pt_size; i++)
	{
		grid_x = static_cast<SOE_32S> ((_point_clound[block.block_index][i].x -
			clound_extent.getMinX()) / pt_win_size);
		grid_y = static_cast<SOE_32S> ((_point_clound[block.block_index][i].y -
			clound_extent.getMinY()) / pt_win_size);
		grid_pt_num[grid_y*grid_col + grid_x]++;
	}

	//��ÿ���id���з���ռ�
	for (size_t i = 0; i < grid_size; i++)
	{
		grid_pt_ids[i] = new SOE_32S[grid_pt_num[i]];
		memset(grid_pt_ids[i], 0, sizeof(SOE_32S) * grid_pt_num[i]);
	}

	//�Ը�����������������Ϊ0
	memset(grid_pt_num, 0, sizeof(SOE_32S) * grid_size);	
	//������Ƹ����ڸ����xy�������,��ֵ��Ÿ������꣬
	//�������������ĸ���+1
	for (size_t i = 0; i < pt_size; i++)
	{
		grid_x = static_cast<SOE_32S> ((_point_clound[block.block_index][i].x -
			clound_extent.getMinX()) / pt_win_size);
		grid_y = static_cast<SOE_32S> ((_point_clound[block.block_index][i].y -
			clound_extent.getMinY()) / pt_win_size);
		grid_pos = grid_y*grid_col + grid_x;
		grid_pt_ids[grid_pos][grid_pt_num[grid_pos]] = i;
		grid_pt_num[grid_pos]++;
	}

	//����ͼ��ֱ��ʣ���ȸ߶ȣ�
	SOE_32S dtm_width = static_cast<SOE_32S> ((clound_extent.getMaxX() - clound_extent.getMinX()) / dtm_cell_size);
	SOE_32S dtm_height = static_cast<SOE_32S> ((clound_extent.getMaxY() - clound_extent.getMinY()) / dtm_cell_size);
	
	//��ʼ��ͼ��ķ�Χ
	soe_envelope dtm_extent;
	dtm_extent.init(clound_extent.getMinX(),
		clound_extent.getMinX() + dtm_width * dtm_cell_size,
		clound_extent.getMinY(),
		clound_extent.getMinY() + dtm_height * dtm_cell_size
		);

	//����ͼ�����Ϣ�����ߣ��Ҷȣ�������)
	SOE_IMG_INFO dsm_file_info;
	dsm_file_info.width = dtm_width;
	dsm_file_info.height = dtm_height;
	dsm_file_info.channel_num = 1;
	dsm_file_info.data_type = BT_32F;

	//ɾ��ָ���ļ�
	std::string dtm_path = block.block_dem_path;
	std::string strname = dtm_path.substr(0, dtm_path.find_last_of("."));
	std::string poly_name = strname + "_valid.polygon";
	remove(poly_name.c_str());

	//����.tif,���÷�Χ��ͶӰ
	ImgCore dtm_dataset;
	if ( !dtm_dataset.Create(dtm_path.c_str(), &dsm_file_info))
	{
		return false;
	}
	dtm_dataset.SetExtent(&dtm_extent);
	dtm_dataset.SetProjection(_wkt.c_str());

	std::vector<SOE_32S> find_ids;
	SOE_64F x, y;
	//Ϊ.tif�����ռ䲢��ʼ��Ϊ0
	SOE_32F * result_data = new SOE_32F[dtm_width * dtm_height];
	memset(result_data, _null_value, sizeof(SOE_32F) * dtm_width * dtm_height);
	
	for (SOE_16U i = 0; i < dtm_height; i++)
	{
		y = dtm_extent.getMaxY() - i * dtm_cell_size - dtm_cell_size / 2.0;
		for (SOE_16U j = 0; j < dtm_width; j++)
		{
			//�������㣨�����½ǵ�һ���������Ŀ�ʼ���������Ըõ�Ϊ����3*3,������id����find_ids,������+1
			x = dtm_extent.getMinX() + j * dtm_cell_size + dtm_cell_size / 2.0;
			SOE_32S grid_x2 = static_cast<SOE_32S> ((x - clound_extent.getMinX()) / pt_win_size);
			SOE_32S grid_y2 = static_cast<SOE_32S> ((y - clound_extent.getMinY()) / pt_win_size);
			SOE_32S find_iter = 0;
			find_ids.clear();
			for (SOE_32S m = -1; m < 2; m++)
			{
				grid_y = grid_y2 + m;
				for (SOE_32S n = -1; n < 2; n++)
				{
					grid_x = grid_x2 + n;
					grid_pos = grid_y * grid_col + grid_x;
					if ( grid_x < 0 ||
						grid_x >= grid_col ||
						grid_y < 0 ||
						grid_y >= grid_row
						)
					{
						continue;
					}

					for (SOE_32S k = 0; k < grid_pt_num[grid_pos]; k++)
					{
						find_ids.push_back(grid_pt_ids[grid_pos][k]);
						find_iter++;
					}
				}
			}
			SOE_64F x_dis, y_dis, xy_dis[find_win_size];
			if ( find_iter < find_win_size)
			{
				result_data[i*dtm_width + j] = _null_value;
				continue;
			}

			//��С��������x_dis������findids�Ķ�Ӧֵ����,����ֵ��xy_dis
			for (SOE_32S m = 0; m < find_win_size; m++)
			{
				x_dis = (_point_clound[block.block_index][find_ids[m]].x - x) *
					(_point_clound[block.block_index][find_ids[m]].x - x) +
					(_point_clound[block.block_index][find_ids[m]].y - y) *
					(_point_clound[block.block_index][find_ids[m]].y - y);
				for (SOE_32S n = m; n < find_iter; n++)
				{
					y_dis = (_point_clound[block.block_index][find_ids[n]].x - x) *
						(_point_clound[block.block_index][find_ids[n]].x - x) +
						(_point_clound[block.block_index][find_ids[n]].y - y) *
						(_point_clound[block.block_index][find_ids[n]].y - y);
					if ( x_dis > y_dis )
					{
						x_dis = y_dis;
						grid_pos = find_ids[m];
						find_ids[m] = find_ids[n];
						find_ids[n] = grid_pos;
					}
				}
				xy_dis[m] = x_dis;
			}

			SOE_64F result_height = 0.0, dis_total = 0.0;
			//����ۼӸ߶ȣ������ֵΪ���ո߶�
			if ( xy_dis[0] == 0 )
			{
				result_height = _point_clound[block.block_index][find_ids[0]].z;
				dis_total = 1;
			}
			else
			{
				result_height = 0.0;
				dis_total = 0.0;
				for (SOE_32S m = 0; m < find_win_size; m++)
				{
					result_height += _point_clound[block.block_index][find_ids[m]].z * 1.0 / xy_dis[m];
					dis_total += 1.0 / xy_dis[m];
				}
			}
			result_height /= dis_total;
			result_data[i*dtm_width + j] = static_cast<SOE_32F>(result_height);
		}
	}
	//д������
	dtm_dataset.DataWriteBand(1, 1, 1, dtm_width, dtm_height, result_data);

	//������Դ
	delete[] result_data;
	result_data = NULL;

	dtm_dataset.Close();
	delete[] grid_pt_num;
	for (size_t i = 0; i < grid_size; i++)
	{
		delete[] grid_pt_ids[i];
	}
	delete[] grid_pt_ids;
	return true;
}
*/
/*
//�������طֱ����Լ��ÿ���ƣ����ⲿ�ֵ������Ϊ.tif
bool recimage_pairs::dtm_resample(SOE_64F dtm_cell_size, epi_block& block)
{
	//�жϵ��ƴ�С�Ƿ�Ϊ�գ����򷵻�false
	SOE_32S pt_size = _point_clound[block.block_index].size();
	if ( pt_size == 0 )
	{
		return false;
	}
	//ȷ���ÿ���Ʒ�Χ
	soe_envelope clound_extent;
	clound_extent.init(_point_clound[block.block_index]);
	//5*5����һ�飬�ֿ�
	SOE_64F pt_win_size = dtm_cell_size * 5;
	//����õ��ư���֣��ж����ж����У��Լ���������
	SOE_32S grid_col = static_cast<SOE_32S>((clound_extent.getMaxX() -
		clound_extent.getMinX()) / pt_win_size) + 1;
	SOE_32S grid_row = static_cast<SOE_32S>((clound_extent.getMaxX() -
		clound_extent.getMinX()) / pt_win_size) + 1;
	SOE_32S grid_size = grid_col * grid_row;

	//��ÿ��ĵ����������÷����ж��ٵ㣩��id�����ռ䣬,���趨��ֵΪ0
	SOE_32S * grid_pt_num = new SOE_32S[grid_size];
	SOE_32S **grid_pt_ids = new SOE_32S*[grid_size];
	memset(grid_pt_num, 0, sizeof(SOE_32S) * grid_size);
	memset(grid_pt_ids, 0, sizeof(SOE_32S*) * grid_size);

	//�����ÿ���ƣ�����������ĸ����飬�����÷����ĸ���+1
	for (size_t i = 0; i < pt_size; i++)
	{
		SOE_32S grid_x = static_cast<SOE_32S> ((_point_clound[block.block_index][i].x - clound_extent.getMinX()) / pt_win_size);
		SOE_32S grid_y = static_cast<SOE_32S> ((_point_clound[block.block_index][i].y - clound_extent.getMinY()) / pt_win_size);
		grid_pt_num[grid_y * grid_col + grid_x]++;
	}

	//���շ��飬�Ե��Ƶĸ���ŷ���ռ�
	for (size_t i = 0; i < grid_size; i++)
	{
		grid_pt_ids[i] = new SOE_32S[grid_pt_num[i]];
		memset(grid_pt_ids[i], 0, sizeof(SOE_32S) * grid_pt_num[i]);
	}
	//��ÿ��ĵ���������0
	memset(grid_pt_num, 0, sizeof(SOE_32S) * grid_size);
	
	//��ÿ������ĵ�������ռ䣬�Լ��Ը��������и����id���и�ֵ�����idָ�����ڵ����е���š�
	for (size_t i = 0; i < pt_size; i++)
	{
		SOE_32S grid_x = static_cast<SOE_32S> ((_point_clound[block.block_index][i].x - clound_extent.getMinX()) / pt_win_size);
		SOE_32S grid_y = static_cast<SOE_32S> ((_point_clound[block.block_index][i].y - clound_extent.getMinY()) / pt_win_size);
		SOE_32S grid_pos = grid_y * grid_col + grid_x;
		grid_pt_ids[grid_pos][grid_pt_num[grid_pos]] = i;
		grid_pt_num[grid_pos]++;
	}

	//����ͼ��ֱ��ʣ�XY����,��ʼ�����귶Χ
	SOE_32S dtm_width = static_cast<SOE_32S> ((clound_extent.getMaxX() - clound_extent.getMinX()) / dtm_cell_size);
	SOE_32S dtm_height = static_cast<SOE_32S> ((clound_extent.getMaxY() - clound_extent.getMinY()) / dtm_cell_size);
	soe_envelope dtm_extent;
	dtm_extent.init(clound_extent.getMinX(),
		clound_extent.getMinX() + dtm_width * dtm_cell_size,
		clound_extent.getMinY(),
		clound_extent.getMinY() + dtm_height * dtm_cell_size
		);

	//����ͼ��Ŀ�ߣ���ͨ���͸���������
	SOE_IMG_INFO dsm_file_info;
	dsm_file_info.width = dtm_width;
	dsm_file_info.height = dtm_height;
	dsm_file_info.channel_num = 1;
	dsm_file_info.data_type = BT_32F;

	//ɾ����Ӧ�ļ� 
	std::string dtm_path = block.block_dem_path;
	std::string strname = dtm_path.substr(0, dtm_path.find_last_of("."));
	std::string ploy_name = strname + "_valid.polygon";
	remove(ploy_name.c_str());

	//����Ӱ��
	ImgCore dtm_dataSet;
	if (!dtm_dataSet.Create(dtm_path.c_str(), &dsm_file_info))
	{
		return false;
	}
	dtm_dataSet.SetExtent(&dtm_extent);
	dtm_dataSet.SetProjection(_wkt.c_str());
	
	//ΪӰ�񴴽��ռ䣬��ʼ��Ϊ0
	std::vector<SOE_32S> find_ids;
	SOE_64F x, y;
	SOE_32F * result_data = new SOE_32F[dtm_width * dtm_height];
	memset(result_data, _null_value, sizeof(SOE_32F) * dtm_width * dtm_height);
	for (SOE_16U i = 0; i < dtm_height; i++)
	{
		y = dtm_extent.getMaxY() - i * dtm_cell_size - dtm_cell_size / 2.0;
		for (SOE_16U j = 0; j < dtm_width; j++)
		{
			//�����Ͻ������±���
			x = dtm_extent.getMinX() + j * dtm_cell_size + dtm_cell_size / 2.0;

			//����õ���x,y����ĵڼ���
			SOE_32S grid_x2 = static_cast<SOE_32S>((x - clound_extent.getMinX()) / pt_win_size);
			SOE_32S grid_y2 = static_cast<SOE_32S>((y - clound_extent.getMinY()) / pt_win_size);

			SOE_32S find_iter = 0;
			find_ids.clear();
			//�Ըõ�Ϊ���ı���3*3��
			for ( SOE_32S m = -1; m < 2; m++)
			{
				SOE_32S grid_y = grid_y2 + m;
				for (SOE_32S n = -1; n < 2; n++)
				{
					SOE_32S grid_x = grid_x2 + n;
					SOE_32S grid_pos = grid_y * grid_col + grid_x;
					if (grid_x < 0 ||
						grid_x >= grid_col ||
						grid_y < 0 ||
						grid_y >= grid_row
						)
					{
						continue;
					}

					//����3*3��id��ż���find_ids����
					for (SOE_32S k = 0; k < grid_pt_num[grid_pos]; k++)
					{
						find_ids.push_back(grid_pt_ids[grid_pos][k]);
						find_iter++;
					}
				}
			}
			SOE_64F x_dis, y_dis, xy_dis[find_win_size];
			if ( find_iter < find_win_size)
			{
				result_data[i * dtm_width + j] = _null_value;
				continue;
			}

			for (SOE_32S m = 0; m < find_wind_size; m++)
			{
				SOE_64F x_dis = (_point_clound[block.block_index][find_ids[m]].x - x) *
					(_point_clound[block.block_index][find_ids[m]].x - x) +
					(_point_clound[block.block_index][find_ids[m]].y - y) *
					(_point_clound[block.block_index][find_ids[m]].y - y);
				for (SOE_32S n = m; n < find_iter; n++)
				{

					SOE_64F y_dis = (_point_clound[block.block_index][find_ids[n]].x - x) *
						(_point_clound[block.block_index][find_ids[n]].x - x) +
						(_point_clound[block.block_index][find_ids[n]].y - y) *
						(_point_clound[block.block_index][find_ids[n]].y - y);
					if ( x_dis > y_dis )
					{
						x_dis = y_dis;
						SOE_32S grid_pos = find_ids[m];
						find_ids[m] = find_ids[n];
						find_ids[n] = grid_pos;
					}
				}
				xy_dis[m] = x_dis;
			}
			SOE_64F result_height = 0.0, dis_total = 1;
			if ( xy_dis[0] = 0 )
			{
				result_height = _point_clound[block.block_index][find_ids[0]].z;
				dis_total = 1;
			}
			else
			{
				result_height = 0;
				dis_total = 0;
				for (SOE_32S m = 0; m < find_win_size; m++)
				{
					result_height += _point_clound[block.block_index][find_ids[m]].z * 1.0 / xy_dis[m];
					dis_total += 1.0 / xy_dis[m];
				}
			}
		}
	}

}
*/

//���õ㼯
void recimage_pairs::setPointSet(pt3Set dataSet)
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
		_pointSet.insert(pt3Pair(id, thePt));
		id = id + 1;
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
	this->splitVectorFromPointCloud(_pointSet, segmentSize, epi_blockVector, _point_clound);
	int sizeofCloudVector = epi_blockVector.size();
	//����ÿ���ֵ���
	for (int i = 0; i < sizeofCloudVector; i++)
	{
		epi_block theEpiBlock = epi_blockVector[i];
		this->dtm_resample(_xResolution, theEpiBlock);
	}
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

	SOE_32S dtm_width = static_cast<SOE_32S>((clound_extent.getMaxX() - clound_extent.getMinX()) / dtm_cell_size);
	SOE_32S dtm_height = static_cast<SOE_32S>((clound_extent.getMaxY() - clound_extent.getMinY()) / dtm_cell_size);
	soe_envelope dtm_extent;
	dtm_extent.init(clound_extent.getMinX(), clound_extent.getMinX() + dtm_width * dtm_cell_size, clound_extent.getMinY(), clound_extent.getMinY() + dtm_height * dtm_cell_size);

	//SOE_IMG_INFO dsm_file_info;
	//dsm_file_info.width = dtm_width;
	//dsm_file_info.height = dtm_height;
	//dsm_file_info.channel_num = 1;
	//dsm_file_info.data_type = BT_32F;

	//std::string dtm_path = block.block_dem_path;
	//std::string strname = dtm_path.substr(0, dtm_path.find_last_of("."));
	//std::string ploy_name = strname + "_vaild.polygon";
	//remove(ploy_name.c_str());
	//std::string dtm_path = "test";

	//ImgCore dtm_dataset;
	//if (!dtm_dataset.Create(dtm_path.c_str(), &dsm_file_info))
	//	return false;

	//dtm_dataset.SetExtent(&dtm_extent);
	//dtm_dataset.SetProjection(_wkt.c_str());


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

		}
	}

	
	//dtm_dataset.DataWriteBand(1, 1, 1, dtm_width, dtm_height, result_data);

	//delete[]result_data;
	//result_data = NULL;


	//dtm_dataset.Close();
	delete[]grid_pt_num;
	for (i = 0; i != grid_size; ++i)
	{
		delete[]grid_pt_ids[i];
	}
	delete[]grid_pt_ids;

	return true;
}