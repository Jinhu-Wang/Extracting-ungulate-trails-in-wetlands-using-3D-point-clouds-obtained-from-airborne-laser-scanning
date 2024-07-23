
#include"CreateDTM.h"

DTM::DTM()
{}


DTM::~DTM()
{
}


void DTM::CreateDTM(PointCloud* pointcloud, DtmData* dtm, float resolution)
{
	float	minX = pointcloud->m_xMax; 
	float	minY = pointcloud->m_yMax; 
	float	maxX = pointcloud->m_xMin; 
	float	maxY = pointcloud->m_yMin; 

	bool	limit = m_points.size() > 2;
	if (m_points.size() != 0)
	{
		for (int i = 0; i < m_points.size(); i++)
		{
			minX = minX < m_points[i].x ? minX : m_points[i].x;
			minY = minY < m_points[i].y ? minY : m_points[i].y;
			maxX = maxX > m_points[i].x ? maxX : m_points[i].x;
			maxY = maxY > m_points[i].y ? maxY : m_points[i].y;
		}
	}
	else
	{
		minX = pointcloud->m_xMin;
		minY = pointcloud->m_yMin;
		maxX = pointcloud->m_xMax;
		maxY = pointcloud->m_yMax;
	}

	double	distanceX = maxX - minX;
	double	distanceY = maxY - minY;

	int m_Heigh = int(distanceY / resolution + 1);
	int m_Width = int(distanceX / resolution + 1);

	dtm->m_dtm_header.OffsetX = minX + resolution / 2.0;
	dtm->m_dtm_header.OffsetY = minY + resolution / 2.0;

	dtm->m_dtm_header.StepX = resolution;
	dtm->m_dtm_header.StepY = resolution;

	dtm->m_dtm_header.Col = m_Heigh;
	dtm->m_dtm_header.Row = m_Width;

	if (dtm->m_dtm_data) delete[]dtm->m_dtm_data;
	dtm->m_dtm_data = new float[m_Width * m_Heigh];

	double* GridPointDis = new double[m_Width * m_Heigh];
	double* GridPointHeigh = new double[m_Width * m_Heigh];

	double StepX = dtm->m_dtm_header.StepX;
	double StepY = dtm->m_dtm_header.StepY;

	memset(GridPointHeigh, 0, sizeof(double) * m_Width * m_Heigh);
	memset(GridPointDis, 0, sizeof(double) * m_Width * m_Heigh);

	dPoint3D pointtmp; double x0, y0; int x, y; double dis = 0.0; double disweight = sqrt(2.0) / 2.0;
	for (int i = 0; i < pointcloud->m_PtsNum; i++) 
	{
		pointtmp = pointcloud->m_Points[i]; 
		if (pointcloud->m_PtsClassInfo && pointcloud->m_PtsClassInfo[i] != 0) { continue; } 	
		if (pointtmp.z != INVALIDATE_VALUE)
		{
			if (limit && (pointtmp.x < minX || pointtmp.y < minY || pointtmp.x > maxX || pointtmp.y > maxY)) continue;

			x0 = (pointtmp.x - minX) / StepX;
			y0 = (pointtmp.y - minY) / StepY;

			x = int(x0);
			y = int(y0);

			dis = pow(0.5 - x0 + x, 2);
			dis += pow(0.5 - y0 + y, 2);
			dis = sqrt(dis);
			dis = disweight - dis;

			if (x < 0 || x >= m_Width || y < 0 || y >= m_Heigh) continue;
			{
				GridPointDis[(int)(y * m_Width + x)] += dis;
				GridPointHeigh[(int)(y * m_Width + x)] += dis * pointtmp.z;
			}
		}
	}

	for (int i = 0; i < m_Heigh * m_Width; i++)
	{
		if (GridPointDis[i] != 0)
			dtm->m_dtm_data[i] = (GridPointHeigh[i] / GridPointDis[i]); 
		else
			dtm->m_dtm_data[i] = dtm->m_dtm_header.NODATA;
	}

	if (GridPointDis)	delete[]GridPointDis;		GridPointDis = nullptr;
	if (GridPointHeigh)	delete[]GridPointHeigh;	GridPointHeigh = nullptr;
}


void DTM::HoleFilling(DtmData* dtm)
{
	if (dtm->m_dtm_data == nullptr) return;

	int position = 0;
	int positiontemp = 0;

	double HeightTemp = 0.0; int pointnum = 0;

	int width = dtm->m_dtm_header.Row;
	int heigh = dtm->m_dtm_header.Col;
	int num = width * heigh;

	float* tempdata1 = new float[num];
	memcpy(tempdata1, dtm->m_dtm_data, sizeof(float) * num);

	float* tempdata2 = new float[num];
	memcpy(tempdata2, dtm->m_dtm_data, sizeof(float) * num);

	float* tempdata3 = new float[num];
	memcpy(tempdata3, dtm->m_dtm_data, sizeof(float) * num);

	unsigned char* Mark = new unsigned char[num];
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < heigh; j++)
		{
			if (Inside(dtm, i, j)) 
				Mark[j * width + i] = 1;
			else Mark[j * width + i] = 0;
		}
	}

	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < heigh; j++)
		{
			position = j * width + i;

			if (Mark[position]) continue;

			if (tempdata1[position] == dtm->m_dtm_header.NODATA)
			{
				HeightTemp = 0.0;
				pointnum = 0;
				for (int k = -1; k < 2; k++)
					for (int l = -1; l < 2; l++)
					{
						if (i + k >= heigh || i + k < 0 || j + l >= width || j + l < 0) continue;
						positiontemp = (i + k) * width + (j + l);
						if (tempdata1[positiontemp] == dtm->m_dtm_header.NODATA) continue;
						pointnum++;
						HeightTemp += tempdata1[positiontemp];
					}
				if (pointnum)
				{
					tempdata1[position] = (HeightTemp / pointnum);
				}
			}
		}
	}

	for (int i = width - 1; i > -1; i--)
	{
		for (int j = heigh - 1; j > -1; j--)
		{
			position = j * width + i;

			if (Mark[position]) continue;

			if (tempdata2[position] == dtm->m_dtm_header.NODATA)
			{
				HeightTemp = 0.0; 		pointnum = 0;
				for (int k = -1; k < 2; k++)
					for (int l = -1; l < 2; l++)
					{
						if (i + k >= heigh || i + k < 0 || j + l >= width || j + l < 0) continue;
						positiontemp = (i + k) * width + (j + l);
						if (tempdata2[positiontemp] == dtm->m_dtm_header.NODATA) continue;
						pointnum++;
						HeightTemp += tempdata2[positiontemp];
					}
				if (pointnum)		tempdata2[position] = (HeightTemp / pointnum);
			}
		}
	}

	for (int i = heigh - 1; i > -1; i--)
	{
		for (int j = width - 1; j > -1; j--)
		{
			position = i * width + j;

			if (Mark[position]) continue;

			if (tempdata3[position] == dtm->m_dtm_header.NODATA)
			{
				HeightTemp = 0.0;
				pointnum = 0;
				for (int k = -1; k < 2; k++)
					for (int l = -1; l < 2; l++)
					{
						if (i + k >= heigh || i + k < 0 || j + l >= width || j + l < 0) continue;
						positiontemp = (i + k) * width + (j + l);
						if (tempdata3[positiontemp] == dtm->m_dtm_header.NODATA) continue;
						pointnum++;
						HeightTemp += tempdata3[positiontemp];
					}
				if (pointnum)
				{
					tempdata3[position] = (HeightTemp / pointnum);
				}
			}
		}
	}

	for (int i = 0; i < heigh; i++)
	{
		for (int j = 0; j < width; j++)
		{
			position = i * width + j;

			if (Mark[position]) continue;

			if (dtm->m_dtm_data[position] == dtm->m_dtm_header.NODATA)
			{
				HeightTemp = 0.0;
				pointnum = 0;
				for (int k = -1; k < 2; k++)
					for (int l = -1; l < 2; l++)
					{
						if (i + k >= heigh || i + k < 0 || j + l >= width || j + l < 0) continue;
						positiontemp = (i + k) * width + (j + l);
						if (dtm->m_dtm_data[positiontemp] == dtm->m_dtm_header.NODATA) continue;
						pointnum++;
						HeightTemp += dtm->m_dtm_data[positiontemp];
					}
				if (pointnum)
				{
					dtm->m_dtm_data[position] = (HeightTemp / pointnum);
				}
			}
		}
	}



	for (int i = 0; i < num; i++)
	{
		if (Mark[num]) continue;

		if (dtm->m_dtm_data[i] == dtm->m_dtm_header.NODATA ||
			tempdata1[i] == dtm->m_dtm_header.NODATA ||
			tempdata2[i] == dtm->m_dtm_header.NODATA ||
			tempdata3[i] == dtm->m_dtm_header.NODATA)
		{
			dtm->m_dtm_data[i] = dtm->m_dtm_header.NODATA;
		}
		else dtm->m_dtm_data[i] = ((dtm->m_dtm_data[i] + tempdata1[i] + tempdata2[i] + tempdata3[i]) / 4.0);
	}


	delete[]tempdata1; tempdata1 = nullptr;
	delete[]tempdata2; tempdata2 = nullptr;
	delete[]tempdata3; tempdata3 = nullptr;
	delete[]Mark;	    Mark = nullptr;

}


void DTM::Smooth(DtmData* dtm)
{
	if (dtm == nullptr || dtm->m_dtm_data == nullptr) return;

	int position = 0;
	int positiontemp = 0;
	int modewidth = 3;

	float* tmp = new float[dtm->m_dtm_header.Col * dtm->m_dtm_header.Row];
	memcpy(tmp, dtm->m_dtm_data, sizeof(float) * dtm->m_dtm_header.Col * dtm->m_dtm_header.Row);

	double HeightTemp = 0.0; int pointnum = 0;

	for (int i = 0; i < dtm->m_dtm_header.Col; i++)
	{
		for (int j = 0; j < dtm->m_dtm_header.Row; j++)
		{
			position = i * dtm->m_dtm_header.Row + j;

			if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA)
			{
				HeightTemp = 0.0;
				pointnum = 0;
				for (int k = -modewidth / 2; k < modewidth / 2 + 1; k++)
					for (int l = -modewidth / 2; l < modewidth / 2 + 1; l++)
					{
						if (k == 0 && l == 0) continue;
						if (i + k >= dtm->m_dtm_header.Col || i + k < 0 || j + l >= dtm->m_dtm_header.Row || j + l < 0) continue;
						positiontemp = (i + k) * dtm->m_dtm_header.Row + (j + l);
						if (dtm->m_dtm_data[positiontemp] == dtm->m_dtm_header.NODATA) continue;
						pointnum++;
						HeightTemp += dtm->m_dtm_data[positiontemp];
					}
				if (fabs(HeightTemp / pointnum - dtm->m_dtm_data[position]) > 0.3)
					tmp[position] = (HeightTemp / pointnum);
			}
		}
	}
	memcpy(dtm->m_dtm_data, tmp, sizeof(float) * dtm->m_dtm_header.Col * dtm->m_dtm_header.Row);
	delete[]tmp; tmp = nullptr;
	return;
}



bool DTM::Inside(DtmData* dtm, int row, int col)
{
	int position = 0;

	if (col >= dtm->m_dtm_header.Col || col < 0 || row < 0 || row >= dtm->m_dtm_header.Row) return false;

	position = col * dtm->m_dtm_header.Row + row;
	if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA) return false;

	int flag = 0;

	for (int i = -1; i < 2; i++)
		for (int j = -1; j < 2; j++)
		{
			if (i + col >= dtm->m_dtm_header.Col || i + col < 0 || j + row >= dtm->m_dtm_header.Row || j + row < 0) continue;
			position = (row + j) + (col + i) * dtm->m_dtm_header.Row;
			if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA) flag++;
		}
	if (flag != 0) return false;

	flag = 0;

	for (int i = row; i < dtm->m_dtm_header.Row; i++)
	{
		position = col * dtm->m_dtm_header.Row + i;
		if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA)
		{
			flag++;
			i = dtm->m_dtm_header.Row;
		}
	}

	for (int i = row; i >= 0; i--)
	{
		position = col * dtm->m_dtm_header.Row + i;
		if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA)
		{
			flag++;
			i = -1;
		}
	}

	for (int i = col; i < dtm->m_dtm_header.Col; i++)	
	{
		position = i * dtm->m_dtm_header.Row + row;
		if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA)
		{
			flag++;
			i = dtm->m_dtm_header.Col;
		}
	}

	for (int i = col; i >= 0; i--)
	{
		position = i * dtm->m_dtm_header.Row + row;
		if (dtm->m_dtm_data[position] != dtm->m_dtm_header.NODATA)
		{
			flag++;
			i = -1;
		}
	}

	if (flag != 4) return true;

	return false;
}
