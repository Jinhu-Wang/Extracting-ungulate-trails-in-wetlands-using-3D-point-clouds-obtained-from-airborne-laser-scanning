#include"FilterVegetation.h"


#include<math.h>
#include<algorithm>

FilterVegetation::FilterVegetation()
	: m_pointcloud(nullptr)
	, m_height_threhold(0.5)
{
}


FilterVegetation::FilterVegetation(PointCloud* pointcloud)
	: m_pointcloud(pointcloud)
	, m_height_threhold(0.5)
{}


FilterVegetation::~FilterVegetation()
{}


void FilterVegetation::SetPointCloud(PointCloud* pointcloud)
{
	this->m_pointcloud = pointcloud; 
}

void FilterVegetation::SetHeightThreshold(double height_threshold)
{
	this->m_height_threhold = height_threshold;
}


void FilterVegetation::IterativeVoxelFiltering(double minVoxel, double maxVoxel, double ratio)
{
	if (m_pointcloud == nullptr) return;
	if (m_pointcloud->m_Points == nullptr) return;

	
	double disweight = sqrt(2.0) / 2.0;
	double dis = 0.0;
	double* GridPointHeigh = NULL;
	double* GridPointDis = NULL;

	int m_Width, m_Heigh;

	double Minx, Miny, Maxx, Maxy;
	Minx = m_pointcloud->m_xMax;
	Miny = m_pointcloud->m_yMax;
	Maxx = m_pointcloud->m_xMin;
	Maxy = m_pointcloud->m_yMin;

	dPoint3D	pointtmp;
	for (int i = 0; i < (int)m_pointcloud->m_PtsNum; i++)	
	{

		Minx = min(Minx, m_pointcloud->m_Points[i].x);
		Maxx = max(Maxx, m_pointcloud->m_Points[i].x);
		Miny = min(Miny, m_pointcloud->m_Points[i].y);
		Maxy = max(Maxy, m_pointcloud->m_Points[i].y);
	}

	
	double	heighlimit;			
	int		x, y;  	double  x0, y0;
	double  xz, yz, temp = 0.0;

	double Step = minVoxel;
	while ((Step >= minVoxel && Step <= maxVoxel) || (Step <= minVoxel && Step >= maxVoxel))
	{
		heighlimit = this->m_height_threhold * Step / 1.0;

		m_Width = int((Maxx - Minx) / Step + 1);
		m_Heigh = int((Maxy - Miny) / Step + 1);

		if (GridPointHeigh)	delete[]GridPointHeigh;	GridPointHeigh = new double[m_Width * m_Heigh];
		if (GridPointDis)	delete[]GridPointDis;		GridPointDis = new double[m_Width * m_Heigh];

		memset(GridPointHeigh, 0, sizeof(double) * m_Width * m_Heigh);
		memset(GridPointDis, 0, sizeof(double) * m_Width * m_Heigh);

		dPoint3D pointtmp;
		for (int k = 0; k < m_pointcloud->m_PtsNum; k++)
		{

			if (m_pointcloud->m_PtsClassInfo[k] != 0)	continue;
			pointtmp = m_pointcloud->m_Points[k];
			{
				x0 = (pointtmp.x - Minx) / Step;
				y0 = (pointtmp.y - Miny) / Step;

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
				GridPointHeigh[i] /= GridPointDis[i];
			else
				GridPointHeigh[i] = INVALIDATE_VALUE;
		}

		memcpy(GridPointDis, GridPointHeigh, sizeof(double) * m_Width * m_Heigh);
		int		modewidth = 5, effectptnum = 0, position;
		for (int i = modewidth / 2; i < m_Heigh - modewidth / 2; i++)
		{
			for (int j = modewidth / 2; j < m_Width - modewidth / 2; j++)
			{
				effectptnum = 0;
				GridPointDis[i * m_Width + j] = 0.0;
				for (int k = -modewidth / 2; k <= modewidth / 2; k++)
					for (int l = -modewidth / 2; l <= modewidth / 2; l++)
					{
						position = (j + k) + (l + i) * m_Width;
						if (GridPointHeigh[position] != INVALIDATE_VALUE)
						{
							effectptnum++;
							GridPointDis[i * m_Width + j] += GridPointHeigh[position];
						}
					}

				if (effectptnum)		GridPointDis[i * m_Width + j] /= effectptnum;
				else				GridPointDis[i * m_Width + j] = INVALIDATE_VALUE;
			}
		}
		memcpy(GridPointHeigh, GridPointDis, sizeof(double) * m_Width * m_Heigh);

		if (GridPointDis)	delete[]GridPointDis; GridPointDis = NULL;

		for (int i = 0; i < m_pointcloud->m_PtsNum; i++)
		{
			pointtmp = m_pointcloud->m_Points[i];

			x0 = (pointtmp.x - Minx) / Step - 0.5;
			y0 = (pointtmp.y - Miny) / Step - 0.5;

			x = x0;		y = y0;

			if (!(x < 0 || x >= m_Width - 1 || y < 0 || y >= m_Heigh - 1))
			{
				if (GridPointHeigh[y * m_Width + x + 1] != INVALIDATE_VALUE && GridPointHeigh[y * m_Width + x] != INVALIDATE_VALUE)
					xz = (x0 - x) * GridPointHeigh[y * m_Width + x + 1] + (x + 1 - x0) * GridPointHeigh[y * m_Width + x];
				else if (GridPointHeigh[y * m_Width + x + 1] != INVALIDATE_VALUE)
					xz = GridPointHeigh[y * m_Width + x + 1];
				else if (GridPointHeigh[y * m_Width + x] != INVALIDATE_VALUE)
					xz = GridPointHeigh[y * m_Width + x];
				else	xz = INVALIDATE_VALUE;

				if (GridPointHeigh[(y + 1) * m_Width + x + 1] != INVALIDATE_VALUE && GridPointHeigh[(y + 1) * m_Width + x] != INVALIDATE_VALUE)
					yz = (x0 - x) * GridPointHeigh[(y + 1) * m_Width + x + 1] + (x + 1 - x0) * GridPointHeigh[(y + 1) * m_Width + x];
				else if (GridPointHeigh[(y + 1) * m_Width + x + 1] != INVALIDATE_VALUE)
					yz = GridPointHeigh[(y + 1) * m_Width + x + 1];
				else if (GridPointHeigh[(y + 1) * m_Width + x] != INVALIDATE_VALUE)
					yz = GridPointHeigh[(y + 1) * m_Width + x];
				else	yz = INVALIDATE_VALUE;


				if (xz != INVALIDATE_VALUE && yz != INVALIDATE_VALUE)
					temp = yz * (y0 - y) + xz * (y + 1 - y0);
				else if (xz != INVALIDATE_VALUE)
					temp = xz;
				else if (yz != INVALIDATE_VALUE)
					temp = yz;

				if (temp != 0)
				{
					if (pointtmp.z - temp > heighlimit / 3.0) 
						m_pointcloud->m_PtsClassInfo[i] = 1;

					if (pointtmp.z - temp < -8.0) 
						m_pointcloud->m_PtsClassInfo[i] = 9;
				}
				temp = 0.0;
			}
		}
		if (GridPointHeigh)	delete[]GridPointHeigh; GridPointHeigh = NULL;

		Step += ratio;
	}
}
