#pragma once

#include"../PointCloud/PointCloud.hpp"


#include<vector>
#include<algorithm>

typedef std::vector<long> INDEXGRIDDATA;


/// <summary>
/// A class definition for Rectangle;
/// </summary>
class Rectangle
{
public:
	int left;
	int top;
	int right;
	int bottom;

	Rectangle(int l, int t, int r, int b)
		: left(l),
		top(t),
		right(r),
		bottom(b)
	{}

	int Width() const { return right - left; }

	int Height() const { return bottom - top; }

	bool Contains(int x, int y) const
	{
		return x >= left && x <= right && y >= top && y <= bottom;
	}
	void DeflateRectangle(int dx, int dy)
	{
		left += dx;
		top += dy;
		right -= dx;
		bottom -= dy;
	}

	void DeflateRectangle(int dx1, int dy1, int dx2, int dy2)
	{
		left += dx1;
		top += dy1;
		right -= dx2;
		bottom -= dy2;
	}

	bool IntersectRectangle(const Rectangle& rectangle)
	{
		int newLeft = max(left, rectangle.left);
		int newTop = max(top, rectangle.top);
		int newRight = min(right, rectangle.right);
		int newBottom = min(bottom, rectangle.bottom);

		if (newRight > newLeft && newBottom > newTop)
		{
			left = newLeft;
			top = newTop;
			right = newRight;
			bottom = newBottom;
			return true; // The rectangles intersect
		}
		else
		{
			left = 0;
			top = 0;
			right = 0;
			bottom = 0;
			return false; // The rectangles do not intersect
		}
	}
};

/// <summary>
/// The header for DTM
/// </summary>
struct DtmHeader
{
	double	OffsetX, OffsetY;
	float	StepX, StepY;			
	int		Col, Row;	
	int  	Scale, NODATA;		
	
	DtmHeader()
	{
		OffsetX = OffsetY = 0.0;
		StepX = StepY = 0.0;
		Col = Row = 0;
		Scale = 100;
		NODATA = -99999;
	}
};


/// <summary>
/// The DTM dataset
/// </summary>
struct DtmData
{
	DtmHeader m_dtm_header;
	float* m_dtm_data;
	DtmData() 
	{
		this->m_dtm_data = nullptr;
	}
};


/// <summary>
/// The class for DTM generation using basic interpolation.
/// </summary>
class DTM
{
public:
	/// <summary>
	/// Default constructor
	/// </summary>
	DTM();

	/// <summary>
	/// Default destructor
	/// </summary>
	~DTM();

public:

	/// <summary>
	/// The point cloud data;
	/// </summary>
	std::vector<dPoint3D> m_points;

	/// <summary>
	/// Create a DTM using the input point cloud data and the given resolution;
	/// </summary>
	/// <param name="pointcloud"></param> The input point cloud data.
	/// <param name="data"></param> The generated DTM
	/// <param name="resolution"></param> The given resolution
	void CreateDTM(PointCloud* pointcloud, DtmData* data, float resolution);

	/// <summary>
	/// To fill the holes in a generated DTM
	/// </summary>
	/// <param name="dtm"></param>
	void HoleFilling(DtmData* dtm);


	/// <summary>
	/// To smooth the outliers of the generated DTM
	/// </summary>
	/// <param name="dtm"></param>
	void Smooth(DtmData* dtm);

private:

	/// <summary>
	/// </summary>
	/// <param name="dtm"></param>
	/// <param name="row"></param>
	/// <param name="col"></param>
	/// <returns></returns>
	bool Inside(DtmData* dtm, int row, int col);

};
