
#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_


#pragma once

#include<Windows.h>
#include<limits.h>
#include<string>

#include"../LASlib/lasreader.hpp"
#include"../LASlib/laswriter.hpp"


#include"BasicStructure.h"



/// <summary>
/// Class for the point cloiud
/// </summary>
class PointCloud
{
public:

	//----begin of attibutes entity;
	long		m_PtsNum;			//points number in this class;

	double		m_xMax;
	double		m_yMax;
	double		m_zMax;
	double		m_xMin;
	double		m_yMin;
	double		m_zMin;

	double		m_xOffset;
	double		m_yOffset;
	double		m_zOffset;

	double		m_xScale;
	double		m_yScale;
	double		m_zScale;

	float		m_rMax;
	float		m_gMax;
	float		m_bMax;

	char			m_FileInfo[32];		//imported file description;
	dPoint3D		*m_Points;			//points; double type points
	unsigned short	*m_PtsIntensity;	//points intensity;
	unsigned char	*m_PtsClassInfo;	//class information of the points;
	unsigned char	*m_PtsReturnNum;	//points return label;
	double			*m_PtsGnssTime;		//points GNSS time label;
	PtsColor		*m_PtsColor;		//points color;
	float			*m_alpha;
	//----end of attributes entity;

	//begin of virtual function decleration;
public:

	/// <summary>
	/// Load the laz file;
	/// </summary>
	/// <param name="lpszPathName"></param>
	/// <returns></returns>
	virtual bool LoadFile(std::string lpszPathName) = 0;//{return 0;}; // left for later modification;

	/// <summary>
	/// save the las file;
	/// </summary>
	/// <param name="lpszPathName"></param>
	/// <returns></returns>
	virtual bool SaveFile(std::string lpszPathName) = 0; //{return 0;}; // left for later modification;

	//end of virtual function decleration;


//----begin of implementation and functions;
public:
	//default constructor; 
	PointCloud()
		:m_PtsNum(0)
		, m_PtsClassInfo(NULL)
		, m_Points(NULL)
		, m_PtsReturnNum(NULL)
		, m_PtsColor(NULL)
		, m_PtsGnssTime(NULL)
		, m_PtsIntensity(NULL)
		, m_alpha(0)
		, m_xOffset(0.0)
		, m_yOffset(0.0)
		, m_zOffset(0.0)
		, m_xScale(1.0)
		, m_yScale(1.0)
		, m_zScale(1.0)
		, m_rMax(0.0)
		, m_gMax(0.0)
		, m_bMax(0.0)
	{
		m_xMin = (std::numeric_limits<double>::max)();
		m_yMin = (std::numeric_limits<double>::max)();
		m_zMin = (std::numeric_limits<double>::max)();

		m_xMax = -m_xMin;
		m_yMax = -m_yMin;
		m_zMax = -m_zMin;
	};

	//destructor, release the memory;
	virtual ~PointCloud()
	{
		if (m_PtsClassInfo)			delete[]m_PtsClassInfo;		m_PtsClassInfo = NULL;
		if (m_Points)				delete[]m_Points;			m_Points = NULL;
		if (m_PtsColor)				delete[]m_PtsColor;			m_PtsColor = NULL;
		if (m_PtsReturnNum)			delete[]m_PtsReturnNum;		m_PtsReturnNum = NULL;
		if (m_PtsIntensity)			delete[]m_PtsIntensity;		m_PtsIntensity = NULL;
		if (m_PtsGnssTime)			delete[]m_PtsGnssTime;		m_PtsGnssTime = NULL;
	};

	//***point cloud initiation from array of points
	//---coefficient: 
	PointCloud(const double *points, const long pointsnumber)
	{
		m_PtsNum = pointsnumber;
		m_Points = new dPoint3D[pointsnumber * 3];

		m_PtsGnssTime = NULL;
		m_PtsColor = NULL;
		m_PtsIntensity = NULL;
		m_PtsClassInfo = NULL;

		for (int _i = 0; _i<pointsnumber * 3; _i = _i + 3)
		{
			m_Points[_i / 3].x = points[_i + 0];
			m_Points[_i / 3].y = points[_i + 1];
			m_Points[_i / 3].z = points[_i + 2];
		}
	}

	//--point cloud initiation from PointCloud objects;
	
	PointCloud& operator=(const PointCloud* pointcloud)
	{
		if (pointcloud->m_PtsNum)			m_PtsNum = pointcloud->m_PtsNum;
		if (pointcloud->m_Points)			m_Points = new dPoint3D[m_PtsNum];
		if (pointcloud->m_PtsColor)			m_PtsColor = new PtsColor[m_PtsNum];
		if (pointcloud->m_PtsReturnNum)		m_PtsReturnNum = new byte[m_PtsNum];
		if (pointcloud->m_PtsGnssTime)		m_PtsGnssTime = new double[m_PtsNum];
		if (pointcloud->m_PtsIntensity)		m_PtsIntensity = new unsigned short[m_PtsNum];
		if (pointcloud->m_PtsClassInfo)		m_PtsClassInfo = new byte[m_PtsNum];

		for (int _i = 0; _i < pointcloud->m_PtsNum; _i++)
		{
			m_Points[_i].x = pointcloud->m_Points[_i].x;
			m_Points[_i].y = pointcloud->m_Points[_i].y;
			m_Points[_i].z = pointcloud->m_Points[_i].z;
			if (m_PtsColor)
			{
				m_PtsColor[_i] = pointcloud->m_PtsColor[_i];
			}
			if (m_PtsReturnNum)
			{
				m_PtsReturnNum[_i] = pointcloud->m_PtsReturnNum[_i];
			}
			if (m_PtsGnssTime)
			{
				m_PtsGnssTime[_i] = pointcloud->m_PtsGnssTime[_i];
			}
			if (m_PtsIntensity)
			{
				m_PtsIntensity[_i] = pointcloud->m_PtsIntensity[_i];
			}
			if (m_PtsClassInfo)
			{
				m_PtsClassInfo[_i] = pointcloud->m_PtsClassInfo[_i];
			}
		}
		return *this;
	}
};



/// <summary>
/// Class inherited from PointCloud.
/// </summary>
class CLazPointData : public PointCloud
{
public:
	CLazPointData();
	~CLazPointData();

	bool	LoadFile(std::string lpszPathName);
	bool	SaveFile(std::string lpszPathName);

protected:
	void	InitMemberValue();
	void	ReleaseMemSpace();
	void	NewMemSpace();

private:
	LASheader		m_nLASheader;

	LASreader*		m_pLASreader;
	LASreadOpener*	m_pLASreadOpener;

	LASwriter*		m_pLASwriter;
	LASwriteOpener*	m_pLASwriteOpener;

	LASpoint	*m_nLASpoint;

	char* m_pFileNameIn;
	char* m_pFileNameOut;

	long		m_nReadPointNum; //imported points number;
	std::string	m_nOpenFileName; //imported file name;
};

#endif
