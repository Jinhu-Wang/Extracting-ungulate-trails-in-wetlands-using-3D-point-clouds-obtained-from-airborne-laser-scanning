
#include"PointCloud.hpp"


CLazPointData::CLazPointData()
	: m_nLASpoint(NULL)
	, m_pLASreader(NULL)
	, m_pLASreadOpener(NULL)
	, m_pLASwriter(NULL)
	, m_pLASwriteOpener(NULL)
{
	this->m_pLASreadOpener = new LASreadOpener;
}


CLazPointData::~CLazPointData()
{
	ReleaseMemSpace();
	if (this->m_pLASreadOpener) delete this->m_pLASreadOpener; this->m_pLASreadOpener = NULL;
}



bool CLazPointData::LoadFile(std::string lpszPathName)
{
	this->m_pLASreadOpener->set_file_name(lpszPathName.c_str());
	this->m_pLASreader = this->m_pLASreadOpener->open();
	this->m_nLASheader = this->m_pLASreader->header;
	
	this->InitMemberValue();
	this->NewMemSpace();

	int i = 0;
	while (this->m_pLASreader->read_point())
	{
		this->m_Points[i].x = this->m_pLASreader->point.X*this->m_xScale + this->m_xOffset;
		this->m_Points[i].y = this->m_pLASreader->point.Y*this->m_yScale + this->m_yOffset;
		this->m_Points[i].z = this->m_pLASreader->point.Z*this->m_zScale + this->m_zOffset;

		this->m_PtsGnssTime[i] = this->m_pLASreader->point.gps_time;
		this->m_PtsIntensity[i] = this->m_pLASreader->point.intensity;
		this->m_PtsColor[i].red = this->m_pLASreader->point.rgb[0];
		this->m_PtsColor[i].green = this->m_pLASreader->point.rgb[1];
		this->m_PtsColor[i].blue = this->m_pLASreader->point.rgb[2];
		this->m_PtsReturnNum[i] = this->m_pLASreader->point.return_number;
		this->m_PtsClassInfo[i] = this->m_pLASreader->point.classification;

		this->m_nLASpoint[i] = this->m_pLASreader->point;
		i++;
	}
	return true;
}



void CLazPointData::InitMemberValue()
{
	this->m_xMax = this->m_nLASheader.max_x;
	this->m_yMax = this->m_nLASheader.max_y;
	this->m_zMax = this->m_nLASheader.max_z;

	this->m_xMin = this->m_nLASheader.min_x;
	this->m_yMin = this->m_nLASheader.min_y;
	this->m_zMin = this->m_nLASheader.min_z;

	this->m_xOffset = this->m_nLASheader.x_offset;
	this->m_yOffset = this->m_nLASheader.y_offset;
	this->m_zOffset = this->m_nLASheader.z_offset;

	this->m_xScale = this->m_nLASheader.x_scale_factor;
	this->m_yScale = this->m_nLASheader.y_scale_factor;
	this->m_zScale = this->m_nLASheader.z_scale_factor;

	this->m_PtsNum = this->m_nLASheader.number_of_point_records;
}



bool CLazPointData::SaveFile(std::string lpszPathName)
{
	if (this->m_Points == NULL)
	{
		return false;
	}
	
	this->m_pLASwriteOpener->set_file_name(lpszPathName.c_str());
	this->m_pLASwriter = this->m_pLASwriteOpener->open(&this->m_nLASheader);
	
	for (int i = 0; i < this->m_PtsNum; ++i)
	{
		this->m_pLASwriter->write_point(&this->m_nLASpoint[i]);
	}

	this->m_pLASwriter->close();
	delete this->m_pLASwriter;

	return true;
}


 
void CLazPointData::NewMemSpace()
{
	//////////////////////////////////////////////////////////////////////////
	ReleaseMemSpace();

	this->m_Points = new dPoint3D[m_PtsNum];
	this->m_PtsGnssTime = new double[m_PtsNum];
	this->m_PtsIntensity = new unsigned short[m_PtsNum];
	this->m_PtsReturnNum = new unsigned char[m_PtsNum];
	this->m_PtsClassInfo = new unsigned char[m_PtsNum];
	this->m_PtsColor = new PtsColor[m_PtsNum];
	this->m_nLASpoint = new LASpoint[m_PtsNum];
}

void CLazPointData::ReleaseMemSpace()
{
	if (m_Points)							delete[]m_Points;				m_Points = NULL;
	if (m_PtsClassInfo)						delete[]m_PtsClassInfo;			m_PtsClassInfo = NULL;
	if (m_PtsColor)							delete[]m_PtsColor;				m_PtsColor = NULL;
	if (m_PtsIntensity)						delete[]m_PtsIntensity;			m_PtsIntensity = NULL;
	if (m_PtsGnssTime)						delete[]m_PtsGnssTime;			m_PtsGnssTime = NULL;
	if (m_PtsReturnNum)						delete[]m_PtsReturnNum;			m_PtsReturnNum = NULL;
}