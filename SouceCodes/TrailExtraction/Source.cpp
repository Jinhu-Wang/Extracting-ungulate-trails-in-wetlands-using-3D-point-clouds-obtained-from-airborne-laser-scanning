
// System
#include<iostream>
#include<vector>
#include<string>
#include<filesystem>
#include<map>
#include<utility>
#include<algorithm>
#include<math.h>
#include<numeric>
#include<stack>
#include<random>


// Local
#include"../PointCloud/PointCloud.hpp"
#include"../FilterVegetation/FilterVegetation.h"
#include"../CreateDtm/CreateDTM.h"
#include"../Include/nanoflann.hpp"


using namespace nanoflann;
typedef KDTreeSingleIndexAdaptor < L2_Simple_Adaptor<double, PointCloud2<double>>,
	PointCloud2<double>, 3 > kdTree;


/// <summary>
/// Convert string to Wide string
/// </summary>
/// <param name="str"></param>
/// <returns></returns>
std::wstring stringToWString(const std::string& str)
{
	if (str.empty()) return std::wstring();
	int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
	std::wstring wstr(sizeNeeded, 0);
	MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstr[0], sizeNeeded);
	return wstr;
}



/// <summary>
/// Convert wide string to string
/// </summary>
/// <param name="wstr"></param>
/// <returns></returns>
std::string wstringToString(const std::wstring& wstr)
{
	if (wstr.empty()) return std::string();
	int sizeNeeded = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
	std::string str(sizeNeeded, 0);
	WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &str[0], sizeNeeded, NULL, NULL);
	return str;
}


/// <summary>
/// List all file in a directory and store in outFiles
/// </summary>
/// <param name="directoryPath"></param>
/// <param name="outFiles"></param>
void ListFilesInDirectory(const std::string& directoryPath, std::vector<std::string>& outFiles)
{
	std::wstring searchPath = stringToWString(directoryPath) + L"\\*";
	WIN32_FIND_DATA findData;
	HANDLE hFind = FindFirstFile(searchPath.c_str(), &findData);

	if (hFind != INVALID_HANDLE_VALUE)
	{
		do
		{
			if (!(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			{
				outFiles.push_back(wstringToString(findData.cFileName));
			}
		} while (FindNextFile(hFind, &findData) != 0);
		FindClose(hFind);
	}
	else
	{
		std::cerr << "Failed to list files in directory." << std::endl;
	}
}

/// <summary>
/// Calculate the standard devidation of the given values.
/// </summary>
/// <param name="vec"></param>
/// <returns></returns>
double calculateStandardDeviation(std::vector<double>& vec)
{
	if (vec.empty())
	{
		return 0; // Return 0 or handle as error for empty vector
	}

	float mean = std::accumulate(vec.begin(), vec.end(), 0.0f) / vec.size();
	float sq_sum = std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0f,
		std::plus<>(), [mean](float a, float b)
		{
			return (a - mean) * (b - mean);
		});

	return std::sqrt(sq_sum / vec.size());
}

/// <summary>
/// Split the points according to thre coefficient.
/// </summary>
/// <param name="points"></param>
/// <param name="coefficient"></param>
/// <param name="threshold"></param>
/// <param name="trailPts"></param>
/// <param name="nonTrailPts"></param>
void splitPoints(std::vector<dPoint3D> points, float coefficient, std::vector<dPoint3D>& trailPts, std::vector<dPoint3D>& nonTrailPts)
{
	if (points.empty()) return;

	std::vector<double> values;
	for (auto p : points)
	{
		values.push_back(p.val);
	}

	double stddev = calculateStandardDeviation(values);
	double mean = std::accumulate(values.begin(), values.end(), 0.0f) / values.size();

	float threshold = mean - stddev * coefficient;

	for (auto& p : points)
	{
		if (p.val <= threshold)
		{
			trailPts.push_back(p);
			p.label = 11;
		}
		else
		{
			nonTrailPts.push_back(p);
			p.label = 12;
		}
	}

	return;
}

int main(int argc, char** argv)
{

	std::string inPlotFolder = "Path\\to\\Input\\Tiles\\";
	std::string outputFolder = "Path\\to\\Output\\Tiles\\";

	std::vector<std::string> plot_FileNames;
	ListFilesInDirectory(inPlotFolder, plot_FileNames);


	//Parameters for filtering;
	float minGridSize = 0.3;
	float maxGridSize = 15.0;
	float Height_Threshold = 0.5;
	float resolution = 0.1;


	int fileIndex = 0;
	for (auto& file : plot_FileNames)
	{

		PointCloud* temp = nullptr;
		std::string filePath = inPlotFolder + file;
		temp = new CLazPointData(); 

		FilterVegetation* filter = nullptr;
		if (temp && temp->LoadFile(filePath))
		{
			filter->SetPointCloud(temp);
			filter->SetHeightThreshold(Height_Threshold);

			filter->IterativeVoxelFiltering(maxGridSize, minGridSize, (minGridSize - maxGridSize) / 10.0);
		}
		if (temp)
		{
			delete temp;
			temp = nullptr;
		}

		std::vector<dPoint3D> terrainPts;
		for (size_t i = 0; i < filter->m_pointcloud->m_PtsNum; ++i)
		{
			if(filter->m_pointcloud->m_PtsClassInfo[i] == 0)
			{
				terrainPts.push_back(filter->m_pointcloud->m_Points[i]);
			}
		}

		int		hf_times = 5;
		int		sm_times = 2; 

		
		PointCloud* terrainData = nullptr;
		terrainData = new CLazPointData();
		terrainData->m_Points = new dPoint3D[terrainPts.size()]; 
		terrainData->m_PtsNum = terrainPts.size();
		for (size_t k = 0; k < terrainPts.size(); ++k) 
		{
			terrainData->m_Points[k] = terrainPts[k];

			if (terrainData->m_Points[k].x > terrainData->m_xMax)
			{
				terrainData->m_xMax = terrainData->m_Points[k].x;
			}
			if (terrainData->m_Points[k].y > terrainData->m_yMax)
			{
				terrainData->m_yMax = terrainData->m_Points[k].y; 
			}
			if (terrainData->m_Points[k].z > terrainData->m_xMax)
			{
				terrainData->m_zMax = terrainData->m_Points[k].z; 
			}
			if (terrainData->m_Points[k].x < terrainData->m_xMin) 
			{
				terrainData->m_xMin = terrainData->m_Points[k].x; 
			}
			if (terrainData->m_Points[k].y < terrainData->m_yMin) 
			{
				terrainData->m_yMin = terrainData->m_Points[k].y; 
			}
			if (terrainData->m_Points[k].z < terrainData->m_zMin) 
			{
				terrainData->m_zMin = terrainData->m_Points[k].z; 
			}
		}

		DTM* dtm = new DTM();
		DtmData dtmData;

		dtm->CreateDTM(terrainData, &dtmData, resolution);
		for (int i = 0; i < hf_times; ++i)
		{
			dtm->HoleFilling(&dtmData);
		}
		for (int i = 0; i < sm_times; ++i)
		{
			dtm->Smooth(&dtmData);
		}

		
		std::vector<dPoint3D> dtmPts; 
		for (int i = 0; i < dtmData.m_dtm_header.Row; ++i)
		{
			for (int j = 0; j < dtmData.m_dtm_header.Col; ++j)
			{
				double x, y;
				double z;

				x = dtmData.m_dtm_header.OffsetX + i * dtmData.m_dtm_header.StepX; 
				y = dtmData.m_dtm_header.OffsetY + j * dtmData.m_dtm_header.StepY; 
				z = dtmData.m_dtm_data[j * dtmData.m_dtm_header.Col + i];

				if (z == -99999)
				{
					continue;
				}
				dPoint3D pt;
				pt.x = x;
				pt.y = y;
				pt.z = z;

				dtmPts.push_back(pt); 
			}
		}
		
		if (terrainData) 
		{
			delete terrainData; 
			terrainData = nullptr;
		}
		if (dtm)
		{
			delete dtm; 
			dtm = nullptr;
		}
		

		std::vector<dPoint3D> old_pts; 
		std::vector<dPoint3D> pts; 
		const int order = 1;

		int curr_order = 0;
		while (curr_order <= order)
		{
			std::vector<dPoint3D> new_pts;
			kdTree* kdtree = nullptr;
			PointCloud2<double> kdPts;
			
			kdPts.pts.resize(dtmPts.size());
			for (size_t i = 0; i < dtmPts.size(); ++i)
			{
				kdPts.pts[i].x = dtmPts[i].x; 
				kdPts.pts[i].y = dtmPts[i].y;
				kdPts.pts[i].z = dtmPts[i].z;
			}
			kdtree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
			kdtree->buildIndex();

			for (auto& pt : dtmPts)
			{
				dPoint3D currPt = pt;
				double queryPt[3];
				queryPt[0] = pt.x;
				queryPt[1] = pt.y; 
				queryPt[2] = pt.z; 

				float search_radius_sqr = std::pow(0.3,2);
				std::vector<std::pair<size_t, double>> pairs; 
				SearchParams params; 
				double sum_height = 0.0;
				const size_t num_pairs = kdtree->radiusSearch(&queryPt[0], search_radius_sqr, pairs, params); 
				for (size_t k = 1; k < num_pairs; ++k)
				{
					size_t id = pairs[k].first;
					sum_height += dtmPts[id].z;
				}
				currPt.z = sum_height / (num_pairs * 1.0);

				new_pts.push_back(currPt); 
			}

			if (curr_order == 0)
			{
				old_pts = new_pts; 
			}
			dtmPts = new_pts; 
			curr_order++;

			if (kdtree != nullptr) 
			{ 
				delete kdtree; 
				kdtree = nullptr; 
			}
		}


		std::vector<dPoint3D> delta_H;
		std::vector<double> values;
		int idx = 0;
		for (size_t i = 0; i < old_pts.size(); ++i) 
		{
			dPoint3D currPt; 
			currPt.x = dtmPts[i].x;  
			currPt.y = dtmPts[i].y;  
			currPt.z = dtmPts[i].z;   
			currPt.val = old_pts[i].z - dtmPts[i].z; 
			currPt.ptID = idx++;
			delta_H.push_back(currPt);
			values.push_back(old_pts[i].z - dtmPts[i].z);  
		}

		std::vector<dPoint3D> trail_pts;
		std::vector<dPoint3D> non_trial_pts;
		float kappa = 0.7;
		splitPoints(delta_H, kappa, trail_pts, non_trial_pts);

	}

	return 0;
}
