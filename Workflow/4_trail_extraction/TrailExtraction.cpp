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
#include<Windows.h>

// Local
#include"../File/FileData.h"
#include"../Filter/Filter.h"
#include"../DemMaker/Interpolation.h"
#include"../RodeFox/nanoflann.hpp"
#include"../MarchingSquares/MarchingSquares.h"


using namespace nanoflann;
typedef KDTreeSingleIndexAdaptor < L2_Simple_Adaptor<double, PointCloud2<double>>,
	PointCloud2<double>, 3 > kdTree;

std::wstring stringToWString(const std::string& str)
{
	if (str.empty()) return std::wstring();
	int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
	std::wstring wstr(sizeNeeded, 0);
	MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstr[0], sizeNeeded);
	return wstr;
}

std::string wstringToString(const std::wstring& wstr)
{
	if (wstr.empty()) return std::string();
	int sizeNeeded = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
	std::string str(sizeNeeded, 0);
	WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &str[0], sizeNeeded, NULL, NULL);
	return str;
}

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



void ListFilesInDirectory_xyz(const std::string& directoryPath, std::vector<std::string>& outFiles)
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
				std::string fileName = wstringToString(findData.cFileName);
				if (fileName.size() > 4 && fileName.substr(fileName.size() - 4) == ".xyz")
				{
					outFiles.push_back(fileName);
				}
			}
		} while (FindNextFile(hFind, &findData) != 0);
		FindClose(hFind);
	}
	else
	{
		std::cerr << "Failed to list files in directory." << std::endl;
	}
}


double calculateStandardDeviation(std::vector<double>& vec, double& mean)
{
	if (vec.empty())
	{
		return 0; // Return 0 or handle as error for empty vector
	}

	mean = std::accumulate(vec.begin(), vec.end(), 0.0f) / vec.size();
	double sq_sum = std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0f,
		std::plus<>(), [mean](double a, double b)
		{
			return (a - mean) * (b - mean);
		});

	return std::sqrt(sq_sum / vec.size());
}


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



// Raw trails points;

int main()
{
	std::string input_dir = "G:\\Data\\20240920_Oost_Trails_FrontierRS\\1_Original_PCD\\0_3_FourRegions\\All_in_one___Retiled_400_out_dtm_xyz_paral_re-arrange_deltaH\\";
	std::string output_dir = "G:\\Data\\20240920_Oost_Trails_FrontierRS\\1_Original_PCD\\0_3_FourRegions\\All_in_one___Retiled_400_out_dtm_xyz_paral_re-arrange_deltaH_Trails\\";

	std::vector<std::string> file_names;
	ListFilesInDirectory_xyz(input_dir, file_names);


	const double threshold = -0.003344;

	for (const auto& file : file_names)
	{
		std::vector<dPoint3D> points;
		std::string file_path = input_dir + file;
		
		FILE* delta_h_file = fopen(file_path.c_str(), "r");
		if (delta_h_file == nullptr)
		{
			return 0;
		}
		while (!feof(delta_h_file))
		{
			dPoint3D pt;
			fscanf(delta_h_file, "%lf %lf %lf %lf\n",
				&pt.x, &pt.y, &pt.z, &pt.val);
			if (pt.val > threshold)
			{
				continue;
			}
			points.push_back(pt);
		}
		fclose(delta_h_file);


		kdTree* kdtree = nullptr;
		PointCloud2<double> kdPts;

		kdPts.pts.resize(points.size());
		for (size_t i = 0; i < points.size(); ++i)
		{
			kdPts.pts[i].x = points[i].x;
			kdPts.pts[i].y = points[i].y;
			kdPts.pts[i].z = points[i].z;
		}
		kdtree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
		kdtree->buildIndex();

		std::vector<dPoint3D> raw_trail_pts;
		for (const auto& pt : points)
		{
			dPoint3D currPt = pt;
			double query_pt[3];

			query_pt[0] = pt.x;
			query_pt[1] = pt.y;
			query_pt[2] = pt.z;

			std::vector<std::pair<size_t, double>> ret_pairs;
			SearchParams params;
			if (kdtree->radiusSearch(&query_pt[0], 0.8 * 0.8, ret_pairs, params) > 40)
			{
				raw_trail_pts.push_back(pt);
			}
		}

		if (kdtree != nullptr)
		{
			delete kdtree;
			kdtree = nullptr;
		}

		std::string output_file_name = output_dir + file;
		FILE* out_file = fopen(output_file_name.c_str(), "w");
		for (size_t i = 0; i < raw_trail_pts.size(); ++i)
		{
			fprintf(out_file, "%lf %lf %lf\n",
				raw_trail_pts[i].x, raw_trail_pts[i].y, raw_trail_pts[i].z);
		}
		fclose(out_file);
	}


	return 0;
}











int main__test(int argc, char** argv)
{
	std::string input_dir = "G:\\Data\\20240920_Oost_Trails_FrontierRS\\1_Original_PCD\\0_3_FourRegions\\All_in_one___Retiled_400_out_dtm_xyz_paral_a\\";
	std::string output_dir = "G:\\Data\\20240920_Oost_Trails_FrontierRS\\1_Original_PCD\\0_3_FourRegions\\All_in_one___Retiled_400_out_dtm_xyz_paral_deltaH_a\\";
	
	//std::string input_dir = "G:\\Data\\20240920_Oost_Trails_FrontierRS\\1_Original_PCD\\0_3_FourRegions\\All_in_one___Retiled_400_out_dtm_xyz_paral_re-arrange\\";
	//std::string output_dir = "G:\\Data\\20240920_Oost_Trails_FrontierRS\\1_Original_PCD\\0_3_FourRegions\\All_in_one___Retiled_400_out_dtm_xyz_paral_re-arrange_deltaH\\";


	std::vector<std::string> file_names;
	ListFilesInDirectory(input_dir, file_names);

	for (const auto& file : file_names)
	{
		std::vector<dPoint3D> points;

		std::string file_path = input_dir + file;

		FILE* xyz_file = fopen(file_path.c_str(), "r");
		if (xyz_file == nullptr)
		{
			return 0;
		}
		while (!feof(xyz_file))
		{
			dPoint3D pt;
			fscanf(xyz_file, "%lf %lf %lf\n",
				&pt.x, &pt.y, &pt.z);
			points.push_back(pt);
		}

		fclose(xyz_file);
		xyz_file = nullptr;

		//std::vector<dPoint3D> old_points;
		//std::vector<dPoint3D> pts;

		// Smoothing;
		std::vector<dPoint3D> new_pts;

		kdTree* kdtree = nullptr;
		PointCloud2<double> kdPts;

		bool isKdTreeBuilt = false;

		kdPts.pts.resize(points.size());
		for (size_t i = 0; i < points.size(); ++i)
		{
			kdPts.pts[i].x = points[i].x;
			kdPts.pts[i].y = points[i].y;
			kdPts.pts[i].z = points[i].z;
		}
		kdtree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
		kdtree->buildIndex();

		for (const auto& pt : points)
		{
			dPoint3D currPt = pt;
			double query_pt[3];

			query_pt[0] = pt.x;
			query_pt[1] = pt.y;
			query_pt[2] = pt.z;

			const size_t ret_num = 64;
			std::vector<size_t> ret_index(ret_num);
			std::vector<double> out_dist_sqr(ret_num);
			kdtree->knnSearch(&query_pt[0], ret_num, &ret_index[0], &out_dist_sqr[0]);

			double sum_height = 0.0;
			for (size_t k = 1; k < ret_num; ++k)
			{
				size_t id = ret_index[k];
				sum_height += points[id].z;
			}
			currPt.z = sum_height / ((ret_num - 1) * 1.0);

			new_pts.push_back(currPt);
		}

		std::vector<dPoint3D> delta_h;
		std::vector<double> values;
		size_t idx = 0;
		for (size_t i = 0; i < new_pts.size(); ++i)
		{
			dPoint3D currPt;
			currPt.x = points[i].x;
			currPt.y = points[i].y;
			currPt.z = points[i].z;
			currPt.val = points[i].z - new_pts[i].z;
			currPt.ptID = idx++;
			delta_h.push_back(currPt);
			values.push_back(points[i].z - new_pts[i].z);
		}

		std::string output_file_name = output_dir + file;
		FILE* out_file = fopen(output_file_name.c_str(), "w");

		for (size_t i = 0; i < delta_h.size(); ++i)
		{
			fprintf(out_file, "%lf %lf %lf %lf\n",
				delta_h[i].x,
				delta_h[i].y,
				delta_h[i].z,
				delta_h[i].val);
		}
		fclose(out_file);

	}

}
