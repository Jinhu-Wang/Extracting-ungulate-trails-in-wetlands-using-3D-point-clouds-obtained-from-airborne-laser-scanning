

//#include<iostream>
//#include<vector>
//#include<string>
//
//#include"../PointCloud/PointCloud.hpp"
//#include"FilterVegetation.h"
//
//
//int main()
//{
//	std::string lasFile = "Path\\to\\Point\\Cloud\\Data";
//	PointCloud* temp = new CLazPointData();
//
//	temp->LoadFile(lasFile.c_str());
//
//	FilterVegetation* filter = new FilterVegetation();
//
//	filter->SetPointCloud(temp);
//	filter->SetHeightThreshold(0.8);
//
//	float minGridSize = 0.1;
//	float maxGridSize = 10.0;
//
//	filter->IterativeVoxelFiltering(maxGridSize, minGridSize, (minGridSize - maxGridSize) / 10.0);
//
//	return 0;
//}