
#include"clip_las.h"

#include"../shapelib/shapelib/shapefil.h"

#include<iostream>
#include<vector>


int main(int argc, char** argv)
{
	//
	//std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\NeverGrazed\\NeverGrazed.shp";
	//std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedBefore1990\\GrazedBefore1990.shp";
	//std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedAfter1990\\GrazedAfter1990.shp";


	//std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\Grazed2019\\Grazed2019.shp";
	//std::string lasFilePath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\All\\";
	//std::string lasFileOutputPath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\All\\Clipped\\";


	//std::string shpFilePath = "D:\\Projects\\Python\\DownloadFrance\\shp\\perimetre_bagnas_fusionne_20221123.shp";
	//std::string lasFilePath = "D:\\Projects\\Python\\DownloadFrance\\downloads\\";
	//std::string lasFileOutputPath = "D:\\Projects\\Python\\DownloadFrance\\output\\";


//	std::string shpFilePath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\Polygons\\2020_PROTOCOLE_ROSELIERES_BAGNAS\\2020_PROTOCOLE_ROSELIERES_BAGNAS.shp";
//	std::string lasFilePath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\Bagnas\\PointClouds\\";
	//std::string lasFilePath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\Bagnas\\pcd\\";
//	std::string lasFileOutputPath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\Reedbed\\";



	std::string shpFilePath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\Polygons\\gestion_agropastorale_bagnas_2024\\gestion_agropastorale_bagnas_2024.shp";
	std::string lasFilePath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\Bagnas\\PointClouds\\";
	std::string lasFileOutputPath = "D:\\Data\\2024.05.15 MAMBO\\MAMBO\\Data\\FR\\vegetation1\\";


	mm::ClipLas* clip = new mm::ClipLas;
	
	//clip->getAllLasFiles(lasFilePath);
	clip->ListFilesInDirectory(lasFilePath);
	clip->setOutputLasPath(lasFileOutputPath);
	clip->setShpFilePath(shpFilePath);
	clip->readShpFile2();
	clip->getPositivePolygons(); 
	clip->setLasFileDirName(lasFilePath);
	clip->runClipping3(); 




	return 0;
}
