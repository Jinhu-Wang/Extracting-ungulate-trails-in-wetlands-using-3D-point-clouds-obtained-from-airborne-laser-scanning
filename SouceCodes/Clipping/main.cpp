
//#include"clip_las.h"
#include"types.h"

#include"../shapelib/shapelib/shapefil.h"
#include"../RodeFox/nanoflann.hpp"

#include"../LASlib/lasreader.hpp"
#include"../LASlib/laswriter.hpp"


#include<iostream>
#include<vector>
#include<math.h>
#include<string>

using namespace nanoflann;
typedef KDTreeSingleIndexAdaptor < L2_Simple_Adaptor<double, PointCloud2<double>>,
	PointCloud2<double>, 3 > kdTree;


std::vector<std::vector< mm::ShpPolygon2>> trees_polygons;
std::vector<mm::dvec2> non_tree_points;
constexpr double pi = 3.141592653589793;


bool readTreeShpFile(std::string shpFilePath)
{
	SHPHandle hSHP = SHPOpen(shpFilePath.c_str(), "rb");
	if (hSHP == nullptr) 
	{
		return false;
	}

    /*
    A pointer to an integer into which the number of entities/structures
    should be placed. May be NULL;
    */
    int pnEntites; 

    /*
    A pointer to an integer into which the shapetype of this file should be placed.
    Shapefiles may contain eigher SHPT_POINT, SHPT_ARC, SHPT_POLYGON or
    SHPT_MULTIPOINT entities. This may be NULL;
    */
    int pnShapetype; 

    /*
    The x, y, z and m minimum/maximum values will be placed into this four entry array.
    */
    double adfMinBound[4]; 
    double adfMaxBound[4]; 

    SHPGetInfo(hSHP, &pnEntites, &pnShapetype, adfMinBound, adfMaxBound); 

    for (int i = 0; i < pnEntites; ++i)
    {
        SHPObject* psShape = SHPReadObject(hSHP, i);
        if (psShape == nullptr)
        {
            continue;
        }

        std::vector<mm::ShpPolygon2> polygon; 

        // Read only polygos;
        if (psShape->nSHPType == 5)
        {
            //mm::ShpPolygon2 currPolygon;
           /* if (psShape->nParts == 1)
            {
                mm::dvec2 vertex; 
                mm::ShpPolygon2 currPolygon;  

                for (int j = 0; j < psShape->nVertices; ++j)
                {
                    vertex.x = psShape->padfX[j];
                    vertex.y = psShape->padfY[j];
                    currPolygon.push_back(vertex);
                }
                polygon.push_back(currPolygon); 
            }

            else if (psShape->nParts > 1)
            {
                for (int j = 0; j < psShape->nParts; ++j)
                {
                    mm::dvec2 vertex;
                    mm::ShpPolygon2 currPolygon; 

                    int index = psShape->panPartStart[j];
                    while (index < psShape->panPartStart[j + 1] &&
                        index < psShape->nVertices)
                    {
                        vertex.x = psShape->padfX[index];
                        vertex.y = psShape->padfY[index];
                        currPolygon.push_back(vertex);
                        index++;
                    }
                    polygon.push_back(currPolygon); 
                }
            }*/
            
            if (psShape->nParts > 1)
            {
                for (int j = 0; j < psShape->nParts; ++j)
                {
                    int startIndex = psShape->panPartStart[j];
                    int endIndex = (j == psShape->nParts - 1) ? psShape->nVertices : psShape->panPartStart[j + 1]; 
                    
                    mm::dvec2 vertex;
                    mm::ShpPolygon2 currPolygon; 
                    for (int m = startIndex; m < endIndex; ++m)
                    {
                        vertex.x = psShape->padfX[m]; 
                        vertex.y = psShape->padfY[m]; 
                        currPolygon.push_back(vertex); 
                    }
                    polygon.push_back(currPolygon);  
                }
            }

            trees_polygons.push_back(polygon); 
        }
    }

    std::cout << "A total of: [ " << trees_polygons.size() << " ] polygons were loaded" << std::endl; 
    return true;
}


bool readTreeShpFile2(std::string shpFilePath)
{
    SHPHandle hSHP = SHPOpen(shpFilePath.c_str(), "rb");
    if (hSHP == nullptr)
    {
        return false;
    }

    /*
    A pointer to an integer into which the number of entities/structures
    should be placed. May be NULL;
    */
    int pnEntites;

    /*
    A pointer to an integer into which the shapetype of this file should be placed.
    Shapefiles may contain eigher SHPT_POINT, SHPT_ARC, SHPT_POLYGON or
    SHPT_MULTIPOINT entities. This may be NULL;
    */
    int pnShapetype;

    /*
    The x, y, z and m minimum/maximum values will be placed into this four entry array.
    */
    double adfMinBound[4];
    double adfMaxBound[4];

    SHPGetInfo(hSHP, &pnEntites, &pnShapetype, adfMinBound, adfMaxBound);

    for (int i = 0; i < pnEntites; ++i)
    {
        SHPObject* psShape = SHPReadObject(hSHP, i);
        if (psShape == nullptr)
        {
            continue;
        }

        std::vector<mm::ShpPolygon2> polygon;

        // Read only polygos;
        if (psShape->nSHPType == 5)
        {
            //mm::ShpPolygon2 currPolygon;
            if (psShape->nParts == 1) 
            {
                mm::dvec2 vertex; 
                mm::ShpPolygon2 currPolygon; 

                for (int j = 0; j < psShape->nVertices; ++j) 
                {
                    vertex.x = psShape->padfX[j]; 
                    vertex.y = psShape->padfY[j]; 
                    currPolygon.push_back(vertex); 
                }
                polygon.push_back(currPolygon); 
            } 
            /*
            else if (psShape->nParts > 1)
            {
                for (int j = 0; j < psShape->nParts; ++j)
                {
                    mm::dvec2 vertex;
                    mm::ShpPolygon2 currPolygon;

                    int index = psShape->panPartStart[j];
                    while (index < psShape->panPartStart[j + 1] &&
                        index < psShape->nVertices)
                    {
                        vertex.x = psShape->padfX[index];
                        vertex.y = psShape->padfY[index];
                        currPolygon.push_back(vertex);
                        index++;
                    }
                    polygon.push_back(currPolygon);
                }
            }*/

            /*if (psShape->nParts > 1)
            {
                for (int j = 0; j < psShape->nParts; ++j)
                {
                    int startIndex = psShape->panPartStart[j];
                    int endIndex = (j == psShape->nParts - 1) ? psShape->nVertices : psShape->panPartStart[j + 1];

                    mm::dvec2 vertex;
                    mm::ShpPolygon2 currPolygon;
                    for (int m = startIndex; m < endIndex; ++m)
                    {
                        vertex.x = psShape->padfX[m];
                        vertex.y = psShape->padfY[m];
                        currPolygon.push_back(vertex);
                    }
                    polygon.push_back(currPolygon);
                }
            }*/

            trees_polygons.push_back(polygon);
        }
    }

    std::cout << "A total of: [ " << trees_polygons.size() << " ] polygons were loaded" << std::endl;
    return true;
}

bool readNonTreePointsShpFile(std::string shpFilePath)
{
    SHPHandle hSHP = SHPOpen(shpFilePath.c_str(), "rb"); 
    if (hSHP == nullptr) 
    {
        return false;
    }

    /*
      A pointer to an integer into which the number of entities/structures
      should be placed. May be NULL;
      */
    int pnEntites; 

    /*
    A pointer to an integer into which the shapetype of this file should be placed.
    Shapefiles may contain eigher SHPT_POINT, SHPT_ARC, SHPT_POLYGON or
    SHPT_MULTIPOINT entities. This may be NULL;
    */
    int pnShapetype; 

    /*
    The x, y, z and m minimum/maximum values will be placed into this four entry array.
    */
    double adfMinBound[4]; 
    double adfMaxBound[4]; 

    SHPGetInfo(hSHP, &pnEntites, &pnShapetype, adfMinBound, adfMaxBound); 

    for (int i = 0; i < pnEntites; ++i)
    {
        SHPObject* psShape = SHPReadObject(hSHP, i);
        if (psShape == nullptr)
        {
            continue;
        }
        
        //Read only points;
        if (psShape->nSHPType == 1)
        {
            mm::dvec2 vertex;
            for (int j = 0; j < psShape->nVertices; ++j)
            {
                vertex.x = psShape->padfX[j];
                vertex.y = psShape->padfY[j];
                
                non_tree_points.push_back(vertex); 
            }
        }
    }

    std::cout << "A total of: [ " << non_tree_points.size() << " ] points were loaded." << std::endl;

    return true;
}

bool isPositivePolygon(std::vector<mm::ShpPolygon2> polygon)
{
    if (polygon.empty())
    {
        return false;
    }
    
    
    return true;
}


double computePolygonArea(std::vector<mm::ShpPolygon2> polygon)
{
    if (polygon.empty())
    {
        return 0.0;
    }
    double signed_area = 0.0;
    double area = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        signed_area = polygon.at(i).signed_area();

        area += signed_area; 
    }
    return area; 
}

double computePolygonArea(std::vector<double>& area, std::vector<double>& radius)
{
    if (trees_polygons.empty())
    {
        return 0.0;
    }
    for (size_t i = 0; i < trees_polygons.size(); ++i)
    {
        double curr_area = 0.0; 
        double curr_radius = 0.0;
        std::vector<mm::ShpPolygon2> polygons = trees_polygons.at(i);
        std::vector<double> areaaa;
        for (size_t j = 0; j < polygons.size(); ++j) 
        {
            curr_area += - polygons.at(j).signed_area();
            areaaa.push_back(polygons.at(j).signed_area());
        }
        
        curr_radius = std::sqrt(curr_area/pi);
        
        radius.push_back(curr_radius);
        area.push_back(curr_area);
    }
}


bool ptInsidePolygonSet(std::vector<mm::ShpPolygon2> polygons, mm::dvec2 pt)
{
    bool inside = false;
    for (int i = 0; i < polygons.size(); ++i)
    {
        //Positive polygon;
        if (!polygons.at(i).is_clockwise())
        {
            if (polygons.at(i).contains(pt))
            {
                inside = true;
            }
        }
        else
        {
            if (polygons.at(i).contains(pt))
            {
                inside = false;
            }
        }

    }
    return inside; 
}

int main_tree_clip(int argc, char** argv)
{
    //std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\All\\Clipped\\Grazed2019.laz";
    //std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\All\\Clipped\\NeverGrazed.laz";
    //std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\All\\Clipped_2m\\GrazedBefore1990.laz";
    std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\All\\Clipped_2m\\GrazedAfter1990.laz";

   
    //std::string shpFilePath_Points = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\NeverGrazed\\100_Random_Points\\NeverGrazed_100_Random_Points.shp"; 
    //std::string shpFilePath_Points = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedBefore1990\\100_Random_Points\\GrazedBefore1990_100_Random_Points.shp";
    std::string shpFilePath_Points = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedAfter1990\\100_Random_Points\\GrazedAfter1990_Random_100_Points.shp";
    //std::string shpFilePath_Points = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\Grazed2019\\100_Random_Points\\Grazed2019_Random_100_Points.shp";


    //std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\NeverGrazed\\100_Random_TreesShrubs\\NeverGrazed_100_Random_TreesShrubs.shp";
    //std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedBefore1990\\100_Random_TreesShrubs\\GrazedBefore1990_100_Random_TreesShrubs.shp";
    //std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedAfter1990\\100_Random_TreesShrubs\\GrazedAfter1990_Random_100_TreesShrubs.shp";
    //std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\Grazed2019\\100_Random_TreesShrubs\\Grazed2019_Random_100_TreesShrubs.shp";
    
    //std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\NeverGrazed\\100_Random_TreesShrubs_2m\\NeverGrazed_100_TreesShrubs_2m_buffer.shp";
    //std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedBefore1990\\100_Random_TreesShrubs_2m\\GrazedBefore1990_100_Random_2m_Buffer.shp";
    std::string shpFilePath_Trees = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\ShpFiles\\GrazedAfter1990\\100_Random_TreesShrubs_2m\\GrazedAfter1990_Random_100_2m_Buffer.shp";

	std::string lasOutputTreesDir = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\GrazedAfter1990\\2m\\Trees_2m\\";
	std::string lasOutputPointsDir = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\GrazedAfter1990\\2m\\Points_2m\\";

    readNonTreePointsShpFile(shpFilePath_Points);

    readTreeShpFile(shpFilePath_Trees); 
    
    std::vector<double> area_polygons; 
    std::vector<double> radius;

    computePolygonArea(area_polygons, radius); 

   
    LASreadOpener readOpener; 
    readOpener.set_file_name(lasInputFile.c_str(), true);  
    LASreader* reader = readOpener.open(); 
    if (!reader) 
    {
        reader = nullptr; 
        return 1; 
    }
    if (reader->npoints == 0) 
    {
        reader->close(); 
        delete reader; 
        reader = nullptr; 
        return 1;
    }
    
    std::vector<LASwriter*> writersTrees(area_polygons.size());
    std::vector<LASwriter*> writersPoints(area_polygons.size());
    for (size_t i = 0; i < area_polygons.size(); ++i)
    {
        std::string treeOutPath = lasOutputTreesDir + "TreeShrub_Plot_" + std::to_string(i) + ".laz";
        LASwriteOpener writeOpenerTrees;
        writeOpenerTrees.set_file_name(treeOutPath.c_str());
        writersTrees[i] = writeOpenerTrees.open(&reader->header);


        std::string pointsOutPath = lasOutputPointsDir + "Points_Plot_" + std::to_string(i) + ".laz";
        LASwriteOpener writeOpenerPoints;
        writeOpenerPoints.set_file_name(pointsOutPath.c_str());
        writersPoints[i] = writeOpenerPoints.open(&reader->header);
    }

    long ptCount = 0;
    while (reader->read_point())
    {
        if (ptCount % 100000 == 0)
        {
            std::cout<<"[ "<<ptCount<<"/"<< reader->npoints << " ] number of points have been parsed."<<std::endl;
        }
        mm::dvec2 pt;
        pt.x = reader->point.get_x(); 
        pt.y = reader->point.get_y();

        int Index = -1; 

        for (int i = 0; i < trees_polygons.size(); ++i)
        {
            std::vector<mm::ShpPolygon2> currPolygon = trees_polygons.at(i);
            
            if (ptInsidePolygonSet(currPolygon, pt))
            {
                Index = i;

                LASpoint lasPt;
                lasPt.init(&reader->header, 
                    reader->header.point_data_format,
                    reader->header.point_data_record_length, nullptr);
                lasPt = reader->point; 

                writersTrees[Index]->write_point(&lasPt); 
                writersTrees[Index]->update_inventory(&lasPt); 
            }
        }

        for (int i = 0; i < non_tree_points.size(); ++i)
        {
            mm::dvec2 currPt = non_tree_points.at(i);
            double dist = mm::distance(currPt, pt); 
            if (dist < radius.at(i))
            {
                Index = i; 

                LASpoint lasPt;
                lasPt.init(&reader->header,
                    reader->header.point_data_format, 
                    reader->header.point_data_record_length, nullptr);
                lasPt = reader->point;

                writersPoints[Index]->write_point(&lasPt);
                writersPoints[Index]->update_inventory(&lasPt); 
            }
        }
        ptCount++;
    }


    for (int i = 0; i < trees_polygons.size(); ++i)
    {
        writersTrees[i]->update_header(&reader->header, true);
        writersTrees[i]->close();
        delete writersTrees[i]; 
        writersTrees[i] = nullptr;
    }
    for (int i = 0; i < non_tree_points.size(); ++i)
    {
        writersPoints[i]->update_header(&reader->header, true);
        writersPoints[i]->close(); 
        delete writersPoints[i];
        writersPoints[i] = nullptr;
    }

    reader->close();
    delete reader;
    reader = nullptr;

	return 0;
}

// For clipping to different las files
int main___()
{
    //std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\OutOost\\All_No_Trees.laz";
    //std::string lasOutputFolder = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\OutOost\\out_plots_1\\";
    //std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\OutOost\\50_squares.shp";

    //std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\NeverGrazed.laz";
    //std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\Polygons\\NeverGrazed_New_1.laz";
   
   /* std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\NeverGrazed_New_1.laz";
    std::string lasOutputFolder = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\NeverGrazed\\";
    std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\Validation_Plots_Polygon\\Never_Grazed\\Never_Grazed_50Plots.shp";*/
    
    /*std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\GrazedBefore1990_New.laz";
    std::string lasOutputFolder = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\GrazedBefore1990\\";
    std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\Validation_Plots_Polygon\\Grazed_Before_1990\\Grazed_Before_1990_50Plots.shp"; */
    

    /*std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\GrazedAfter1990_New.laz";
    std::string lasOutputFolder = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\GrazedAfter1990\\";
    std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\Validation_Plots_Polygon\\Grazed_After_1990\\Grazed_After_1990_50Plots.shp";*/


    std::string lasInputFile = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\Grazed2019_New.laz";
    std::string lasOutputFolder = "D:\\TestData\\Oostvaardersplassen\\GeeseGrazingAreas\\PCD\\50_Plots\\Grazed2019\\";
    std::string shpFilePath = "D:\\TestData\\Oostvaardersplassen\\Validation_Plots_Polygon\\Grazed_In_2019\\Grazed_In_2019_50Plots.shp";


    


    LASreadOpener readOpener;
    readOpener.set_file_name(lasInputFile.c_str(), true);
    LASreader* reader = readOpener.open();
    if (!reader)
    {
        reader = nullptr;
        return 1;
    }
    if (reader->npoints == 0)
    {
        reader->close();
        delete reader;
        reader = nullptr;
        return 1;
    }



    readTreeShpFile2(shpFilePath);

    std::vector<LASwriter*> writersPolygons(trees_polygons.size());
    for (size_t i = 0; i < trees_polygons.size(); ++i)
    {
        std::string plotOutPath = lasOutputFolder + "Plot_" + std::to_string(i) + ".laz";
        LASwriteOpener writeOpener;
        writeOpener.set_file_name(plotOutPath.c_str());
        writersPolygons[i] = writeOpener.open(&reader->header);
    }

    long ptCount = 0;
    while (reader->read_point())
    {
        mm::dvec2 pt;
        pt.x = reader->point.get_x();
        pt.y = reader->point.get_y();

        int Index = -1;
        for (int i = 0; i < trees_polygons.size(); ++i)
        {
            std::vector<mm::ShpPolygon2> currPolygon = trees_polygons.at(i);
            if (ptInsidePolygonSet(currPolygon, pt))
            {
                Index = i; 

                LASpoint lasPt;
                lasPt.init(&reader->header,
                    reader->header.point_data_format, 
                    reader->header.point_data_record_length, nullptr);
                lasPt = reader->point; 

                writersPolygons[Index]->write_point(&lasPt);
                writersPolygons[Index]->update_inventory(&lasPt); 
                break;
            }
        }
        ptCount++;
    }

    for (int i = 0; i < trees_polygons.size(); ++i)
    {
        writersPolygons[i]->update_header(&reader->header, true);
        writersPolygons[i]->close();
        delete writersPolygons[i];
        writersPolygons[i] = nullptr;
    }

    reader->close();
    delete reader;
    reader = nullptr;



    return 0;
}