Folder [Workflow] consists of the source code of the workflow and the contents are described below:

## Table of Contents

- [Overview](#overview)
- [File Structure](#file-structure)
- [Requirements](#requirements)
- [Usage Instructions](#usage-instructions)
- [Example](#example)
- [License](#license)
- [Contact](#contact)

## Overview
This repository holds the input data and the source codes for extracting ungulate trails in marsh area of [Oostvaardersplassen](https://www.staatsbosbeheer.nl/uit-in-de-natuur/locaties/oostvaardersplassen) using 3D point clouds obtained from airborne laser scanning, i.e. [AHN4](https://www.arcgis.com/home/webscene/viewer.html?webscene=c6db29808aad459cbf6488cd96828e9a) point clouds. Their are four major steps in the workflow: 1) Pre-processing; 2) Near-Terrain filtering; 3) Digital Terrain Model generation; and 4) Trail extraction. 

## File Structure

```plaintext
Workflow/
│
├── 0_dependencies/      # The used dependencies, i.e. LASTools, PointCloud data structure and nanoflann for neighbourhood searching.
|
├── 1_clipping/          # The source code for clipping the original point clouds to smaller tiles.
|
├── 2_near_terrain_filtering/       # Filtering of the near-terrain and the vegetation points from the re-tiled original point clouds.
|
├── 3_dtm_generation/       # Generate the Digital Terrain Models of the re-tiled point clouds.
|
└── 4_trail_extraction/       # Extract the trails from the generated DTMs.

```

## Requirements

### 'C++' codes

The `C++` scripts in this repository depends on the **[LAStools](https://lastools.github.io/)** to read point cloud data in **LAS/LAZ** formats.

To use the scripts, a `C++` compiler, i.e. `g++`,`gcc`, `mscv`, `clang++`, etc., should be installed.

### 'Python' codes

The 'Python' can be run either on Colab or on your local environment, just to ensure the required packages are correctly installed.


## Usage Instructions 

### There are three ways to build:

- Option 1: Using CMake to generate makefiles and then 'make' (on Linux/macOS).

  - On Linux or maxOS, simply:
    ```
    $ cd path-to-root-dir-of-project
    $ mkdir release  && cd release
    $ cmake -DCMAKE_BUILD_TYPE=release ..
    $ make
    ```
  - On Windows with Microsoft Visual Studio, use the `x64 Native Tools Command Prompt for VS XXXX` (**don't** use the x86 one), then
    ```
      $ cd path-to-root-dir-of-project
      $ mkdir Release
      $ cd Release
      $ cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
      $ nmake
    ```

- Option 2: Use any IDE that can directly handle CMakeLists files to open the `CMakeLists.txt` in the **root** directory of Trees3D.
  Then you should have obtained a usable project and just build it. I recommend using
  [CLion](https://www.jetbrains.com/clion/) or [QtCreator](https://www.qt.io/product). For Windows users: your IDE must be set for `x64`.
  
- Option 3: Use CMake-Gui to generate project files for your IDE. Then load the project to your IDE and build it. For Windows users: your IDE must be set for `x64`.


## Example 
Below shows an example on the usage of 'C++' Module [1_clipping] 

```javascript {.line-number}

#include"clip_las.h"
#include"../shapelib/shapelib/shapefil.h"

#include<iostream>
#include<vector>

int main(int argc, char** argv)
{
	// Set the file and folder paths.
	std::string shpFilePath = "Path\\to\\shapefile";
	std::string lasFilePath = "Path\\to\\Input\\LAS\\Files\\";
	std::string lasFileOutputPath = "Path\\to\\Output\\LAS\\Files\\";

  	mm::ClipLas* clip = new mm::ClipLas;
	
	clip->ListFilesInDirectory(lasFilePath);
	clip->setOutputLasPath(lasFileOutputPath);
	clip->setShpFilePath(shpFilePath);
	clip->readShpFile2();
	clip->getPositivePolygons(); 
	clip->setLasFileDirName(lasFilePath);
	clip->runClipping3(); 

  	if(clip)
  	{
    		delete clip;
    		clip=nullptr;
  	}
	return 0;
}
````
Below shows an example on the usage of 'Python' Module [3_dtm_generation]

```javascript {.line-number}

input_laz = "Path/to/LAS/file"  # Replace with your LiDAR file path

# Set the DTM resolution in meters
resolution = 1.0 

# Generate and visualize DTM using KNN
# Here, six options are available, i.e., IDW, NNI, KRIGING, SPLINE, TIN, KNN
dtm = generate_dtm(input_laz, resolution, method="IDW")
``` 

## License

MIT License

## Contact

For any suggestions and bug reports, please contact:

Jinhu Wang

jinhu.wang (at) hotmail (dot) com



