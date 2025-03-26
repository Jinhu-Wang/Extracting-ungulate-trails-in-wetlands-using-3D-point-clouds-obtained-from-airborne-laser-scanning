Folder **[Workflow]** contains the source code for the workflow, with its contents described below:

## Table of Contents

- [Overview](#overview)
- [File Structure](#file-structure)
- [Requirements](#requirements)
- [Usage Instructions](#usage-instructions)
- [Example](#example)
- [License](#license)
- [Contact](#contact)

## Overview

This repository contains the input data and source code for extracting ungulate trails in the marsh area of [Oostvaardersplassen](https://www.staatsbosbeheer.nl/uit-in-de-natuur/locaties/oostvaardersplassen) using 3D point clouds obtained from airborne laser scanning, specifically [AHN4](https://www.arcgis.com/home/webscene/viewer.html?webscene=c6db29808aad459cbf6488cd96828e9a) point clouds. The workflow consists of four major steps:

1. **Pre-processing**  
2. **Near-terrain filtering**  
3. **Digital Terrain Model (DTM) generation**  
4. **Trail extraction**  

## File Structure

```plaintext
Workflow/
│
├── 0_dependencies/          # Required dependencies, including LAStools, PointCloud data structures, and nanoflann for neighborhood searching.
│
├── 1_clipping/              # Source code for clipping the original point clouds into smaller tiles.
│
├── 2_near_terrain_filtering/ # Filtering near-terrain and vegetation points from the re-tiled point clouds.
│
├── 3_dtm_generation/        # Generation of Digital Terrain Models (DTMs) from the re-tiled point clouds.
│
└── 4_trail_extraction/      # Extraction of trails from the generated DTMs.
```

## Requirements

### C++ Code

The `C++` scripts in this repository depend on **[LAStools](https://lastools.github.io/)** for reading point cloud data in **LAS/LAZ** formats.

To use the scripts, a `C++` compiler such as `g++`, `gcc`, `msvc`, or `clang++` must be installed.


## Usage Instructions 

### There are three ways to build the project:

- **Option 1: Using CMake to generate Makefiles and then running `make` (Linux/macOS).**

  - On Linux or macOS:
    ```sh
    cd path-to-root-dir-of-project
    mkdir release && cd release
    cmake -DCMAKE_BUILD_TYPE=release ..
    make
    ```
  - On Windows with Microsoft Visual Studio, use the `x64 Native Tools Command Prompt for VS XXXX` (**do not** use the x86 version). Then run:
    ```sh
    cd path-to-root-dir-of-project
    mkdir Release && cd Release
    cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
    nmake
    ```

- **Option 2: Using an IDE that supports CMakeLists.txt.**  
  Open the `CMakeLists.txt` file in the **root** directory of the project with an IDE such as [CLion](https://www.jetbrains.com/clion/) or [QtCreator](https://www.qt.io/product). Then, build the project.  

  **Note for Windows users:** Ensure that your IDE is set to `x64`.  

- **Option 3: Using CMake-GUI to generate project files for an IDE.**  
  Load the generated project into your IDE and build it.  

  **Note for Windows users:** Ensure that your IDE is set to `x64`.  

## Example 

### Example Usage of the C++ Module `[1_clipping]`

```cpp
#include "clip_las.h"
#include "../shapelib/shapelib/shapefil.h"

#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
    // Set the file and folder paths.
    std::string shpFilePath = "Path\\to\\shapefile.shp";
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

    if (clip)
    {
        delete clip;
        clip = nullptr;
    }
    return 0;
}
```



## License

This project is licensed under the **MIT License**.

## Contact

For any suggestions or bug reports, please contact:

**Jinhu Wang**  
📧 jinhu.wang (at) hotmail (dot) com  
