The source codes consist of the following modules:

[Include]
This folder contains the [nanoflann] class, which facilitates efficient neighborhood searching.

[LASlib]
This module is responsible for reading point clouds in LAZ/LAS formats.

[PointCloud]
This module primarily functions as the interface for input/output operations and includes the basic 3D data structures used.

[Clipping]
The clipping process involves providing the program with the path to the ESRI shapefiles, the directory of the LAZ files, and the output directory for the clipped point cloud datasets.

[FilterVegetation]
This module segments near-terrain points from vegetation points.

[CreateDtm]
This module uses the obtained near-terrain points to create a Digital Terrain Model (DTM) from the point cloud data.

[MarchingSquares]
This module takes the generated DTM to perform segmentation of the trails and extract the edge length.

[TrailExtraction]
This module integrates the aforementioned modules to perform trail extraction.

 