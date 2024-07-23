#pragma once

#include"../PointCloud/PointCloud.hpp"


/// <summary>
/// A class for filtering the vegetation (near-terrain and vegetation) points.
/// </summary>
class FilterVegetation
{

public:
	/// <summary>
	/// Default constructor
	/// </summary>
	FilterVegetation();

	/// <summary>
	/// Constructor with the initialization point cloud data.
	/// </summary>
	/// <param name="pointcloud"></param>
	FilterVegetation(PointCloud* pointcloud);

	/// <summary>
	/// Default desctructor
	/// </summary>
	~FilterVegetation();

	/// <summary>
	/// The input point cloud data.
	/// </summary>
	PointCloud*	m_pointcloud;

	/// <summary>
	/// The input height threshold;
	/// </summary>
	double		m_height_threhold;

	/// <summary>
	/// To set the input point cloud data.
	/// </summary>
	/// <param name="pointcloud"></param>
	void SetPointCloud(PointCloud* pointcloud);

	/// <summary>
	/// To set the height threshold.
	/// </summary>
	/// <param name="height_threshold"></param>
	void SetHeightThreshold(double height_threshold);

	/// <summary>
	/// Perform the iterative filtering.
	/// </summary>
	/// <param name="minVoxel"></param>
	/// <param name="maxVoxel"></param>
	/// <param name="ratio"></param>
	void IterativeVoxelFiltering(double minVoxel, double maxVoxel, double ratio);
};
