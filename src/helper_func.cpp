
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "processPointClouds.h"
#include "helper_func.h"

Box bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
	// Initialize the box
	Box box = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
				-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() };

	// Calculate the minimum and maximum coordinates on each axis
	for (const auto& point : cluster->points)
	{
		box.x_min = std::min(box.x_min, point.x);
		box.y_min = std::min(box.y_min, point.y);
		box.z_min = std::min(box.z_min, point.z);
		box.x_max = std::max(box.x_max, point.x);
		box.y_max = std::max(box.y_max, point.y);
		box.z_max = std::max(box.z_max, point.z);
	}

	return box;
}


void cluster_helper(int index, const pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);

	std::vector<int> nearest = tree->search(points->points[index], distanceTol);
	for (int id : nearest)
	{
		if (!processed[id])
			cluster_helper(id, points, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr points, KdTree *tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points->size(), false);

	for (int i = 0; i < points->size(); i++)
	{
		if (processed[i])
			continue;

		std::vector<int> cluster;
		cluster_helper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
	}

	return clusters;
}

