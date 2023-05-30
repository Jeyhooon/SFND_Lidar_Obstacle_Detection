#ifndef HELPER_FUNC_H
#define HELPER_FUNC_H

#include <unordered_set>
#include "kd_tree.h"

void cluster_helper(int index, const pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr points, KdTree *tree, float distanceTol);

Box bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);

#endif // HELPER_FUNC_H