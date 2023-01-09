#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

template <typename PointT>
typename std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointT>::Ptr cloud, int maxIterations, float distanceTol);