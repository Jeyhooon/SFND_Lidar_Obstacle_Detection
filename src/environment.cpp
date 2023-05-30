/* \author Aaron Brown */
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include <unordered_set>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"
#include "kd_tree.h"
#include "helper_func.h"

// using templates for processPointClouds so also include .cpp to help linker
template<typename PointT>
typename std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	while (maxIterations--)
	{
		// Randomly sample subset (three points) and fit plane
		std::unordered_set<int> inliers; // set only contains unique elements!
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin(); // returns a iterator pointer to the beginning of the inliers (access its value by de-referencing)
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float b = (x3 - x1) * (z2 - z1) - (x2 - x1) * (z3 - z1);
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -(a * x1 + b * y1 + c * z1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0) // if index exists in the set; then skip
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x0 = point.x;
			float y0 = point.y;
			float z0 = point.z;

			float dist = fabs(a * x0 + b * y0 + c * z0 + d) / sqrt(a * a + b * b + c * c); // be careful to use float abs: fabs()
			if (dist <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted plane with most inliers
	return inliersResult;
}

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputClouds = lidar->scan();
    // renderRays(viewer, lidar->position, inputClouds);
    // renderPointCloud(viewer, inputClouds, "input_clouds");

    // TODO:: Create point processor
    // creating the obj on the stack
    // ProcessPointClouds<pcl::PointXYZ> pointProcessorStack;
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessorStack.SegmentPlane(inputClouds, 100, 0.2);
    // creating the obj on the heap
    // ProcessPointClouds<pcl::PointXYZ> *pointProcessorHeap = new ProcessPointClouds<pcl::PointXYZ>();
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessorHeap->SegmentPlane(inputClouds, 100, 0.2);
    
    // TODO: Use segmentation from Quiz (developed ourselves using 3D kdtree)
    std::unordered_set<int> inliers = RansacPlane<pcl::PointXYZ>(inputClouds, 100, 0.2f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZ>());

    KdTree *tree = new KdTree;
    for (int index = 0; index < inputClouds->points.size(); index++)
	{
		pcl::PointXYZ point = inputClouds->points[index];
		if (inliers.count(index))
			planeCloud->points.push_back(point);
		else
			obstacleCloud->points.push_back(point);
            tree->insert(point, index);
	}

    // renderPointCloud(viewer, segmentCloud.first, "obstClouds", Color(1, 0, 0));
    renderPointCloud(viewer, planeCloud, "planeClouds", Color(0, 1, 0));

    // TODO: use clustering from Quiz (developed ourselves)
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorHeap->Clustering(obstacleCloud, 1.0, 3, 30);
    std::vector<std::vector<int>> clusters = euclideanCluster(obstacleCloud, tree, 1.0);
    std::cout << "clustering found " << clusters.size() << endl;
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 0.9, 0), Color(0, 0.2, 1)};
    int num_colors = colors.size();

    for (std::vector<int> cluster : clusters)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
		for (int indice : cluster)
			clusterCloud->points.push_back(obstacleCloud->points[indice]);
		
        renderPointCloud(viewer, clusterCloud, "cluster" + std::to_string(clusterId), colors[clusterId % 3]);

        Box box = bounding_box(clusterCloud);
        renderBox(viewer, box, clusterId, colors[clusterId % num_colors], 0.2);

		++clusterId;
	}
    
    // USING pointProcessorHeap OBJECT:
    // for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    // {
    //     std::cout << "cluster size ";
    //     pointProcessorHeap->numPoints(cluster);
    //     renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % num_colors]);
    //     Box box = pointProcessorHeap->BoundingBox(cluster);
    //     renderBox(viewer, box, clusterId, colors[clusterId % num_colors], 0.2);
    //     ++clusterId;
    // }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    Eigen::Vector4f minPoint(-10.0f, -6.0f, -2.0f, 0.0f);
    Eigen::Vector4f maxPoint(18.0f, 6.5f, 2.0f, 0.0f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, 0.1f, minPoint, maxPoint);
    // renderPointCloud(viewer, filteredCloud, "filteredCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "planeClouds", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.5, 10, 100000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 0.9, 0), Color(0, 0.2, 1)};
    int num_colors = colors.size();

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % num_colors]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId % num_colors], 0.2);

        ++clusterId;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Create point processor
    // creating the obj on the heap
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);
    // cityBlock(viewer);

    while (!viewer->wasStopped())
    {

        // Clear v~iewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce(100);
    }
}