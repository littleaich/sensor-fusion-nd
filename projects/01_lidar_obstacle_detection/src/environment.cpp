/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <unordered_set>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "custom/ransac.h"
#include "custom/cluster.h"

typedef struct Lidar Lidar;


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // if true, cars are shown as well
    // if false, only point cloud is rendered
    bool renderScene = false; 
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // new creates object on the heap rather than stack
    // because lidar holds point cloud data which can be quite large
    // so we want to initiate it in the heap
    Lidar * lidar = new Lidar(cars, 0); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "initial", Color(1,0,0));

    // TODO:: Create point processor
    // on stack
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor();
    // on the heap
    ProcessPointClouds<pcl::PointXYZ> * pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); 
    // get segmented point clouds (obstacles, road)
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr > 
        segmentedClouds = pointProcessor->SegmentPlane(cloud, 100, 0.02);

    // // render both clouds with different color
    // renderPointCloud(viewer, segmentedClouds.first, "obstacles", Color(1,0,0));
    // renderPointCloud(viewer, segmentedClouds.second, "road", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloudClusters 
        { pointProcessor->Clustering(segmentedClouds.first, 1.0, 3, 30) }; 

    // 3 clusters, so hard-coded colors vector for now
    std::vector<Color> colors {Color(1,0,0), Color(0,1,0), 
                            Color(0,0,1), Color(0,1,1)};

    // render each cluster with corresponding color
    renderPointCloudClusters<pcl::PointXYZ> (viewer, cloudClusters, colors);

    renderBoundingBoxes<pcl::PointXYZ> (viewer, pointProcessor, cloudClusters, colors);

}


void cityBlockPCL(pcl::visualization::PCLVisualizer::Ptr & viewer )
// void cityBlockPCL(pcl::visualization::PCLVisualizer::Ptr & viewer,
//                 ProcessPointClouds<pcl::PointXYZI> * pointProcessorI,
//                 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // ----------------------------------------------------
    // Open 3D viewer and display real point cloud data contraining a city block 
    // ----------------------------------------------------    

    // for single file DEBUG
    ProcessPointClouds<pcl::PointXYZI> * pointProcessorI { new ProcessPointClouds<pcl::PointXYZI> };
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud { pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd") };

    renderPointCloud(viewer, cloud, "realCloud");
    std::cout << "Raw point cloud size = " << cloud->width * cloud->height << std::endl;

    cloud = pointProcessorI->FilterCloud(cloud, 0.3f, Eigen::Vector4f (-30,-6.0,-5,1), Eigen::Vector4f(30,7.2,1,1)) ;
    std::cout << "Filtered point cloud size = " << cloud->width * cloud->height << std::endl;
    // renderPointCloud(viewer, cloud, "filteredCloud");

    // plane segmentation
    // get segmented point clouds (obstacles, road)
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr >
        segmentedClouds = pointProcessorI->SegmentPlane(cloud, 100, 0.2) ;
    // render both clouds with different color
    // renderPointCloud(viewer, segmentedClouds.first, "obstacles", Color(1,0,0));
    // renderPointCloud(viewer, segmentedClouds.second, "road", Color(0,1,0));

    // cluster the cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > cloudClusters 
        { pointProcessorI->Clustering(segmentedClouds.first, 0.5, 60, 500) };

    // render each cluster with modulo color
    std::vector<Color> colors {Color(1,0,0)};
    // renderPointCloudClusters<pcl::PointXYZI> (viewer, cloudClusters, colors);
    colors[0] = Color(0,1,1);
    renderBoundingBoxes<pcl::PointXYZI> (viewer, pointProcessorI, cloudClusters, colors);

}


// void cityBlockCustom(pcl::visualization::PCLVisualizer::Ptr & viewer )
void cityBlockCustom(pcl::visualization::PCLVisualizer::Ptr & viewer,
                ProcessPointClouds<pcl::PointXYZI> * pointProcessorI,
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // ----------------------------------------------------
    // Open 3D viewer and display real point cloud data contraining a city block 
    // ----------------------------------------------------    

    // for single file DEBUG
    // ProcessPointClouds<pcl::PointXYZI> * pointProcessorI { new ProcessPointClouds<pcl::PointXYZI> };
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud { pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd") };

    renderPointCloud(viewer, cloud, "realCloud");
    std::cout << "Raw point cloud size = " << cloud->width * cloud->height << std::endl;

    // cloud = pointProcessorI->FilterCloud(cloud, 0.3f, Eigen::Vector4f (-30,-6.0,-5,1), Eigen::Vector4f(30,7.2,1,1)) ;
    cloud = pointProcessorI->FilterCloud(cloud, 0.3f, Eigen::Vector4f (-30,-6.0,-5,1), Eigen::Vector4f(30,7.2,1,1)) ;
    std::cout << "Filtered point cloud size = " << cloud->width * cloud->height << std::endl;
    // renderPointCloud(viewer, cloud, "filteredCloud", Color(0,1,0));

    // plane segmentation
    // get road point indices
    std::unordered_set<int> roadIndices = Ransac3D<pcl::PointXYZI>(cloud, 100, 0.1);
    // extract obstacle cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacles { new pcl::PointCloud<pcl::PointXYZI> };
    for (int i{0}; i<cloud->points.size(); ++i) {
        if (! roadIndices.count(i) ) {
            cloud_obstacles->points.push_back( cloud->points[i] );
        }
    }

    // renderPointCloud(viewer, cloud_obstacles, "obstacles", Color(1,0,0));

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // cluster the cloud
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > cloudClusters 
    //     { pointProcessorI->Clustering(cloud_obstacles, 0.5, 60, 500) };
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > cloudClusters 
        = clustering<pcl::PointXYZI>(cloud_obstacles, 0.5, 10, 500) ;


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds" << std::endl;




    // render each cluster with modulo color
    // std::vector<Color> colors {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0)};
    // renderPointCloudClusters<pcl::PointXYZI> (viewer, cloudClusters, colors);
    std::vector<Color> colors {Color(0,1,1)};
    renderBoundingBoxes<pcl::PointXYZI> (viewer, pointProcessorI, cloudClusters, colors);

}


// int main (int argc, char** argv)
// {
//     std::cout << "starting enviroment" << std::endl;

//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     CameraAngle setAngle = Side;
//     initCamera(setAngle, viewer);
//     // simpleHighway(viewer);
//     // cityBlockPCL(viewer);
//     cityBlockCustom(viewer);

//     while (!viewer->wasStopped ())
//     {
//         viewer->spinOnce ();
//     } 
// }



int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI> * pointProcessorI { new ProcessPointClouds<pcl::PointXYZI> };
    std::vector<boost::filesystem::path> stream { pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1") };
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;


    int countFrame{0};
    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd and run obstacle detection process
        cloud = pointProcessorI->loadPcd( (*streamIterator).string() );
        cityBlockCustom( viewer, pointProcessorI, cloud );

        std::cout << "================ Saving frame " << countFrame << " ================" << std::endl;
        viewer->saveScreenshot("frame_" + std::to_string(countFrame++) + ".png");

        ++streamIterator;
        if (streamIterator == stream.end()) {
            // streamIterator = stream.begin();
            break;
        }

        viewer->spinOnce (50); // in ms

    } 
}
