/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, typename pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud)
{
    //ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    //typename pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion = pointProcessor.FilterCloud(pointCloud, 0.25f, Eigen::Vector4f(-10,-5,-2,1), Eigen::Vector4f(30,6.5f,10,1));

    //renderPointCloud(viewer, cloudRegion, "cloudRegion");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pairCloud = pointProcessor.SegmentPlaneAlex(cloudRegion, 100, 0.2f);
    
    //First is the plane
    renderPointCloud(viewer, pairCloud.first, "plane_cloud", Color(0.0, 1.0, 0.0 ));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor.ClusteringAlex(pairCloud.second, 0.4f, 15, 400);

    std::vector<Color> colors { Color(1.0, 0.0, 0.0 ), Color(1.0, 1.0, 0.0 ), Color(0.0, 0.0, 1.0 ) };

    for(int id = 0; id < clusters.size(); ++id)
    {
        pointProcessor.numPoints(clusters[id]);
        renderPointCloud(viewer, clusters[id], std::string("cluster") + std::to_string(id), colors[id%3]);
        Box box = pointProcessor.BoundingBox(clusters[id]);
        renderBox(viewer, box, id);
    }

}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 

    //Lidar * lidar = new Lidar(cars, 0);

    Lidar * lidar = new Lidar(cars, 0);
    

    // TODO:: Create point processor    

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> pair = pointProcessor.SegmentPlane(lidar->scan(), 100, 0.2);
    
    //renderPointCloud(viewer, pair.first, std::string("first"), Color(1.0,0.0,0.0));
    //renderPointCloud(viewer, pair.second, std::string("second"), Color(0.0,1.0,0.0));

    //NOTE: estaba muy corto el tolerance!
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.Clustering(pair.second, 1.0, 3, 30);

    std::vector<Color> colors { Color(1.0, 0.0, 0.0 ), Color(0.0, 1.0, 0.0 ), Color(0.0, 0.0, 1.0 ) };

    for(int id = 0; id < clusters.size(); ++id)
    {
        pointProcessor.numPoints(clusters[id]);
        renderPointCloud(viewer, clusters[id], std::string("cluster") + std::to_string(id), colors[id]);
        Box box = pointProcessor.BoundingBox(clusters[id]);
        renderBox(viewer, box, id);
    }

    //renderRays(viewer, lidar->position, lidar->scan());

    //NOTE: The pointer is stored 
    //renderRays(viewer, lidar->position, lidar->scan());

    //renderHighway(viewer);

    //renderPointCloud(viewer, lidar->scan(), std::string("Alejandro"), Color(1.0,0.0,0.0));
}


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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    //Son un monton de paths a pcd files!
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1/");

    auto streamIterator = stream.begin();

    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    //typename pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = pointProcessor->loadPcd((*streamIterator).string());
        
    //cityBlock(viewer, *pointProcessor, pointCloud);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //path to string!

        typename pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = pointProcessor->loadPcd((*streamIterator).string());
        
        cityBlock(viewer, *pointProcessor, pointCloud);

        streamIterator++;

        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    } 
}