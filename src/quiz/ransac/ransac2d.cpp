/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	int bestSize = 0;

	std::unordered_set<int> inliersResult;

	srand(time(NULL));
	

	// TODO: Fill in this function

	// For max iterations 

	for(int i = 0; i < maxIterations; ++i)
	{
		std::unordered_set<int> line;

		// Randomly sample subset and fit line

		while(line.size() < 2)
		{
			line.insert(rand() % (cloud->points.size()));
		}

		auto it = line.begin();

		pcl::PointXYZ p0 = cloud->points[*it];

		++it;
		
		pcl::PointXYZ p1 = cloud->points[*it];

		//NOTE: This is the line.

		float A = (p0.y - p1.y);
		float B = (p1.x - p0.x);
		float C = (p0.x * p1.y - p1.x * p0.y);

		for(int j = 0; j < cloud->points.size(); ++j)
		{
			//auto lineIt = line.find(i);
			if(line.count(j) > 0)
			{
				continue;
			}

			pcl::PointXYZ p = cloud->points[j];
 
			// Measure distance between every point and fitted line
			float d = fabs(A*p.x + B*p.y + C) / sqrt(A*A + B*B);

			// If distance is smaller than threshold count it as inlier

			if(d < distanceTol)
			{
				line.insert(j);
			}
		}

		if(line.size() > inliersResult.size())
		{
			inliersResult = line;
		}
	}



	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	int bestSize = 0;

	std::unordered_set<int> inliersResult;

	srand(time(NULL));
	

	// TODO: Fill in this function

	// For max iterations 

	for(int it = 0; it < maxIterations; ++it)
	{
		std::unordered_set<int> line;

		// Randomly sample subset and fit line

		while(line.size() < 3)
		{
			line.insert(rand() % (cloud->points.size()));
		}

		auto iter = line.begin();

		pcl::PointXYZ p0 = cloud->points[*iter];

		++iter;
		
		pcl::PointXYZ p1 = cloud->points[*iter];

		++iter;

		pcl::PointXYZ p2 = cloud->points[*iter];


		//NOTE: This is the line.

		float a1 = p1.x - p0.x;
		float a2 = p1.y - p0.y;
		float a3 = p1.z - p0.z;

		float b1 = p2.x - p0.x;
		float b2 = p2.y - p0.y;
		float b3 = p2.z - p0.z;

		float i = a2*b3 - a3*b2;
		float j = a3*b1 - a1*b3;
		float k = a1*b2 - a2*b1;

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*p0.x + j*p0.y + k*p0.z);

		for(int j = 0; j < cloud->points.size(); ++j)
		{
			//auto lineIt = line.find(i);
			if(line.count(j) > 0)
			{
				continue;
			}

			pcl::PointXYZ p = cloud->points[j];
 
			// Measure distance between every point and fitted line
			float d = fabs(A*p.x + B*p.y + C * p.z + D) / sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier

			if(d < distanceTol)
			{
				line.insert(j);
			}
		}

		if(line.size() > inliersResult.size())
		{
			inliersResult = line;
		}
	}



	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
