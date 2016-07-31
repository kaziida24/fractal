// practice file 

#include <iostream>
#include <stdio.h>
#include <string>
#include <omp.h>
#include <ctime>
#include <vector>

// #include "hello.hpp"
#include "fractalAnalysis.hpp"

// PCL includes: 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

using namespace std; 

int main(int argc, char** argv)
{

	// fractal(); 
	// hello();

	string filename = "../Wrapped_Orange 1.pcd";
	// string filename = "../Wrapped_Orange 1 sampled.pcd"; 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(filename, *cloud);

	// Initialize Octree
	float resolution = 32.0f;

  	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

  	octree.setInputCloud (cloud);
  	octree.addPointsFromInputCloud ();

  	// Set search point 
  	pcl::PointXYZ searchPoint;

  	// searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  	// searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  	// searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  	searchPoint.x = 641.3; 
  	searchPoint.y = 557.6; 
  	searchPoint.z = 591.9; 

  	// Radius search 
  	vector<int> pointIdxRadiusSearch;
  	vector<float> pointRadiusSquaredDistance;

  	float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  	cout << "Neighbors within radius search at (" << searchPoint.x 
      	<< " " << searchPoint.y 
      	<< " " << searchPoint.z
      	<< ") with radius=" << radius << endl;


    int start_time = clock(); 
  	if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  	{
    // 	for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    //   	{
    //   		cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
    //             	<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
    //             	<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
    //             	<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
  		// }
  		printf("%d neighbers found within a %f micrometer radius search of (%f, %f, %f)\n", 
  			int(octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)),
  			radius, searchPoint.x, searchPoint.y, searchPoint.z); 
  	}
  	else 
  	{
  		printf("No neighbors found within a %f micrometer radius search of (%f, %f, %f)\n", 
  			radius, searchPoint.x, searchPoint.y, searchPoint.z); 
  	}
  	int stop_time = clock(); 
  	printf("Radius search time: %f ms\n", 1000.0*(stop_time-start_time)/double(CLOCKS_PER_SEC)); 
	

	return 0;
}