#ifndef FRACTALANALYSIS_HPP_
#define FRACTALANALYSIS_HPP_

// This header file simply contains the necessary includes 

// Includes: 
// System includes: 
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include "boost/multi_array.hpp"
#include <ctime>
#include <vector>
#include <math.h>

// PCL includes: 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

// OpenMP includes: 
#include <omp.h>

// Other includes: 
#include "fractalAnalysis.hpp"

// Namespaces: 
using namespace std; 
using namespace Eigen; 

// Initializations: 
FILE* fp; 
string outputFile; 
string filename; 
string path_addition = "../datasets/"; 
string fullpath; 
bool plot_flag; 
string plot_option = "-p"; 
string help_msg = "-h"; 

// Function prototypes 
void fractal_help(); 

#endif // FRACTALANALYSIS_HPP_