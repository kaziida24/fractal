#ifndef ASCII2PCD_HELP_HPP_
#define ASCII2PCD_HELP_HPP_

// This header file simply contains the necessary includes 

// Includes: 
// System includes: 
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>

// PCL includes: 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ascii2pcd_help.hpp"

// Namespaces: 
using namespace std; 

// Initializations: 
string ascii_filename; 
string help_flag = "-h"; 

// Function prototypes 
void ascii2pcd_help(); 

#endif 