# Fractal Analysis code documentation 
Status: Experimental  
Written by Kazuaki Iida  
Updated 7/31/16  

## Table of Contents
- [Summary](#summary)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Questions](#questions)

## Summary
fractal - fractal analysis entry program  
visualize - visualize a dataset using PCL's native viewer  
ascii2pcd - convert an ASCII point cloud to a PCD file 

## Dependencies
CMake (should be standard on Ubuntu)  
PCL (for point cloud processing and visualization)  
Eigen  
Boost (should be standard on Ubuntu)  
Python (for plotting)  
MATLAB (for visualization), optional  

## Installation
1. Clone the repository to local computer.  
'''bash
$ git clone https://github.com/kaziida24/fractal
'''
mkdir build  
cd build  
cmake ..  
make all 

## Usage
Usage: Run fractal with the filename of the dataset 
Before running fractal, make sure input file is PCD file. ASCII to PCD converter included

2 command line arguments. First argument specifies name of PCD file to be analyzed. 
Second argument is a flag that specifies whether a matplotlib plot needs to be generated or not. 

Example: 

Run without plotting (from ASCII file converison step)

./ascii2pcd dataset.txt
./fractal dataset.pcd output.txt

Run with plot (from ASCII file conversion step)

./ascii2pcd dataset.txt
./fractal dataset.pcd output.txt -p 

Visualize a dataset

./visualize dataset.pcd 

Any of the executables can be run with the help flag (-h) to display the help menu, which explains
how to run the executable and what command line arguments to include 

Help for ascii2pcd
./ascii2pcd -h 

Help for fractal 
./fractal -h 

Help for visualize 
./visualize -h

### Steps
1. Parse input file 
2. Load point cloud and parse into x, y, z coordinates 
3. Create occupancy grid 
4. Perform fractal analysis to determine N and R
5. Write results to file 
6. If choose MATLAB for plotting, end program. Otherwise, plot in python

## Questions
### Project page
https://kaziida24.github.io/fractal/




 
