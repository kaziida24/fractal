# Fractal Analysis code documentation 
Status: Experimental  
Written by Kazuaki Iida  

*Read this in Japanese: [日本語版ガイド](README.ja.md)*  

## Table of Contents
- [Summary](#summary)
  - [C++ executables](#c-executables)
  - [Python and MATLAB scripts](#python-and-matlab-scripts)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Converting an ASCII file to a more PCL friendly PCD file](#converting-an-ascii-file-to-a-more-pcl-friendly-pcd-file)
  - [Running the fractal analysis main program](#running-the-fractal-analysis-main-program)
  - [Visualize a dataset using the PCL native viewer](#visualize-a-dataset-using-the-pcl-native-viewer)
  - [Getting help](#getting-help)
- [Questions and Contact Information](#questions-and-contact-information)

## Summary

### C\+\+ executables
*fractal* - fractal analysis entry program (performs fractal analysis on a given 3D dataset)  
*visualize* - visualize a dataset using PCL's native viewer  
*ascii2pcd* - convert an ASCII point cloud to a PCD file 

### Python and MATLAB scripts
Additionally, Python and MATLAB scripts faciliate post-processing the fractal analysis results as well as provide an alternative to
point cloud visualizations. 

*plot_fractal.py* - post-process fractal analysis results by creating a plot and estimating the fractal dimension  
*plot_fractal.m* - same as *plot_fractal.py* but in MATLAB instead of Python  
*visualizeDataset.m* - renders any point cloud dataset using the MATLAB viewer. You will need the computer vision toolbox to use this.  
## Dependencies
* CMake  
* PCL (for point cloud processing and visualization)
* Eigen
* Boost
* Python and/or MATLAB (for plotting and optional visualization)

## Installation
*The following installation has only been checked on Ubuntu 14.04.*  

* Clone the repository to local computer.  
```bash
$ git clone https://github.com/kaziida24/fractal
```  
* Move into the fractal directory, make a "build" directory, and move into the "build" directory.
```bash
$ cd fractal
$ mkdir build
$ cd build
```  
* Call CMake to autogenerate the Makefile and the MATLAB and Python scripts.  
```bash
$ cmake ..
```  
* Build the project using make. 
```bash
$ make all 
```  

## Usage  

### Converting an ASCII file to a more PCL friendly PCD file
Call the *ascii2pcd* executable along with the filename of the dataset.  

Example: This command takes an ASCII file called *dataset.txt* and creates a PCD file called *dataset.pcd*. 
```bash
$ ./ascii2pcd dataset.txt 
```
### Running the fractal analysis main program
Call the *fractal* executable along with the filename of the input PCD file, as well as the filename of the output file. 

```bash 
$ ./fractal dataset.pcd output.txt
```

### Visualize a dataset using the PCL native viewer  
Call the *visualize* executable along with the filename of the PCD file you would like to visualize. 
```bash
$ ./visualize dataset.pcd 
```

### Getting help 
Any of the executables can be run with the help flag *(-h)* to display the help menu, which explains how 
to run the executable and what command line arguments to include. 

Example: Access help menu for *ascii2pcd*. 
```bash
$ ./ascii2pcd -h
```

Example: Access help menu for *fractal*. 
```bash
$ ./fractal -h
```

Example: Access help menu for *visualize*. 
```bash
$ ./visualize -h
```

## Questions and Contact Information 
Project page: https://kaziida24.github.io/fractal/  
Email: kiida2@illinois.edu
