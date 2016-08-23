<!-- A few fractal images here?  -->
# Fractal: A General Purpose Architecture for Estimating the Fractal Dimension of any Pattern or Geometry
Status: Experimental  
Written by Kazuaki Iida  

*Read this in Japanese: [日本語版ガイド](README.ja.md)*  

## Overview of Program

Fractal provides a convenient framework to estimate the fractal dimension of any pattern contained in an image, or any geometry contained in a point cloud. 

### Capabilities

| Input Data Dimension | Features | 
| :------------------: | :------: |
| 2D                   | fractal analysis core program  |
| 3D                   | fractal analysis core code, visualization tool (point clouds) | 

### 2D Input Case
![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/2d_flowchart.png "Summary of Program Capabilities")

### 3D Input case 
![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/3d_flowchart.png "Summary of Program Capabilities")

## Table of Contents
- [Summary](#summary)
  - [C++ executables](#c-executables)
  - [Python and MATLAB scripts](#python-and-matlab-scripts)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Running the fractal analysis program on an image](#running-the-fractal-analysis-program-on-an-image)
  - [Converting an ASCII file to a more PCL friendly PCD file](#converting-an-ascii-file-to-a-more-pcl-friendly-pcd-file)
  - [Running the fractal analysis program on a 3D dataset](#running-the-fractal-analysis-program-on-a-3d-dataset)
  - [Visualize a dataset using the PCL native viewer](#visualize-a-dataset-using-the-pcl-native-viewer)
  - [Getting help](#getting-help)
- [Examples](#examples)
- [Questions and Contact Information](#questions-and-contact-information)

## Summary

### C\+\+ executables
*fractal2d* - fractal analysis program for image (2D pattern)  
*fractal3d* - fractal analysis program for 3D dataset  
*visualize* - visualize a 3D dataset using PCL's native viewer  
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
* OpenCV (for 2D analysis and image operations)
* Eigen
* Boost
* Python and/or MATLAB (for plotting and optional visualization)

## Installation
**The following installation procedure has only been confirmed on Ubuntu 14.04.**  

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

### Running the fractal analysis program on an image
Call the *fractal2d* executable along with the filename of the input image. The name of the output file will be printed to screen at the end of the program. For example, the following input produces an output file called *image_output.txt*. 

```bash
$ ./fractal2d image.jpg
```


### Converting an ASCII file to a more PCL friendly PCD file
Call the *ascii2pcd* executable along with the filename of the dataset.  

Example: This command takes an ASCII file called *dataset.txt* and creates a PCD file called *dataset.pcd*. 
```bash
$ ./ascii2pcd dataset.txt 
```
### Running the fractal analysis program on a 3D dataset
Call the *fractal3d* executable along with the filename of the input PCD file. The program will print the name of the output file to screen. For example, the following input produces an output file called *dataset_output.txt*. 

```bash 
$ ./fractal3d dataset.pcd
```

### Visualize a dataset using the PCL native viewer  
Call the *visualize* executable along with the filename of the PCD file you would like to visualize. 
```bash
$ ./visualize dataset.pcd 
```

### Getting help 
Any of the executables can be run with the help flag *(-h)* to display the help menu, which explains how 
to run the executable and what command line arguments to include. 

Example: Access help menu for *fractal2d*. 

```bash
$ ./fractal2d -h
``` 

Example: Access help menu for *ascii2pcd*. 
```bash
$ ./ascii2pcd -h
```

Example: Access help menu for *fractal3d*. 
```bash
$ ./fractal3d -h
```

Example: Access help menu for *visualize*. 
```bash
$ ./visualize -h
```

## Examples 

Show a few test cases. 

## Questions and Contact Information 
Project page: https://kaziida24.github.io/fractal/  
Email: kiida2@illinois.edu
