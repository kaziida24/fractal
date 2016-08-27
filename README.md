<!-- A few fractal images here?  -->
# Fractal: A General Purpose Architecture for Estimating the Fractal Dimension of any Pattern or Geometry
Status: Experimental  
Written by Kazuaki Iida  

*Read this in Japanese: [日本語版ガイド](README.ja.md)*  

**Fractal** provides a convenient framework to estimate the fractal dimension of any pattern contained in an image, or any geometry contained in a point cloud using the box counting method. This framework is a command-line implementation of the MATLAB implementation found [here](https://www.mathworks.com/matlabcentral/fileexchange/13063-boxcount/content/boxcount/html/demo.html). The motivation and background for the development of the code is described on the [project page](https://kaziida24.github.io/fractal). 

## Table of Contents
- [Summary](#summary)
	- [Capabilties](#capabilities)
	- [2D Input Case](#2d-input-case)
	- [3D Input Case](#3d-input-case)
	- [Executables](#executables)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Running the fractal analysis program on an image](#running-the-fractal-analysis-program-on-an-image)
  - [Converting an ASCII file to a more PCL friendly PCD file](#converting-an-ascii-file-to-a-more-pcl-friendly-pcd-file)
  - [Running the fractal analysis program on a 3D dataset](#running-the-fractal-analysis-program-on-a-3d-dataset)
  - [Visualize a dataset using the PCL native viewer](#visualize-a-dataset-using-the-pcl-native-viewer)
  - [Getting help](#getting-help)
- [Example](#example)
- [Questions and Contact Information](#questions-and-contact-information)

## Summary

### Capabilities

| Input Data Dimension | Primary Feature | Secondary Feature |
| :------------------: | :------: | :-------: |
| 2D (image)           | fractal analysis core program | N/A |
| 3D (point cloud)     | fractal analysis core code | visualization tool (point clouds), point cloud file conversion |

### 2D Input Case
![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/2d_flowchart.png "2D Input Figure")

### 3D Input case 
![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/3d_flowchart.png "3D Input Figure")

### Executables

| Name | Programming language used | Description |
| :--: | :-----------------------: | :---------: |
| fractal2d | C++ | fractal analysis program for image (2D pattern) |
| fractal3d | C++ | fractal analysis program for 3D dataset | 
| visualize | C++ | visualize a 3D dataset using PCL's native viewer |
| ascii2pcd | C++ | convert an ASCII point cloud to a PCD file | 
| plot_fractal.py | Python | post-process fractal analysis results by creating a plot to help estimate the fractal dimension | 
| plot_fractal.m  | MATLAB | MATLAB version of post-processing program | 
| visualizeDataset.m |  MATLAB | renders any ASCII point cloud dataset using MATLAB. You will need the [computer vision toolbox](http://www.mathworks.com/products/computer-vision/) to use this. |

## Dependencies
* CMake (version 2.8 or higher)
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
Call the *ascii2pcd* executable along with the filename of the dataset. If you have an ASCII point cloud file, this is the preceding step before the *fractal3d* executable is called. 

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

## Example

This example will detail the procedure in estimating the fractal dimension of the pattern in the following image. 

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/images/fractal_img.png "Sample Image")

This image must first be saved in the *images* folder. This is the default directory where *fractal2d* will look for the image file. The analysis can be initiated by running the executable with the corresponding image filename. In this case, the image filename is **fractal_img.png**. 

```bash
$ ./fractal2d fractal_img.png 
```

The on-screen instructions will tell you to press spacebar to continue. Please follow the instructions until it states that the analysis is complete. 

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/fractal2d_example1.png)

You will now have the option to post-process directly in Python. Post-processing produces the following plots. 

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/test1_loglog.png) ![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/test2_slope.png)

On the top is the canonical fractal analysis plot showing a log-log plot of the number of boxes needed to fill the image and the corresponding bin size. The slope in the constant region of the slope plot is around 1.65. According to this result, the pattern contained in the image is indeed a fractal set since it has a noninteger fractal dimension that is larger than its topological dimension (1). The topological dimension is 1 since the branch can be approximated as a continuous curve.  

## Questions and Contact Information 
Project page: https://kaziida24.github.io/fractal/  
Email: kiida2@illinois.edu
