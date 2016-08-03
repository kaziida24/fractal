// Program to perform fractal analysis

// Other includes: 
#include "fractalAnalysis.hpp"

// Overall algorithm flow
// Step 0. Take text file and convert to to .PCD file for easier use in PCL. 
//         Can already complete this step in MATLAB (See matpcl)
// Step 1. Read .PCD file
// Step 2. Create occupancy grid as Eigen matrix type 
// Step 3. Perform fractal analysis on occupancy grid 
// Step 4. Write N and R to text file
// Step 5. Plot log(R) vs. log(N)

/* 

Create command-line interface for easy use: 

Run fractal analysis on specified PCD file and specified output file
./fractal filename.pcd output.txt

Show help menu
./fractal -h

./fractal filename.pcd 

or 

./fractal -h

Export results of fractal analysis to text file 
Plot using matplotlib or MATLAB (maybe use a command-line argument)

*/

// Main code 
int main (int argc, char** argv)
{

	// Parse command-line arguments 

	if (argc != 2)
	{
		cout << "Check usage again." << endl; 
		return -1; 
	}
	else
	{
		if (!strcmp(argv[1], help_msg.c_str()))
		{
			fractal_help(); 
			return -2; 
		}
		else
		{
			filename = argv[1]; 
			int position = int(filename.rfind('.'));
			string newName = filename.substr(0, position); 
			outputFile = newName+"_output.txt";  
		}
	}

	fullpath = path_addition+filename; 
	printf("Analyzing %s\n", fullpath.c_str()); 

	////////////////////////////////////////////////////////////////////////////////
	// Step 1. Read .PCD file
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(fullpath, *cloud);

	VectorXf x_cloud = ArrayXf::Zero(int(cloud->width)); 
	VectorXf y_cloud = ArrayXf::Zero(int(cloud->width));
	VectorXf z_cloud = ArrayXf::Zero(int(cloud->width));

	// Parallelize this section 
	int start_time = clock(); 
	#pragma omp parallel
	{
		#pragma omp for 
		for (int i = 0; i < cloud->points.size (); i++)
	  	{
		    x_cloud(i) = float(cloud->points[i].x);
		    y_cloud(i) = float(cloud->points[i].y);
		    z_cloud(i) = float(cloud->points[i].z); 
		}
	}
	int stop_time = clock(); 

	printf("Step 1. Finished reading file %s containing %d points.\n", 
		filename.c_str(), int(cloud->width));
	printf("Loop time: %f seconds\n", double(double(stop_time)-double(start_time))/(double(CLOCKS_PER_SEC)));

	/////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////
	// Step 2. Create occupancy grid as Eigen matrix type 
	// Search the point cloud using an octree 

	// Init octree: 
	float resolution = 32.0f; 
  	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

  	octree.setInputCloud (cloud);
  	octree.addPointsFromInputCloud();

	// Grid parameters
	float grid_scale = 1.0;

	float x_min = x_cloud.minCoeff();
	float x_max = x_cloud.maxCoeff();
	int Nx = 1024; 

	float y_min = y_cloud.minCoeff();
	float y_max = y_cloud.maxCoeff();
	int Ny = 1024; 

	float z_min = z_cloud.minCoeff();
	float z_max = z_cloud.maxCoeff();
	int Nz = 1024; 

	VectorXf x_grid = VectorXf::LinSpaced(Nx, x_min, x_max);
	VectorXf y_grid = VectorXf::LinSpaced(Ny, y_min, y_max);
	VectorXf z_grid = VectorXf::LinSpaced(Nz, z_min, z_max); 

	int gridSize = Nx*Ny*Nz;

	printf("Occupancy grid contains %d points.\n", gridSize); 
	printf("Nx = %d, Ny = %d, Nz = %d\n", Nx, Ny, Nz); 

	typedef boost::multi_array<int, 3> array_type; 
	typedef array_type::index index; 
	array_type grid(boost::extents[Nx][Ny][Nz]); 

	int start_s = clock();

	printf("Starting to fill occupancy grid.\n");

	int fillPoint = 0;
	int counter = 0; 
	bool go; 
	int q; 

	// Init search parameters: 
	pcl::PointXYZ searchPoint; 
	float radius = 10.0f; 
	unsigned int max_value = 1; 

	#pragma omp parallel 
	{
		vector<int> pointIdxRadiusSearch; 
		vector<float> pointRadiusSquaredDistance;
		#pragma omp for 
		for (index i = 0; i < Nx; i++)
		{
			for (index j = 0; j < Ny; j++)
			{
				for (index k = 0; k < Nz; k++)
				{
					counter++; 

					// Current point 
					searchPoint.x = x_grid(i); 
					searchPoint.y = y_grid(j); 
					searchPoint.z = z_grid(k); 

					// Search for nearest points inside the point cloud 
					if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, max_value) > 0)
					{
						grid[i][j][k] = 1; 
						fillPoint++; 
						printf("Filled: %d/%d, Empty: %d/%d, %f percent complete\n", fillPoint, gridSize,
							gridSize-fillPoint, gridSize, float(float(counter)/float(gridSize))*100.0); 
					}
					else
					{
						grid[i][j][k] = 0; 
					}

				}
			}
		}
	}

	int stop_s = clock();

	printf("Step 2. Finished making occupancy grid.\n");
	printf("Grid time: %f ms\n", double(stop_s-start_s)/(double(CLOCKS_PER_SEC)));

	/////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////
	// Step 3. Perform fractal analysis 

	// Place fractal code here: 

	// Preprocessing: 
	Vector3f gridDim = Array3f::Zero(3); 
	gridDim(0) = Nx; 
	gridDim(1) = Ny; 
	gridDim(2) = Nz; 
	double width = gridDim.maxCoeff();	 
	double p = ceil(log(width)/log(2.0)); 

	VectorXf n = ArrayXf::Zero(int(p)+1); 

	// Output values: 
	VectorXf r = ArrayXf::Zero(int(p)+1);
	r(0) = 1.0; 
	n(0) = 1.0; 
	for (int i = 1; i < int(r.size()); i++) 
	{

		r(i) = pow(2.0, double(i)); 
	} 

	// Implement boxcounting 

	double siz; 
	double siz2; 
	float running_sum; 

	n(int(p)) = fillPoint; 

	for (int g = int(p)-1; g > 0; g--)
	{
		siz = pow(2.0, double(p-g)); 
		siz2 = round(siz/2.0); 

		running_sum = 0; 

		for (int a = 0; a < int(width-siz+1); a = a+int(siz))
		{
			for (int b = 0; b < int(width-siz+1); b = b+int(siz))
			{
				for (int c = 0; c < int(width-siz+1); c = c+int(siz))
				{
					grid[a][b][c] = (bool(grid[a][b][c]) || bool(grid[a+siz2][b][c]) 
						|| bool(grid[a][b+siz2][c]) || bool(grid[a+siz2][b+siz2][c]) 
						|| bool(grid[a][b][c+siz2]) || bool(grid[a+siz2][b][c+siz2]) 
						|| bool(grid[a][b+siz2][c+siz2]) || bool(grid[a+siz2][b+siz2][c+siz2])); 
					running_sum = running_sum+grid[a][b][c]; 
				}
			}
		}
		n(g) = running_sum; 
	}

	// Flip n: 
	n = n.colwise().reverse().eval(); 

	printf("Step 3. Finished performing fractal analysis.\n");
	/////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////
	// Step 4. Write results to text file (N and R)
	fp = fopen(outputFile.c_str(), "w"); 
	for (int i = 0; i < int(r.size()); i++)
	{
		fprintf(fp, "%f %f\n", r(i), n(i));
	}
	fclose(fp); 

	printf("Step 4. Wrote results to %s.\n", outputFile.c_str());
	/////////////////////////////////////////////////////////////////////////////////


	// Exit program
	return 0;
}