// Convert ASCII point cloud file to a more PCL friendly PCD file 

#include "ascii2pcd_help.hpp"

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << "Check usage again. Type ./ascii2pcd -h to see the help message." << endl; 
		return -1; 
	}
	else
	{
		if (!strcmp(argv[1], help_flag.c_str()))
		{
			ascii2pcd_help(); 
			return -2; 
		}
		else
		{
			ascii_filename = argv[1]; 
		}
	}

	// Init point cloud object: 
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Add the filename to its path. 
	string fullpath = "../datasets/" + ascii_filename; 

	printf("Filename: %s\n", ascii_filename.c_str()); 
	printf("Full relative path: %s\n", fullpath.c_str());

	int position = int(fullpath.rfind('.')); 
	string newpath = fullpath.substr(0,position); 
	string target_name = newpath+".pcd"; 

	printf("Target filename: %s\n", target_name.c_str()); 

	ifstream myfile(fullpath.c_str(), ios_base::in);

	float n1, n2, n3; 
	int k = 0; 
	if (myfile.is_open())
	{
	    while (myfile >> n1 >> n2 >> n3)
		{
		    k++; 
		}
	   	myfile.close(); 
	}
	    
	printf("%d points in %s\n", k, ascii_filename.c_str()); 

	cloud.width = k; 
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	ifstream textfile(fullpath.c_str(), ios_base::in);
	size_t i = 0; 
	float x_file, y_file, z_file; 
	if (textfile.is_open())
	{
	  	while (textfile >> x_file >> y_file >> z_file)
	  	{
	  		cloud.points[i].x = x_file; 
		    cloud.points[i].y = y_file; 
		    cloud.points[i].z = z_file;
		    i++;  
	  	}
	  	textfile.close(); 
	}

	pcl::io::savePCDFileASCII(target_name.c_str(), cloud);

	printf("Finished converting %s to PCD file. Check the datasets folder.\n", 
	    	ascii_filename.c_str()); 

	return 0; 
}