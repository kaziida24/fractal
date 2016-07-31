// C++ file that includes the help messages 

#include "ascii2pcd_help.hpp"

void ascii2pcd_help()
{
	printf("\n"); 
	cout << "Welcome to the help menu for the executable, ascii2pcd." << endl << 
		"This executable converts an ASCII point cloud file into a PCD file." << endl << endl; 

	cout << "Usage: " << endl; 
	cout << "1. Run executable on specified ASCII file." << endl;
	cout << "   ./ascii2pcd filename.txt" << endl << endl; 

	cout << "2. Access this help message again." << endl; 
	cout << "   ./ascii2pcd -h" << endl << endl; 

	return; 
}
