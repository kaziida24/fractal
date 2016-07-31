/* Fractal analysis source code 

See the header file for more details. 

Input: occupancy grid (boost array?), name of output file
Outputs: No variables, but write to textfile


*/

#include "fractalAnalysis.hpp"

void fractal_help()
{
	printf("\n"); 
	cout << "Welcome to the help menu for the executable, fractal." << endl << 
		"This executable conducts fractal analysis on the geometry specified in a given PCD file." << endl;
	cout << "The results of the fractal analysis are written to a text file that can be used to generate a plot externally" <<
	" (MATLAB) or using Python in the terminal." << endl << endl;  

	cout << "Usage: " << endl; 
	cout << "1. Run executable on specified PCD file and output file." << endl;
	cout << "   ./fractal filename.pcd output.txt" << endl << endl; 

	cout << "2. Access this help message again." << endl; 
	cout << "   ./fractal -h" << endl << endl; 
	return; 
}

void fractal()
{
	cout << "Fractal analysis source code goes here." << endl;  
	return; 
}