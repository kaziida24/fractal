#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <Eigen/Dense>
#include <math.h>

using namespace std; 
using namespace Eigen; 

int main(int argc, char** argv)
{
	cout << "Practice for fractal analysis." << endl; 

	/* 
	MATLAB example from boxcounting code 
	c = randcantor(0.8, 512, 2); 
	C is a 512x512 binary array (similar to the occupancy grid)
	The fractal dimension of C should be 1.66
	*/

	MatrixXd C = MatrixXd::Zero(512, 512); 

	string filename = "../datasets/fractal_dataset.txt"; 

	ifstream myfile;
    myfile.open(filename.c_str());

    float number; 
    int row_counter = 0; 
    int col_counter = 0; 
    int i_max = int(512*512); 

    for(int i = 0; i < i_max; i++)
    {
    	if (col_counter == 512)
    	{
    		if (row_counter == 512)
    		{
    			break; 
    		}
    		else
    		{
    			col_counter = 0; 
    			row_counter++; 
    		}
    	}

		myfile >> number;

		C(row_counter, col_counter) = number; 
		col_counter++; 

	}

    myfile.close();

    // Outputs are N and R

    double width = 512.0; 
    double p = log(width)/log(double(2.0)); 

    // Init N: 
    VectorXf N = VectorXf::Zero(int(p)+1); 

    // Easier to implement if sizes are equal and power of 2

    // Core boxcounting code: 
    N(int(p)) = C.sum(); 

    // Init other stuff: 
    double siz; 
    double siz2; 
    // MatrixXf dummy; 
    float running_sum; 

    for (int g = int(p)-1; g > 0; g--)
    {
    	siz = pow(2.0, double(p-g)); 
    	siz2 = round(siz/2.0); 

    	running_sum = 0; 

    	for (int i = 0; i < int(width-siz+1); i = i+int(siz))
    	{
    		for (int j = 0; j < int(width-siz+1); j = j+int(siz))
    		{
    			C(i,j) = (bool(C(i,j)) || bool(C(i+siz2,j)) || bool(C(i,j+siz2) || bool(C(i+siz2,j+siz2)))); 
    			running_sum = running_sum+C(i,j); 
    		}	
    	}
    	
    	N(g) = running_sum; 


    }

    // Flip N
    N = N.colwise().reverse().eval(); 

    VectorXf R = VectorXf::Zero(int(p)+1); 
    R(0) = 1.0; 
    for (int k = 1; k < R.size(); k++)
    {
    	R(k) = pow(2.0, double(k)); 
    }


    printf("Analysis complete.\n"); 

    printf("Saved to resultsGT.txt\n"); 

    FILE* fp; 
    fp = fopen("resultsGT.txt", "w"); 

    for (int k = 0; k < R.size(); k++)
    {
    	fprintf(fp, "%f %f\n", R(k), N(k)); 
    }

    fclose(fp); 



	return 0; 
}