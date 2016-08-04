#!/usr/bin/python
# Python script to plot text file

import sys, numpy, math
import matplotlib.pyplot as plt

##########################################################################
# Parse command-line arguments: 
commandArgs = str(sys.argv); 
splitArgs = commandArgs.split(' '); 

if (len(splitArgs) != 3):
	print("Please provide the output filename and the source filename.\n"); 
	print('python plot_fractal.py output.txt source .png\n'); 
	print('or\n'); 
	print("python plot_fractal.py output.txt source.pcd\n");

	sys.exit(); 

filename = splitArgs[1];  
filename = filename.replace("'", ""); 
filename = filename[0:len(filename)-1]; 
pcdFilename = splitArgs[2];  
pcdFilename = pcdFilename.replace("'", ""); 
pcdFilename = pcdFilename[0:len(pcdFilename)-1]; 

# Read file line by line: 
fp = open(filename, 'r');
x = fp.readlines(); 
fp.close(); 

#############################################################################
# Init N and R (variables to plot): 
N = numpy.zeros(len(x)); 
R = numpy.zeros(len(x)); 
logN = numpy.zeros(len(x)); 
logR = numpy.zeros(len(x)); 
slope = numpy.zeros(len(x)); 

for i in range(0, len(x)): 
	# Fill N and R
	currentRow = x[i]; 
	currentRow = currentRow.split(' '); 

	R[i] = float(currentRow[0]); 
	N[i] = float(currentRow[1]);
	if (N[i] == 0): 
		N[i] = float('NaN'); 

	logR[i] = math.log(R[i], 10); 
	logN[i] = math.log(N[i], 10);  

for i in range(1, len(x)-1): 
	slope[i] = -float(logN[i+1]-logN[i])/float(logR[i+1]-logR[i]); 

slope[0] = float('NaN'); 

print('Close both plot figures to exit the program.'); 

##############################################################################
# Make plots (log-log and semilogx plot with slope: 

plt.figure(1); 
plt.loglog(R, N, basex=10)
plt.grid(True);
plt.title('Log-log plot from Analysis of %s' % pcdFilename); 
plt.xlabel('log(R)'); 
plt.ylabel('log(N)');

plt.figure(2); 
plt.semilogx(R, slope, basex=10); 
plt.title('Slope plot for %s'% pcdFilename); 
plt.xlabel('log(R)');
plt.ylabel('Slope');
plt.grid(True); 

plt.show(); 