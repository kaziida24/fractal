#!/usr/bin/python

import sys

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv); 

args = str(sys.argv); 

for i in range(0,len(args)): 
	print(args[i]); 