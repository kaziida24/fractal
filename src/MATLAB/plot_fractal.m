function plot_fractal(varargin)

if (nargin > 2 || nargin == 0)
	fprintf('Check usage again.\n'); 
	return;
elseif (nargin == 1)
	if (strcmp(varargin{1,1}, '-h'))
		help_msg(); 
	else
		fprintf('Check usage again.\n'); 
	end
	return;
else
	filename = varargin{1,1}; 
	sourcename = varargin{1,2}; 
end
	

data = dlmread(filename); 

% Use finite difference to find slope 
logR = log10(data(:,1)); 
logN = log10(data(:,2)); 

slope = zeros(size(logR)); 

for i = 2:length(slope)-1
    slope(i) = (logN(i+1)-logN(i-1))/(logR(i+1)-logR(i-1)); 
end

slope(1) = nan; 
slope = -slope; 

% Make plots 
close all force; 
figure(); 
loglog(data(:,1), data(:,2), 'LineWidth', 4); 
grid on; 
title(sprintf('Log(N) vs. Log(R) for %s', sourcename), 'Interpreter', 'none');
xlabel('Log(R)');
ylabel('Log(N)');

figure(); 
semilogx(data(:,1), slope, 'LineWidth', 4); 
grid on; 
title(sprintf('Slope vs. Log(R) for %s', sourcename), 'Interpreter', 'none'); 
xlabel('Log(R)');
ylabel('Slope'); 

	function help_msg()
		fprintf('Welcome to the help message for plot_fractal.\n'); 
		fprintf('\n');
		fprintf('Usage:\n\n');
		fprintf('Call the program along with the name of the output file and the name of the input file.\n'); 
		fprintf('\nplot_fractal output.txt input.png');
		fprintf('\nor\n'); 
		fprintf('plot_fractal output.txt input.pcd');  
		fprintf('\n\n');  

	end
end
