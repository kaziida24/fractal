function plot_fractal(filename)

if (nargin ~= 1)
	fprintf('Please specify the filename of the text file to be plotted.\n'); 
	return; 
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
title('Log(N) vs. Log(R)');
xlabel('Log(R)');
ylabel('Log(N)');

figure(); 
semilogx(data(:,1), slope, 'LineWidth', 4); 
grid on; 
title('Slope ($-\frac{\partial log(N)}{\partial log(R)}$) vs. log(R)', ... 
    'interpreter', 'Latex'); 

end
