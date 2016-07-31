function visualizeDataset()

% Need MATLAB R2016a with the computer vision toolbox to use this
% visualization program 

% filename = '../datasets/Wrapped_Orange 1 (copy).txt'; 
filename = '../../datasets/Wrapped_Orange 1 (copy).txt';
points = dlmread(filename); 
dataset_name = parseString(filename); 

close all force; 
figure();
pcshow(points); 
grid on; 
title(sprintf('Rendering of %s\n', dataset_name), 'Interpreter', 'none'); 
xlabel('x');
ylabel('y');
zlabel('z');

    function output_string = parseString(input_string)
       % Parses path string so that it only includes the textfile name
       for i = length(input_string):-1:1
           if (strcmp(input_string(i), '/') == 1)
               % Divide up the string here
               output_string = input_string(i+1:length(input_string)); 
               break; 
           end

       end
    end
end