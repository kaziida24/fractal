function visualizeDataset(varargin)

% Need MATLAB R2016a with the computer vision toolbox to use this
% visualization program 

if (nargin ~= 1)
    fprintf('Check usage again.\n'); 
    return; 
else
    if (strcmp(varargin{1,1}, '-h'))
        help_menu(); 
        return;
    else
        dataset_name = varargin{1,1}; 
    end
end

filename = sprintf('../datasets/%s', dataset_name); 
points = dlmread(filename); 

close all force; 
figure();
pcshow(points); 
grid on; 
title(sprintf('Rendering of %s', dataset_name), 'Interpreter', 'none'); 
xlabel('x');
ylabel('y');
zlabel('z');

    function help_menu()
        fprintf('\n'); 
        fprintf('Welcome to the help menu for visualizeDataset.\n');
        fprintf('\n'); 
        fprintf('Usage:\n\n'); 
        fprintf('Call the program with the name of the ASCII file to render.\n'); 
        fprintf('visualizeDataset filename.txt\n\n'); 
        fprintf('See the help menu again.\n'); 
        fprintf('visualizeDataset -h\n\n');
        fprintf('List of ASCII files in the datasets folder\n\n'); 
        dir_info = dir('../datasets'); 
        for i = 1:size(dir_info)
            currentFile = dir_info(i).name; 
            if (length(currentFile) > 4 && strcmp(currentFile(length(currentFile)-3:length(currentFile)), '.txt'))
                fprintf('%s\n', currentFile); 
            end
        end
        fprintf('\n'); 
    end
end