clc

vec = 1:10;  % A random vector of 100 elements
s = 1;               % Select 5 items

selected_indices = biasedRandomSelection(vec, s,10,1);
disp(selected_indices);  % Display the indices of the selected elements
