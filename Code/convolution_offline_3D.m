function [grid_distance] = convolution_offline_3D(grid_obstacles,kernel_dimenion,sigma)

    % Function Name: convolution_offline_3D
    %
    % Description:
    % This function performs 3D convolution on a given input grid with a 
    % pre-defined Gaussian kernel. It calculates the distance of each point in 
    % the input grid from the obstacles present in the grid. The output of the 
    % function is the grid of density depending depending on nearby obstacles
    % obtained after performing convolution.
    %
    % Inputs:
    % grid_obstacles: A 3D matrix representing the input grid with obstacles. 
    % The matrix contains values between 0's and 1's, where 0's represent free 
    % space and 1's represent obstacles.
    %
    % Outputs:
    % grid_distance: A 3D matrix representing the grid of density depending on 
    % nearby obstacles calculated by performing 3D convolution on the input grid. 
    % The matrix contains distance values for each point in the input grid. 
    % The density values represent the distance of each point from the obstacles 
    % present in the input grid.


    kernel = gaussian_kernel_3d(kernel_dimenion, kernel_dimenion, kernel_dimenion, sigma);
    grid_distance = convn(grid_obstacles, kernel, 'same');

end