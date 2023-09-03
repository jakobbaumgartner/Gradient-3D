% REPULSIVE FIELD GENERATION
close all
%% SET POINT
point = [1,1,1]

%% CREATE GRID
% -----------------------------------------------------------

% create grid
grid = OctoGrid(2,2,2,100);

% add the wall
grid.addBox(0.2,0.8,0,1.6,0.1,1.5)

%% GET REPULSIVE KERNEL
% -----------------------------------------------------------

% set directional kernels
sigma = 11;
kernel_length = 61;
kernel_width = 9;
kernel_height = kernel_width;

% form kernels
kernel_x = directional_kernel_3d('x', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
kernel_y = directional_kernel_3d('y', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
kernel_z = directional_kernel_3d('z', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;

kernels = {kernel_x, kernel_y, kernel_z};

%% GET REP FIELD VALUES
% -----------------------------------------------------------

[rep_values] = REP_field_calculation(grid, kernels, point)

% generate vectors from values
rep_vectors = eye(3) .* rep_values';

%% DISPLAY REP VECTORS
% -----------------------------------------------------------
REP_field_simulation(grid, point, kernels)
