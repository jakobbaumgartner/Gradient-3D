close all
%% OCCUPANCY GRID
% call the create_3d_space function with the desired dimensions
space_resolution = 10; % 10x10x10 cm voxels
space_height = 2; 
space_width = 5;
space_length = 5;

% create grid obstacle
grid_occupancy = create_3d_grid(space_resolution, space_height, space_width, space_length);

% % add counter 1
grid_occupancy = add_box(grid_occupancy, space_resolution, 1, 150, 150, 250, 120, 130);
grid_occupancy = add_box(grid_occupancy, space_resolution, 50, 70, 150, 250, 1, 79);
grid_occupancy = add_box(grid_occupancy, space_resolution, 150, 170, 150, 250, 1, 79);



%% DENSITY GROUND GRID
kernel_size = 10; % how far should obstacles exert effect
sigma = 3; % how quickly should effect exerted by obstacles fall
[grid_distance] = convolution_offline_3D(grid_occupancy,kernel_size,sigma)*1;

figure()

% create a 50x50 matrix with random ones and zeros to represent occupied and empty cells
matrix = flip(rot90(grid_distance(:,:,2)));
matrix = repelem(matrix, 10, 10);


% plot the matrix with occupied cells in red and empty cells in blue
imagesc(matrix);
colormap(flipud(gray)); % use gray colormap with flipped order

%% ONLY VISUALIZATION

min_voxel_display_value = 0.001; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [45, 65]; % angle at which we look at plot
display_grid(grid_occupancy, min_voxel_display_value, space_resolution)
% display_grid(grid_distance, min_voxel_display_value, space_resolution)

hold on

% % display obstacles (with big patches for faster rendering)
% voxel([210 110 0],[70 70 50],'g',0.5)
% voxel([0 0 0],[80 290 50],'g',0.5)
% voxel([100 310 0],[280 90 50],'g',0.5)

% set fixed axis size
axis([0 size(grid_occupancy,1)*space_resolution 0 size(grid_occupancy,2)*space_resolution 0 size(grid_occupancy,3)*space_resolution])
