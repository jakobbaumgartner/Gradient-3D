% call the create_3d_space function with the desired dimensions
space_resolution = 10; % 10x10x10 cm voxels
space_height = 2; 
space_width = 5;
space_length = 5;

% create grid obstacle
grid = create_3d_grid(space_resolution, space_height, space_width, space_length);

% add square table
grid = add_box(grid, space_resolution, 200, 300, 100, 200, 1, 50);

% add counter 1
grid = add_box(grid, space_resolution, 1, 100, 100, 400, 1, 50);

% add counter 2
grid = add_box(grid, space_resolution, 100, 400, 300, 400, 1, 50);

min_voxel_display_value = 0.05; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [25, 65]; % angle at which we look at plot
display_grid(grid, min_voxel_display_value, view_angle)