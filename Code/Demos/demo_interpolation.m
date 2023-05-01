%% OCCUPANCY GRID
% -------------------------------------------------------------------------------------------

% call the create_3d_space function with the desired dimensions
space_resolution = 10; % 10x10x10 cm voxels
space_height = 2; 
space_width = 5;
space_length = 5;

% create grid obstacle
grid_occupancy = create_3d_grid(space_resolution, space_height, space_width, space_length);

% add square table
grid_occupancy = add_box(grid_occupancy, space_resolution, 200, 210, 100, 200, 1, 50);
grid_occupancy = add_box(grid_occupancy, space_resolution, 290, 300, 100, 200, 1, 50);
grid_occupancy = add_box(grid_occupancy, space_resolution, 200, 300, 100, 110, 1, 50);
grid_occupancy = add_box(grid_occupancy, space_resolution, 200, 300, 190, 200, 1, 50);


% add counter 1
grid_occupancy = add_box(grid_occupancy, space_resolution, 90, 100, 100, 400, 1, 50);
grid_occupancy = add_box(grid_occupancy, space_resolution, 1, 100, 100, 110, 1, 50);


% add counter 2
grid_occupancy = add_box(grid_occupancy, space_resolution, 100, 400, 300, 310, 1, 50);
grid_occupancy = add_box(grid_occupancy, space_resolution, 390, 400, 300, 400, 1, 50);

% -------------------------------------------------------------------------------------------

%% DENSITY GRID
[grid_distance] = convolution_offline_3D(grid_occupancy,25,1)*10;

%% ONLY VISUALIZATION
% -------------------------------------------------------------------------------------------
min_voxel_display_value = 0.001; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [45, 65]; % angle at which we look at plot
display_grid(grid_occupancy, min_voxel_display_value, space_resolution)

hold on

% display obstacles (with big patches for faster rendering)
voxel([210 110 0],[70 70 50],'g',0.5)
voxel([0 110 0],[80 290 50],'g',0.5)
voxel([100 310 0],[280 90 50],'g',0.5)

%     % add floor

        % Define the x and y ranges
        x = linspace(0, 1, size(grid_occupancy,1)) * size(grid_occupancy,1) * space_resolution;
        y = linspace(0, 1, size(grid_occupancy,2)) * size(grid_occupancy,2) * space_resolution;
        
        % Create a grid of points from x and y
        [X,Y] = meshgrid(x,y);
        
        % Define the function to plot
        Z = zeros(size(grid_occupancy,1),size(grid_occupancy,2));
            
        % Plot the surface 
        surf(X,Y,Z, 'FaceAlpha',0.5, 'FaceColor',"#EDB120");


% set fixed axis size
axis([0 size(grid_occupancy,1)*space_resolution 0 size(grid_occupancy,2)*space_resolution 0 size(grid_occupancy,3)*space_resolution])

% set viewing angle
% view(view_angle);
% -------------------------------------------------------------------------------------------






















