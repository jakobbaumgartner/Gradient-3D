%% OCCUPANCY GRID
% -------------------------------------------------------------------------------------------

% call the create_3d_space function with the desired dimensions
space_resolution = 10; % 10x10x10 cm voxels
space_height = 2; 
space_width = 5;
space_length = 5;

% create grid obstacle
grid_occupancy = create_3d_grid(space_resolution, space_height, space_width, space_length);

% add counter 2
grid_occupancy = add_box(grid_occupancy, space_resolution, 100, 400, 300, 310, 1, 50);

% -------------------------------------------------------------------------------------------

%% DENSITY GRID
kernel_size = 15; % how far should obstacles exert effect
sigma = 5; % how quickly should effect exerted by obstacles fall
[grid_distance] = convolution_offline_3D(grid_occupancy,kernel_size,2);

%% OBSTACLES VISUALIZATION
% -------------------------------------------------------------------------------------------
min_voxel_display_value = 0.001; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [45, 65]; % angle at which we look at plot
% display_grid(grid_occupancy, min_voxel_display_value, space_resolution) 

hold on

% display obstacles (with big patches for faster rendering)
voxel([100 310 0],[280 90 50],'g',0.5)


%     % add floor
% 
%         % Define the x and y ranges
%         x = linspace(0, 1, size(grid_occupancy,1)) * size(grid_occupancy,1) * space_resolution;
%         y = linspace(0, 1, size(grid_occupancy,2)) * size(grid_occupancy,2) * space_resolution;
%         
%         % Create a grid of points from x and y
%         [X,Y] = meshgrid(x,y);
%         
%         % Define the function to plot
%         Z = zeros(size(grid_occupancy,1),size(grid_occupancy,2));
%             
%         % Plot the surface 
%         surf(X,Y,Z, 'FaceAlpha',0.5, 'FaceColor',"#EDB120");


% set fixed axis size
axis([0 size(grid_occupancy,1)*space_resolution 0 size(grid_occupancy,2)*space_resolution 0 size(grid_occupancy,3)*space_resolution])

% set viewing anglecha
% view(view_angle);
% -------------------------------------------------------------------------------------------

%% GRADIENT FIELD VISUALIZATION
% -------------------------------------------------------------------------------------------

arrow_length = 30; % adjust the length to your preference

% multiple points

% define the x, y, and z coordinates of the grid
x_coords = 0.25:0.25:4.5;
y_coords = 1:0.25:2.9;
z_coords = 0.1:0.25:1.5;

for x = x_coords

    for y = y_coords

        for z = z_coords

            % select point
            start_point = [x y z];

            % calculate partial derivatives
            [dx, dy, dz] = interpolate_derivative(start_point, grid_distance, space_resolution);

            % plot the arrow
            quiver3(start_point(1)*100, start_point(2)*100, start_point(3)*100, -dx * arrow_length, -dy * arrow_length, -dz * arrow_length, arrow_length/2, 'LineWidth', 2, 'MaxHeadSize', 1);


        end
    end



end











