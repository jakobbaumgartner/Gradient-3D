%% OCCUPANCY GRID
% -------------------------------------------------------------------------------------------

% call the create_3d_space function with the desired dimensions
space_resolution = 10; % 10x10x10 cm voxels
space_height = 200; 
space_width = 500;
space_length = 500;

% create grid obstacle
grid = create_3d_grid(space_resolution, space_height, space_width, space_length);

% add square table
grid = add_box(grid, space_resolution, 200, 210, 100, 200, 1, 50);
grid = add_box(grid, space_resolution, 290, 300, 100, 200, 1, 50);
grid = add_box(grid, space_resolution, 200, 300, 100, 110, 1, 50);
grid = add_box(grid, space_resolution, 200, 300, 190, 200, 1, 50);


% add counter 1
grid = add_box(grid, space_resolution, 90, 100, 100, 400, 1, 50);
grid = add_box(grid, space_resolution, 1, 100, 100, 110, 1, 50);


% add counter 2
grid = add_box(grid, space_resolution, 100, 400, 300, 310, 1, 50);
grid = add_box(grid, space_resolution, 390, 400, 300, 400, 1, 50);

% -------------------------------------------------------------------------------------------



%% ONLY VISUALIZATION
% -------------------------------------------------------------------------------------------
min_voxel_display_value = 0.05; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [25, 65]; % angle at which we look at plot
display_grid(grid, min_voxel_display_value)

% display obstacles (with big patches for faster rendering)
voxel([210 110 0]/space_resolution,[70 70 50]/space_resolution,'g',0.5)
voxel([0 110 0]/space_resolution,[80 290 50]/space_resolution,'g',0.5)
voxel([100 310 0]/space_resolution,[280 90 50]/space_resolution,'g',0.5)

%     % add floor

        % Define the x and y ranges
        x = linspace(0, 1, size(grid,1))*size(grid,1);
        y = linspace(0, 1, size(grid,2))*size(grid,2);
        
        % Create a grid of points from x and y
        [X,Y] = meshgrid(x,y);
        
        % Define the function to plot
        Z = zeros(size(grid,1),size(grid,2));
            
        % Plot the surface
        surf(X,Y,Z, 'FaceAlpha',0.5, 'FaceColor','y');


% set fixed axis size
axis([0 size(grid,1) 0 size(grid,2) 0 size(grid,3)])

% set viewing angle
view(view_angle);
% -------------------------------------------------------------------------------------------


%% PATH
% -------------------------------------------------------------------------------------------

% Create three sets of points
points1 = [(130:0.6:160)' (80:1.6:160)' ones(51,1)*80];
points2 = [(180:5.4:310)' (230:1.21:260)' ones(25,1)*80];
points3 = [(315:5:450)' ones(28,1)*260 ones(28,1)*100];

% Combine the three sets of points into a single matrix
points = [points1 ; points2; points3];

% Call the polyfit_xyz_trajectory function to fit a polynomial curve to the points
[X, Y, Z] = polyfit_xyz_trajectory(points);

% Plot the polynomial curve in 3D space
plot3(X/space_resolution, Y/space_resolution, Z/space_resolution, 'r-', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;


%  -------------------------------------------------------------------------------------------

%% OPTIMIZATION
% -------------------------------------------------------------------------------------------

































