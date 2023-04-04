%% OCCUPANCY GRID
% -------------------------------------------------------------------------------------------

% call the create_3d_space function with the desired dimensions
space_resolution = 10; % 10x10x10 cm voxels
space_height = 200; 
space_width = 500;
space_length = 500;

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

%% ONLY VISUALIZATION
% -------------------------------------------------------------------------------------------
min_voxel_display_value = 0.05; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [25, 65]; % angle at which we look at plot
display_grid(grid_occupancy, min_voxel_display_value, space_resolution)

hold on

% display obstacles (with big patches for faster rendering)
voxel([210 110 0],[70 70 50],'g',0.5)
voxel([0 110 0],[80 290 50],'g',0.5)
voxel([100 310 0],[280 90 50],'g',0.5)

% %     % add floor
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

% set viewing angle
view(view_angle);
% -------------------------------------------------------------------------------------------


%% PATH
% -------------------------------------------------------------------------------------------

% Create three sets of points
points1 = [(130:0.6:160)' (80:1.6:160)' ones(51,1)*100];
points2 = [(180:5.4:310)' (230:1.21:260)' ones(25,1)*110];
points3 = [(315:5:450)' ones(28,1)*260 ones(28,1)*120];

% Combine the three sets of points into a single matrix
points = [points1 ; points2; points3];

% Call the polyfit_xyz_trajectory function to fit a polynomial curve to the points
[X, Y, Z] = polyfit_xyz_trajectory(points);

% Plot the polynomial curve in 3D space
plot3(X, Y, Z, 'r-', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;


%  -------------------------------------------------------------------------------------------


%% PATH OPTIMIZATION
% -------------------------------------------------------------------------------------------

% Define maksimal distance from goal at which optimization stops
max_dist = 0.01;

% Define goal position for the robot to reach
goal = [130 73 100 0 0 0]/100;

% Define initial robot states
robot_states = [1.50 0 pi/2 0 pi/4 0 -pi/3 0 1.8675 0];

% Create a history of robot states
robot_states_hist = [robot_states];

% Create a history of robot end effector positions
robot_ee_positions = [];

% Create a history of robot the differences between goal and robot position
diff_hist = [];
norm_diff_hist = [];


for i=1:1:length(X)

   % Calculate the transformation matrix for the robot's current position
   [T] = GeometricRobot(robot_states);

   % Save EE positions
   robot_ee_positions = [robot_ee_positions ; T(1:3,4)'];

   % Calculate the difference between the goal and the robot's current position
   diff = [goal - [T(1:3,4)' 0 0 0]];

   % Use an optimizer to determine the optimal velocities for the robot
   [q_vel] = optimizer(robot_states, diff, []);

   % Simulate a step for the robot using the optimal velocities
   [robot_states] = simulate_step(robot_states, q_vel);

   % Add the current robot state to the history list
   robot_states_hist = [robot_states_hist; robot_states];

   % Add the current difference to the difference history list
   diff_hist = [diff_hist; diff];
   norm_diff_hist = [norm_diff_hist ; norm(diff)];

end






























