close all
%% OCCUPANCY GRID
[grid_occupancy, space_resolution] = map_lab1();

%% DENSITY GRID
kernel_size = 25; % how far should obstacles exert effect
sigma = 5; % how quickly should effect exerted by obstacles fall
[grid_distance] = convolution_offline_3D(grid_occupancy,kernel_size,sigma)*0.5;

%% ONLY VISUALIZATION

min_voxel_display_value = 0.001; % min value to draw voxel, to avoid rendering of empty voxels
view_angle = [45, 65]; % angle at which we look at plot
% display_grid(grid_occupancy, min_voxel_display_value, space_resolution)
% display_grid(grid_distance, min_voxel_display_value, space_resolution)

hold on

% display obstacles (with big patches for faster rendering)
voxel([210 110 0],[70 70 50],'g',0.5)
voxel([0 0 0],[80 290 50],'g',0.5)
voxel([100 310 0],[280 90 50],'g',0.5)

% set fixed axis size
axis([0 size(grid_occupancy,1)*space_resolution 0 size(grid_occupancy,2)*space_resolution 0 size(grid_occupancy,3)*space_resolution])


%% PATH
% add EE path
[points X Y Z] = path_lab_1()

%% PATH OPTIMIZATION
% -------------------------------------------------------------------------------------------

% Define initial robot states
robot_states = [1.50 0 pi/2 0 pi/4 0 -pi/3 0 1.8675 0];

% Create a history of robot states
robot_states_hist = [robot_states];

% Create a history of robot end effector positions
robot_ee_positions = [];

% Create a history of robot the differences between goal and robot position
diff_hist = [];
diff_hist_norm = [];

% Create a history of secondary task velocities
q_avoid_hist = [];


for i=1:1:length(X)

   % Select goal point
   goal = [X(i) Y(i) Z(i) 0 0 0];

   % Calculate the transformation matrix for the robot's current position
   [T] = GeometricRobot(robot_states);

   % Save EE positions
   robot_ee_positions = [robot_ee_positions ; T(1:3,4)'];

   % Calculate the difference between the goal and the robot's current position
   diff = [goal - [T(1:3,4)' 0 0 0]];
   
   % Use an optimizer to determine the optimal velocities for the robot
   [q_vel, q_avoid] = optimizer(robot_states, diff, grid_distance, space_resolution);

   % Simulate a step for the robot using the optimal velocities
   [robot_states] = simulate_step(robot_states, q_vel);

   % Add the current robot state to the history list
   robot_states_hist = [robot_states_hist; robot_states];

   % Add the current difference to the difference history list
   diff_hist = [diff_hist; diff];
   diff_hist_norm = [diff_hist_norm ; norm(diff)];

   % Add the current secondary task velocity to the avoidance history list
   q_avoid_hist = [q_avoid_hist ; q_avoid'];

   % Every few points draw robot
   if ~mod(i,5)
        [T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states);
        showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,space_resolution*10, "blue", true)


        % calculate partial derivatives
        [dx, dy, dz] = interpolate_derivative([robot_states(1), robot_states(2), 0.2], grid_distance, space_resolution);

        % plot the arrow
        arrow_length = 30;
        quiver3(robot_states(1)*100, robot_states(2)*100, 20, -dx * arrow_length, -dy * arrow_length, -dz * arrow_length, arrow_length/2, 'LineWidth', 2, 'MaxHeadSize', 1);

        drawnow;
   end

end

plot3(robot_ee_positions(:,1)*100, robot_ee_positions(:,2)*100, robot_ee_positions(:,3)*100,'LineWidth',2,'Color','b')
% -------------------------------------------------------------------------------------------

%% PLOT DATA

% -------------------------------------------------------------------------------------------


hold off

figure('Name', 'Avoidance angular velocity [w]')

subplot(2,1,1)
plot(q_avoid_hist(:,2))
xlabel('step')
ylabel('rad')
title('Avoidance Angular Velocity')

subplot(2,1,2)
plot(diff_hist_norm)
xlabel('step')
ylabel('m')
title('Euclidian Norm of End Effector Error')





