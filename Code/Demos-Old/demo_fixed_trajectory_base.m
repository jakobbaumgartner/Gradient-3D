close all
%% OCCUPANCY GRID
[grid_occupancy, space_resolution] = map_lab1();

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
[X Y Z Xb Yb] = path_lab_1();


%% PATH OPTIMIZATION
% -------------------------------------------------------------------------------------------

% Define initial robot states
robot_states = [1.50 0.2 pi/2 0 pi/4 0 -pi/3 0 1.8675 0];

% Create a history of robot states
robot_states_hist = [robot_states];

% Create a history of robot end effector positions
robot_ee_positions = [];

% Create a history of robot the differences between goal and robot position
diff_hist = [];
diff_hist_norm = [];

% Create a history of secondary task velocities
q_avoid_hist = [];

[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states);
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "blue", true)

for i=1:1:length(X)

   % Select goal point
   goal = [X(i) Y(i) Z(i) 0 0 0];

   % Select base goal point
   goal_base = [Xb(i) Yb(i)];

   % Set fixed base position
   robot_states(1) = Xb(i);
   robot_states(2) = Yb(i);  

   % Calculate the transformation matrix for the robot's current position
   [T] = GeometricRobot(robot_states);

   % Save EE positions
   robot_ee_positions = [robot_ee_positions ; T(1:3,4)'];

   % Calculate the difference between the goal and the robot's current position
   diff = [goal - [T(1:3,4)' 0 0 0]];
   
   % Use an optimizer to determine the optimal velocities for the robot
   J = jacobianAnalitic(robot_states); % Calculate the Jacobian matrix analitically
   J(:,1:2) = zeros(6,2);

   damping_factor = 0.01;
    
   pinv_J = J'*(J*J'+ damping_factor^2 * eye(6))^-1;
    
   q_vel = pinv_J * diff' * 10; % Calculate the joint velocities using the pseudo-inverse of the Jacobian and the end-effector velocity

   % Simulate a step for the robot using the optimal velocities
   [robot_states] = simulate_step(robot_states, q_vel);

   % Add the current robot state to the history list
   robot_states_hist = [robot_states_hist; robot_states];

   % Add the current difference to the difference history list
   diff_hist = [diff_hist; diff];
   diff_hist_norm = [diff_hist_norm ; norm(diff)];

   % Add the current secondary task velocity to the avoidance history list
   q_avoid_hist = [q_avoid_hist ; q_vel'];

   % Every few points draw robot
   if ~mod(i,10)
        [T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states);
        showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "yellow", true)
        drawnow;

        % plot point
%         scatter3(goal_base(1)*100, goal_base(2)*100, 10, 'SizeData', 100, 'MarkerFaceColor', 'blue')
%         drawnow;
   end

end

plot3(robot_ee_positions(:,1)*100, robot_ee_positions(:,2)*100, robot_ee_positions(:,3)*100,'LineWidth',2,'Color','magenta')
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





