clear
close all

% PREPARE MAP & ROBOT
% ------------------------------------------------------------

% select map
map_selection = 'four_pillars'

[grid] = MAPS(map_selection);

if (matches(map_selection, 'wall'))

    % Generate goal points
    % trajectory generation points (xyz position, xyz angles)
    control_points = [1.232 1.027 0.971 0 0 0;
                      1.4 1 0.5 0 0 0]; 

    % goal
    goal = control_points(2,1:3);
    
    % Robot Base position
    % robot base transformation (move to x - 1m, y - 1m)
    Tbase = eye(4);
    Tbase(1:2,4) = [1 1]';
    
    % set joints starting pose
    q = [1.74582837386633
    0.164551561035897
    -2.28912954318266
    -0.400811659236395
    0.497594642234695
    0.654743179774447
    -0.0801486263893128];

elseif (matches(map_selection, 'corridor'))     

   % Generate goal points
    % trajectory generation points (xyz position, xyz angles)
    control_points = [1.232 1.027 0.971 0 0 0;
                      1.4 1 0.5 0 0 0]; 

    % goal
    goal = control_points(2,1:3);
    
    % Robot Base position
    % robot base transformation (move to x - 1m, y - 1m)
    Tbase = eye(4); 
    Tbase(1:2,4) = [1 1]';
    
    % set joints starting pose
    q = [1.74582837386633
    0.164551561035897
    -2.28912954318266
    -0.400811659236395
    0.497594642234695
    0.654743179774447
    -0.0801486263893128];

elseif (matches(map_selection, 'none'))

   % Generate goal points
    % trajectory generation points (xyz position, xyz angles)
    control_points = [1.232 1.027 0.971 0 0 0;
                      1.4 1 0.5 0 0 0]; 

    % goal
    goal = control_points(2,1:3);
    
    % Robot Base position
    % robot base transformation (move to x - 1m, y - 1m)
    Tbase = eye(4);
    Tbase(1:2,4) = [1 1]';
    
    % set joints starting pose
    q = [1.74582837386633
    0.164551561035897
    -2.28912954318266
    -0.400811659236395
    0.497594642234695
    0.654743179774447
    -0.0801486263893128];

elseif (matches(map_selection, 'four_pillars'))

   % Generate goal points
    % trajectory generation points (xyz position, xyz angles)
    control_points = [1.232 1.027 0.971 0 0 0;
                      1.4 1 0.5 0 0 0]; 

    % goal
    goal = control_points(2,1:3);
    
    % Robot Base position
    % robot base transformation (move to x - 1m, y - 1m)
    Tbase = eye(4);
    Tbase(1:2,4) = [1 1]';
    
    % set joints starting pose
    q = [1.74582837386633
    0.164551561035897
    -2.28912954318266
    -0.400811659236395
    0.497594642234695
    0.654743179774447
    -0.0801486263893128];



end



%% RUN MOTION PLANNING - JOINT 4 ONLY POI
% ------------------------------------------------------------

tic()
[output] = Full_RTConvolution(grid, goal, Tbase, q);
toc()

%% RUN MOTION PLANNING - MULTIPLE POI (POINTS OF INTEREST)

tic()
[output] = Full_RTConvolution_Multiple_Points(grid, goal, Tbase, q);
toc()

%% SHOW RESULTS
% ------------------------------------------------------------

% PLOT DISTANCE AND MANIPULABILITY
% ------------------------- 

figure()
plot(output.goal_distances, 'g')
hold on
plot(output.manipulability_avoidance*10, 'r')
plot(output.manipulability_primary*10, 'b')

legend('Goal Distances', 'Manipulability Avoidance x10', 'Manipulability Primary x10')
xlabel('Iteration')
ylabel('Value')
title('Plot of Goal Distances and Manipulabilities')
grid on


% PLOT REPULSIVE VALUES
% ------------------------- 

% Create a new figure
figure;

% Plot the grad values
plot(output.repulsive_field(1,:), 'r');
hold on
plot(output.repulsive_field(2,:), 'g');
plot(output.repulsive_field(3,:), 'b');

legend('X', 'Y', 'Z')



% Add X and Y labels
xlabel('Iteration');
ylabel('Repulsive field');

% Add a title
title('Repulsive field');

% Display grid lines on the plot
grid on;


% PLOT VISUALIZATION
% ------------------------- 

showMovementPanda(grid, [], control_points, output.joints_positions, output.EE_positions, output.values_APF, Tbase)
