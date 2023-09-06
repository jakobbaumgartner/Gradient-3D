clear
close all

% PREPARE MAP & ROBOT
% ------------------------------------------------------------

% select map
map_selection = 'wall'

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


end




%% RUN MOTION PLANNING
% ------------------------------------------------------------

tic()
[joints_positions, EE_positions, goal_distances, q_velocities, ee_velocities, values_secondary] = Full_RTConvolution(grid, goal, Tbase, q,'avoid_task', true, 'mid_joints', false);
toc()

%% SHOW RESULTS
% ------------------------------------------------------------
figure()
plot(goal_distances)

showMovementPanda(grid, [], control_points, joints_positions, EE_positions, values_secondary, Tbase)

