clear
close all

% PREPARE MAP & ROBOT
% ------------------------------------------------------------

    grid = OctoGrid(2,2,2,10);

   % Generate goal points
    % trajectory generation points (xyz position, xyz angles)
    control_points = [1.5 1.05 0.25 0 0 0]; 

    % goal
    goal = control_points;
    
    % Robot Base position
    % robot base transformation (move to x - 1m, y - 1m)
    Tbase = eye(4);
    Tbase(1:2,4) = [1 1]';
    
    % set joints starting pose
    q = [0.0319913344656175
0.164451369212247
0.00933902434572278
-2.38024192403430
0.00518018325880462
2.53266747724704
0.0400646905027541];
    



%% RUN MOTION PLANNING - JOINT 4 ONLY POI
% ------------------------------------------------------------

% tic()
% [output] = Full_RTConvolution(grid, goal, Tbase, q);
% toc()

%% RUN MOTION PLANNING - MULTIPLE POI (POINTS OF INTEREST)

% tic()
% [output] = Full_RTConvolution_Multiple_Points(grid, goal, Tbase, q);
% toc()

%%  RUN MOTION PLANNING - BALL AVOIDANCE

[output] = Full_RTConvolution_Moving_Ball(grid, goal, Tbase, q);

%% RUN MOTION PLANNING - AVOIDANCE FIRST PRIORITY
% 
% tic()
% [output] = Full_RTConvolution_AvoidanceFirst(grid, goal, Tbase, q);
% toc()

%% PLOT DISTANCE AND MANIPULABILITY
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

% PLOT ORIENTATION ERROR
figure()
plot(output.goal_orientation)


%% PLOT VISUALIZATION
% ------------------------- 

showMovementPandaMultiplePoints(grid, Tbase, control_points, output)