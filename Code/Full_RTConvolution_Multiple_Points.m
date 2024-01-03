function [output] = Full_RTConvolution_Multiple_Points(grid, goal_point, Tbase, q)

%% PARAMETERS

% select which goals
mid_joints = 0
avoid_task = 1
kinematics_solution = 'exact-reduced' % OPTIONS: exact-reduced , exact , approximate
timestep_secondary_gain_change = 0 % if selected, secondary task will start with normal gain and fall with time
secondary_exec_stop_k = 1 % primary task will slow down (>0) or stop executing if secondary task has big velocities 
min_exec_slowdown_size = 0 % if poi is closer than this value, primary task will slow down

% -----------------------------------------------------------
% number of points per segment for obstacle avoidance taskmanipulability_avoidance
points_per_segment = 1*[1 1 2 1 3 1 1];

% the number of points taken into account and weighting factors
weights_avoidance = [1 1 1 1 1 1];
weights_avoidance = weights_avoidance / norm(weights_avoidance,1) / 10;

% -----------------------------------------------------------

Tstep = 0.1 % time step
Nmax = 500 % max number of iterations
space_resolution = grid.resolution; % resolution of the obstacles grid

% weights for different tasks
wp = 0.75 % primary task
wp_att = 1 % primary task - attractive component
wp_rep = 0 % primary task - repulsive component
wm = 0.15 % mid-joints task
wa = 1 % obstacle avoidance task



% function parameters
goal_dist = 0.001 % distance which satisfies ending of optimization

% damping factor to avoid inverse Jacobain matrix singularities
damping_factor_primary = 0.01
damping_factor_avoidance = 0.01

% joints range
q_range = [2.8973 -2.8973;
           1.7628 -1.7628;
           2.8973 -2.8973;
           -0.0698 -3.0718;
           2.8973 -2.8973;
           3.7525 -0.0175;
           2.8973 -2.8973];

%% START ROBOT SETTINGS / VALUES
% -----------------------------------------------------------

% get a list of transforms for every APF detection point on the robot
transformations_list = getPartialTransformationsListPanda('points_per_segment', points_per_segment);

% get current robot pose
[robot_transforms] = GeometricPandaMATLAB(q, Tbase);

% set current ee point to start point
ee_point = robot_transforms(1:3,4,8);

% calculate distance from goal
current_dist = norm(ee_point'- goal_point(1:3));

%% PREPARE LOG ARRAYS
% -----------------------------------------------------------

output = struct;
output.joints_positions = [q];
output.EE_positions = [ee_point];
output.goal_distances = [current_dist];
output.q_velocities = [];
output.ee_velocities = [];
output.values_APF = [];
output.manipulability_primary = [];
output.manipulability_avoidance = [];
output.repulsive_field = [];
output.POI_locations = [];
output.POI_values = [];

manipulability_primary = 0;
manipulability_avoidance = [];


%% GET KERNELS
% -----------------------------------------------------------

% repulsive kernels
rep_kernels = REP_kernels('linear');

% obstacles distance kernel
% dist_kernel = euclidian_kernel_3D(61, 61, 61);
% dist_kernel = gaussian_kernel_3d(61, 61, 61, 10);



%% OPTIMIZATION LOOP
% -----------------------------------------------------------

% CREATE PROGRESS BAR
f = waitbar(0, 'Running kinematic optimization')

% set itarations count
Niter = 1;

% trajectory calculation loop
while current_dist > goal_dist && Niter <= Nmax  

    %% UPDATE PROGRESS BAR
    waitbar(Niter/Nmax)

    %% CALCULATE PRIMARY TASK 
    % --------------------------------------------------

    % OPTION JACOBIAN
    [~,~,J]=kinmodel_panda(q); % calculate jacobian

    % calculate pseudo inverse
    pinv_J = J'*(J*J'+ damping_factor_primary * eye(6))^-1; % damping to avoid singularities

    % --------------------------------------------------

    % ATTRACTIVE ( OPTION KINEMATICS CLASSIC END EFFECTOR )
    ee_vel_att_magn = norm(goal_point(1:3)' - ee_point);
    ee_vel_att = (goal_point(1:3)' - ee_point)/ee_vel_att_magn * atan(100*ee_vel_att_magn)/pi*2; % direction only - normalised - sigmoid

    % REPULSIVE 
    ee_vel_rep = REP_field_calculation(grid, rep_kernels, ee_point);
    
    % TOTAL : SUM   
    ee_vel = (wp_att * ee_vel_att + wp_rep * avoid_task * ee_vel_rep');
    ee_vel = wp .* [ee_vel ; 0 ; 0 ; 0];
    

    
    %% CALCULATE SECONDARY TASKS (AVOIDANCE + MID-JOINTS)
    % --------------------------------------------------
    
    % calculate Null Space 
    N = (eye(7)-pinv_J*J);

    %%  MANIPULABILITY MEASUREMENTS
    % -----------------------

    % primary
    manipulability_primary = sqrt(det(J*J'));


    %% MID JOINTS
    % --------------------------------------------------

    dq_mid = zeros(7,1);

    if mid_joints 

        % option 1 - joints speeds proportional joints difference from mid
        for i = 1:1:7
            dq_mid(i) = wm * (sum(q_range(i,:))/2) - q(i);
        end

    end

    %% APF AVOIDANCE TASK
    % --------------------------------------------------
    rep_values = [];

    q_avoid_total = zeros(7,1);

    % location of different POI on robot [x1 x2 ...
    %                                     y1 y2 ...
    %                                     z1 z2 ... ]
    poi_locations = [];

    % values in POI
    obstacles_field_values = [];

    % if timestep_secondary_gain_change is selected, secondary task will start with normal gain and fall with time
    if timestep_secondary_gain_change
        wa = wa * (1 - 0.1 * Niter/Nmax);
    end


    if avoid_task
        
        % for every poi
        for index_poi = 1:1:sum(points_per_segment)

            % GET POI POSITIONS
            % --------------------
        
            % get transformation for selected poi
            robot_tree = transformations_list(index_poi).tree;
        
            % set configuration to a robot tree
            config = homeConfiguration(robot_tree); % Create an empty struct
            
            % Assign values to joints, there will be 1-7 joints for Panda
            for i = 1:1:robot_tree.NumBodies
                config(i).JointPosition  = q(i); % set i-th joint
            end
        
            % get transform of selected poi
            transform = getTransform(robot_tree,config,'body'+string(robot_tree.NumBodies));
        
            % add base transformation
            transform = Tbase * transform;
        
            % get x,y,z values of joint 4 position
            poi_locations = [poi_locations transform(1:3,4)];

            % GET POI FIELD VECTORS
            % --------------------
            rep_values = [rep_values REP_field_calculation(grid, rep_kernels, transform(1:3,4))'];


        end

        % SORT POI BASED ON DISTANCE FROM OBSTACLES
        % --------------------
        [poi_sizes, poi_indeces] = sort(vecnorm(rep_values));            


        %% FOR EVERY POI CALCULATE AVOIDANCE JOINT SPEEDS (USING INVERSE KINEMATICS SOLUTION)
        % --------------------------------------------------

         q_avoid_list = []; % joint avoidance velocities for n-POI

         for selected_poi = length(poi_indeces):-1:(length(poi_indeces)-length(weights_avoidance)+1) % biggest n-POI

            % poi index
            poi_index = poi_indeces(selected_poi);

            % avoidance vel
            rep_vel = rep_values(:,poi_index);

            % avoidance magnitude
            rep_magnitude = poi_sizes(selected_poi);        
    
            % avoidance direction - unit vector
            if(rep_magnitude > 10^-12)
                rep_direction = rep_vel' ./ rep_magnitude;
            else
                rep_direction = [0 0 0];
            end
    
            % scalling
            avoid_vel = wa * rep_vel;

    
            % CALCULATE JACOBIAN IN POINT0
            % --------------------
    
            % calculate Jacobian in that point
            robot = transformations_list(poi_index).tree;
            config = homeConfiguration(robot); % set configuration
    
            % set joint positions
            for i = 1:1:robot.NumBodies
                config(i).JointPosition = q(i);
            end
    
            % toolbox jacobian returns first three rows angular velocity and
            % second three rows linear velocities
            J2 = geometricJacobian(robot,config,'body'+string(robot.NumBodies));
    
            % only positional jacobian part
            J0 = zeros(3,7);
            J0(1:3,1:size(J2,2)) = J2(4:6,:);

                
            if(matches(kinematics_solution, 'exact'))
    
                % EXACT SOLUTION
                % -----------------------
    
                % calculate pseudo inverse
                pinv_J0 = (J0*N)'*inv((J0*N)*(J0*N)' + damping_factor_avoidance * eye(3)); %damping to avoid singularities
    
                % calculate avoidance joints velocities
                q_avoid_list = [ q_avoid_list pinv_J0 * (avoid_vel - J0*pinv_J * ee_vel - J0*N*dq_mid)];
    
            elseif(matches(kinematics_solution, 'exact-reduced'))
    
                % EXACT SOLUTION - REDUCED AVOIDANCE OPERATIONAL SPACE
                % -----------------------
                % REF: Obstacle Avoidance for Redundant Manipulators as Control Problem 
                %      (page 209) Petric, Tadej ; Zlajpah, Leon
    
                % Jacobian that relates velocity in avoidance direction and joint velocities
                Jd0 = (rep_direction * J0);    
    
                % calculate pseudo inverse
                pinv_Jd0 = N*Jd0'*(Jd0*N*Jd0' + damping_factor_avoidance)^-1;
    
                % calculate avoidance joints velocities
                q_avoid_list = [ q_avoid_list pinv_Jd0 * ( wa * rep_magnitude - Jd0*pinv_J * ee_vel - Jd0*N*dq_mid)];
    
            elseif(matches(kinematics_solution, 'approximate'))
    
                % APPROXIMATE SOLUTION
                % -----------------------
    
                % calculate pseudo inverse
                pinv_J0 = J0'*(J0*J0'+ damping_factor_avoidance * eye(3))^-1;        
    
                % calculate avoidance joints velocities
                q_avoid_list = [ q_avoid_list , N * pinv_J0*(avoid_vel)];

            end

         end

        % COMBINE JOINT AVOIDANCE VELOCITIES
        % -----------------------
        q_avoid_total = q_avoid_list * weights_avoidance';

    end

    %% COMBINE TASKS
    % --------------------------------------------------
    
    % PRIMARY: position

    % exec slowdown - if secondary task has big velocities, primary task will slow down to give more time to collisions avoidance

    if poi_sizes(end) > min_exec_slowdown_size % if poi is too close
        exec_slowdown = 1 / (1 + secondary_exec_stop_k * poi_sizes(end));
    else
        exec_slowdown = 1; % or set to the default value if no slowdown is needed
    end
    
    q_vel_position = pinv_J * exec_slowdown * ee_vel;

    % SECONDARY: mid-joints
    q_vel_mid = N * dq_mid;
        
    % SECONDARY: avoidance
    q_vel_avoid = q_avoid_total;

    % TOTAL:
    % -----------------------
    q_vel = q_vel_position + q_vel_mid + q_vel_avoid;
    

    %% ONE STEP SIMULATION 
    % --------------------------------------------------

    % calculate new joint positions
    q = q + q_vel * Tstep;

    % limit positions within joint limits
    [q, ~] = checkPositionLimits(q);

    % calculate new EE position
    [robot_transforms] = GeometricPandaMATLAB(q, Tbase);
    ee_point = robot_transforms(1:3,4,8);

    % update goal distance
    current_dist = norm(ee_point'- goal_point(1:3))

    %% LOGS
    % -----------------------------------------------------------

    % save joint positions
    output.joints_positions = [output.joints_positions q];

    % save EE positions
    output.EE_positions = [output.EE_positions ee_point];

    % save goal distance
    output.goal_distances = [output.goal_distances current_dist];

    % save poi positions
    output.POI_locations{Niter} = poi_locations;

    % save poi values
    output.POI_values{Niter} = rep_values;

    % save joint velocities
    output.q_velocities = [output.q_velocities q_vel];

    % save manipulability measurements of jacobians
    output.manipulability_primary = [output.manipulability_primary manipulability_primary];
    output.manipulability_avoidance = [output.manipulability_avoidance manipulability_avoidance];


    % ITER COUNTER
    % -----------------------------------------------------------
    % raise iterations count
    Niter = Niter + 1;

end

%% REMOVE PROGRESS BAR
close(f)


end