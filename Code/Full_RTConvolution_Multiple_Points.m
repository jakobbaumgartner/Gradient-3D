function [output] = Full_RTConvolution_Multiple_Points(grid, goal_point, Tbase, q)

%% PARAMETERS

% select which goals
mid_joints = 1
avoid_task = 1
kinematics_solution = 'exact-reduced' % OPTIONS: exact-reduced , exact , approximate
task_order = 'position-avoidance' % OPTIONS: position-avoidance, avoidance-position
timestep_primary_gain_change = 1 % if selected, primary task will start with little gain and grow with time

% -----------------------------------------------------------
% number of points per segment for obstacle avoidance taskmanipulability_avoidance
points_per_segment = 1*[2 1 5 2 6 2 1];

% the number of points taken into account and weighting factors
weights_avoidance = [1 1/2 1/4];
weights_avoidance = weights_avoidance / norm(weights_avoidance);

% -----------------------------------------------------------

Tstep = 0.1 % time step
Nmax = 200 % max number of iterations
space_resolution = grid.resolution; % resolution of the obstacles grid

% weights for different tasks
wp = 5 % primary task
wp_att = 5 % primary task - attractive component
wp_rep = 0.1 % primary task - repulsive component
wm = 1 % mid-joints task
wa = 10 % obstacle avoidance task
wa_i = 10 % obstacle avoidance 

% factor that controls sigmoid function (tanh) for primary task
sigm_factor_primary = 1

% factor that controls sigmoid function (tanh) for avoidance task
sigm_factor_avoidance = 1

% function parameters
goal_dist = 0.02 % distance which satisfies ending of optimization

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

if timestep_primary_gain_change
    wp = wp/Nmax % primary task
end

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
rep_kernels = REP_kernels();

% obstacles distance kernel
% dist_kernel = euclidian_kernel_3D(61, 61, 61);
dist_kernel = gaussian_kernel_3d(61, 61, 61, 10);



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
    ee_vel_att = goal_point(1:3)' - ee_point

    % REPULSIVE 
    ee_vel_rep = REP_field_calculation(grid, rep_kernels, ee_point)
    
    % scale ee_rep to not interfere with ee_att
    scaled_ee_vel_rep = norm(ee_vel_att) * ee_vel_rep'

    % TOTAL : SUM   
    ee_vel = (wp_att * ee_vel_att + wp_rep * avoid_task * scaled_ee_vel_rep);

    if timestep_primary_gain_change
        ee_vel = Niter * wp .* tanh([ee_vel ; 0 ; 0 ; 0]/sigm_factor_primary);
    else
        ee_vel = wp .* tanh([ee_vel ; 0 ; 0 ; 0]/sigm_factor_primary);
    end

    
    %% CALCULATE SECONDARY TASKS (AVOIDANCE + MID-JOINTS)
    % --------------------------------------------------
    
    % calculate Null Space 
    N = (eye(7)-pinv_J*J);

    %%  MANIPULABILITY MEASUREMENTS
    % -----------------------

    % primary
    manipulability_primary = sqrt(det(J*J'))


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
            rep_direction = rep_vel' ./ rep_magnitude;
    
            % sigmoid transformation and scalling
            avoid_vel = wa * tanh(wa_i * rep_vel/sigm_factor_avoidance); 

    
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
                q_avoid_list = [ q_avoid_list pinv_Jd0 * (rep_magnitude - Jd0*pinv_J * ee_vel - Jd0*N*dq_mid)];
    
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

    if(matches(task_order, 'position-avoidance'))

        % PRIMARY: position
        q_vel_position = pinv_J * ee_vel;
    
        % SECONDARY: mid-joints
        q_vel_mid = N * dq_mid;
            
        % SECONDARY: avoidance
        q_vel_avoid = q_avoid_total;
    
        % TOTAL:
        % -----------------------
        q_vel = q_vel_position + q_vel_mid + q_vel_avoid;
        

    elseif(matches(task_order, 'avoidance-position'))


    end




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