function [output] = Full_RTConvolution(grid, goal_point, Tbase, q, varargin)

%% PARAMETERS

% Parse the optional arguments
p = inputParser;
addOptional(p, 'mid_joints', true); % Default value is false
addOptional(p, 'avoid_task', true); % Default value is true
addOptional(p, 'kinematics', 'exact-reduced'); % Default value is true


parse(p, varargin{:});

% select which goals
mid_joints = p.Results.mid_joints
avoid_task = p.Results.avoid_task
kinematics_solution = p.Results.kinematics

% -----------------------------------------------------------

% weights for different tasks
wp = 2 % 2*[5 5 0.5 1 1 1]'; % primary task
wm = 0 % mid-joints task
wa = 10 % obstacle avoidance task

% factor that controls sigmoid function (tanh) for primary task
sigm_factor_primary = 25

% factor that controls sigmoid function (tanh) for avoidance task
sigm_factor_avoidance = 50

% function parameters
goal_dist = 0.01 % distance which satisfies ending of optimization

% damping factor to avoid inverse Jacobain matrix singularities
damping_factor_primary = 0.01
damping_factor_avoidance = 0.5

Tstep = 0.1 % time step
Nmax = 1200 % max number of iterations
space_resolution = grid.resolution; % resolution of the obstacles grid

% joints range
q_range = [2.8973 -2.8973;
           1.7628 -1.7628;
           2.8973 -2.8973;
           -0.0698 -3.0718;
           2.8973 -2.8973;
           3.7525 -0.0175;
           2.8973 -2.8973];

% number of points per segment for obstacle avoidance task
points_per_segment = 1*[1 1 1 1 1 1 1];

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
output.manipulability_secondary = [];

%% GET REPULSIVE KERNEL
% -----------------------------------------------------------
kernels = REP_kernels();

%% OPTIMIZATION LOOP
% -----------------------------------------------------------

% CREATE PROGRESS BAR
f = waitbar(0, 'Running kinematic optimization')

% set itarations count
Niter = 0;

% trajectory calculation loop
while current_dist > goal_dist && Niter < Nmax  

    %% UPDATE PROGRESS BAR
    waitbar(Niter/Nmax)

    %% CALCULATE PRIMARY TASK 
    % --------------------------------------------------

    % OPTION JACOBIAN
    [~,~,J]=kinmodel_panda(q); % calculate jacobian

    % calculate pseudo inverse
    pinv_J = J'*(J*J'+ damping_factor_primary * eye(6))^-1; % damping to avoid singularities

    % --------------------------------------------------
    % OPTION KINEMATICS CLASSIC END EFFECTOR

    % calculate ee velocities
    ee_vel = goal_point(1:3)' - ee_point;
    ee_vel = wp .* tanh([ee_vel ; 0 ; 0 ; 0]/sigm_factor_primary);

    % calculate joint velocities using inverse kinematics
    q_vel = pinv_J * ee_vel;

    % --------------------------------------------------

    
    %% CALCULATE SECONDARY TASKS (AVOIDANCE + MID-JOINTS)
    % --------------------------------------------------
    
    % calculate Null Space 
    N = (eye(7)-pinv_J*J);


    %% MID JOINTS
    % --------------------------------------------------
    if mid_joints 

        dq_sec = zeros(7,1);

        % option 1 - joints speeds proportional joints difference from mid
        for i = 1:1:7
            dq_sec(i) = wm * (sum(q_range(i,:))/2) - q(i);
        end

        % Primary + Null-Space Task
        q_vel = q_vel + N * dq_sec;

    end

    %% APF AVOIDANCE TASK
    % --------------------------------------------------
    avoid_vel = [];

    % GET JOINT 4 POSE
    % --------------------

    % select transformation for joint 4
    robot_tree = transformations_list(4).tree;

    % set configuration to a robot tree
    config = homeConfiguration(robot_tree); % Create an empty struct
    
    % Assign values to joints, there will be 1-7 joints for Panda
    for i = 1:1:robot_tree.NumBodies
        config(i).JointPosition  = q(i); % set i-th joint
    end

    % get transform of joint 4 position
    transform = getTransform(robot_tree,config,'body'+string(robot_tree.NumBodies));

    % add base transformation
    transform = Tbase * transform;

    % get x,y,z values of joint 4 position
    xyz = transform(1:3,4);
        

    if avoid_task

        % GET VELOCITIES USING DIRECTIONAL KERNELS
        % --------------------
        
        [rep_values] = REP_field_calculation(grid, kernels, xyz);

        % convert values to vectors (this only works for 3 kernels, in x y z
        % directions)
        rep_vectors = eye(3) .* rep_values';

        % sum of vector components
        rep_sum = sum(rep_vectors');

        % avoidance magnitude
        rep_magnitude = norm(rep_sum);        

        % avoidance direction - unit vector
        rep_direction = rep_sum ./ rep_magnitude;
 
        % sigmoid transformation and scalling
        avoid_vel = wa * tanh(rep_sum'/sigm_factor_avoidance); 

        
        % CALCULATE JACOBIAN IN POINT0
        % --------------------

        % calculate Jacobian in that point
        robot = transformations_list(4).tree;
        config = homeConfiguration(robot); % set configuration
            
        % set joint positions
        for i = 1:1:robot.NumBodies
            config(i).JointPosition = q(i);
        end
        
        % toolbox jacobian returns first three rows angular velocity and
        % second three rows linear velocities, change rows to be in line with
        % used convention in default jacobian
        J2 = geometricJacobian(robot,config,'body'+string(robot.NumBodies));
%         J0 = zeros(6,7);
%         J0(1:3,1:size(J2,2)) = J2(4:6,:);
%         J0(4:6,1:size(J2,2)) = J2(1:3,:);

        % only positional jacobian part
        J0 = zeros(3,7);
        J0(1:3,1:size(J2,2)) = J2(4:6,:);


       
        %% INVERSE KINEMATICS SOLUTION
        % --------------------------------------------------

        if(matches(kinematics_solution, 'exact'))
         
            % EXACT SOLUTION
            % -----------------------
    
            % calculate pseudo inverse
            pinv_J0 = (J0*N)'*inv((J0*N)*(J0*N)' + damping_factor_avoidance * eye(3)); %damping to avoid singularities
     
            % calculate avoidance joints velocities
            q_vel = q_vel + pinv_J0 * (avoid_vel - J0*pinv_J * ee_vel);

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
            q_vel = q_vel + pinv_Jd0 * (rep_magnitude - Jd0*pinv_J * ee_vel);

        elseif(matches(kinematics_solution, 'approximate'))
           
            % APPROXIMATE SOLUTION
            % -----------------------
            
            % calculate pseudo inverse
            pinv_J0 = J0'*(J0*J0'+ damping_factor_avoidance * eye(3))^-1;
        
        
            % calculate avoidance joints velocities
            q_vel = q_vel + N * pinv_J0*(avoid_vel);
    
        % -----------------------
        end

        % manipulability measures
        % -----------------------

        manipulability_primary = sqrt(det(J*J'))

        manipulability_secondary = sqrt(det(J0*J0'))


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
    current_dist = norm(ee_point'- goal_point(1:3));

    %% LOGS
    % -----------------------------------------------------------

    % save joint positions
    output.joints_positions = [output.joints_positions q];

    % save EE positions
    output.EE_positions = [output.EE_positions ee_point];

    % save goal distance
    output.goal_distances = [output.goal_distances current_dist];

    % save visited APF value
    if exist('xyz', 'var') == 1 
        output.values_APF(Niter+1).xyz = xyz';
        output.values_APF(Niter+1).grad = avoid_vel';
    end
    % save joint velocities
    output.q_velocities = [output.q_velocities q_vel];

    % save manipulability measurements of jacobians
    output.manipulability_primary = [output.manipulability_primary manipulability_primary];
    output.manipulability_secondary = [output.manipulability_secondary manipulability_secondary];


    % ITER COUNTER
    % -----------------------------------------------------------
    % raise iterations count
    Niter = Niter + 1;

end

%% REMOVE PROGRESS BAR
close(f)


end