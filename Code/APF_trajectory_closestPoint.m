function [joints_positions, EE_positions, goal_distances, q_velocities, ee_velocities, values_APF] = APF_trajectory_closestPoint(grid_field, grid_repulsive, space_resolution, goal_point, Tbase, q, varargin)

% Parse the optional arguments
p = inputParser;
addOptional(p, 'mid_joints', false); % Default value is false
addOptional(p, 'avoid_task', true); % Default value is true

parse(p, varargin{:});

mid_joints = p.Results.mid_joints;
avoid_task = p.Results.avoid_task;

% -----------------------------------------------------------

% function parameters
goal_dist = 0.01; % distance which satisfies ending of optimization
damping_factor = 0.1; % damping factor to avoid inverse Jacobain matrix singularities
Tstep = 0.01; % time step
Nmax = 1000; % max number of iterations

% joints range
q_range = [2.8973 -2.8973;
           1.7628 -1.7628;
           2.8973 -2.8973;
           -0.0698 -3.0718;
           2.8973 -2.8973;
           3.7525 -0.0175;
           2.8973 -2.8973];

% number of points per segment for obstacle avoidance task
points_per_segment = 5*[1 1 1 1 1 1 1];

% -----------------------------------------------------------

% get a list of transforms for every APF detection point on the robot
transformations_list = getPartialTransformationsListPanda('points_per_segment', points_per_segment);

% get current robot pose
[robot_transforms] = GeometricPandaMATLAB(q, Tbase);

% set current ee point to start point
ee_point = robot_transforms(1:3,4,8);

% calculate distance from goal
current_dist = norm(ee_point'- goal_point(1:3));

% add starting positions to saved array
EE_positions = [ee_point];
joints_positions = [q];
goal_distances = [current_dist];
q_velocities = [];
values_APF = [interpolate_points(ee_point, grid_field, space_resolution)];
ee_velocities = [];

% set itarations count
Niter = 0;

% trajectory calculation loop
while current_dist > goal_dist && Niter < Nmax  

    % OPTION JACOBIAN
    [~,~,J]=kinmodel_panda(q); % calculate jacobian

    % --------------------------------------------------
    % OPTION MATLAB ROBOT TOOLBOX JACOBIAN
%         [~,robot] = GeometricPandaMATLAB(q, Tbase); % get robot configuration
%         config = homeConfiguration(robot); % set configuration
%         
%         % set joint positions
%         for i = 1:1:7
%             config(i).JointPosition = q(i);
%         end
%     
%         % toolbox jacobian returns first three rows angular velocity and
%         % second three rows linear velocities, change rows to be in line with
%         % used convention in default jacobian
%         J2 = geometricJacobian(robot,config,'body7');
%         J3 = [J2(4:6,:) ; J2(1:3,:)];
%     
%         J = J3;
    % --------------------------------------------------

    % calculate pseudo inverse
    pinv_J = J'*(J*J'+ damping_factor^2 * eye(6))^-1; % damping to avoid singularities

    % calculate ee velocities by interpolating APF
    [dx,dy,dz] = interpolate_derivative(ee_point, grid_field, space_resolution);
    ee_vel = -[dx ; dy ; dz ; 0 ; 0 ; 0]; 

    % calculate joint velocities using inverse kinematics
    q_vel = pinv_J * ee_vel;    

    % calculate Null Space 
    N = (eye(7)-pinv_J*J);


    % add mid-joints secondary task
    if mid_joints 

        dq_sec = zeros(7,1);

        % option 1 - joints speeds proportional joints difference from mid
        for i = 1:1:7
            dq_sec(i) = (sum(q_range(i,:))/2) - q(i);
        end

        % option 2 - The distance from mechanical joint limits using
        % Sicilliano equation and partial derivatives
            % ... TODO

        % Primary + Null-Space Task
        q_vel = q_vel + N*dq_sec;

    end

    % APF Avoidance Task
    % --------------------------------------------------
 	if avoid_task

        % get APF value in every of calculated 3D points
        APF_interpolated_values = []; % [value x y z]
    
        for i = 1:1:length(transformations_list) % for every detection point
        
            % calculate APF value and get T matrix
            [value, transform] = getPartialAPFValueRobotTreePoint(grid_repulsive, space_resolution, transformations_list(i).tree, Tbase, q);
        
            % save to list
            APF_interpolated_values = [APF_interpolated_values ; value, transform(1:3,4)'];
        
        end
        
        % find in which point APF is the highest
        [val,ind] = max(APF_interpolated_values(:,1));
    
        [dx,dy,dz] = interpolate_derivative(APF_interpolated_values(ind,2:4), grid_repulsive, space_resolution);
        avoid_vel = -[dx ; dy ; dz ; 0 ; 0 ; 0];
    
        % calculate Jacobian in that point
        robot = transformations_list(ind).tree;
        config = homeConfiguration(robot); % set configuration
            
        % set joint positions
        for i = 1:1:robot.NumBodies
            config(i).JointPosition = q(i);
        end
        
        % toolbox jacobian returns first three rows angular velocity and
        % second three rows linear velocities, change rows to be in line with
        % used convention in default jacobian
        J2 = geometricJacobian(robot,config,'body'+string(robot.NumBodies));
        J0 = zeros(6,7);
        J0(1:3,1:size(J2,2)) = J2(4:6,:);
        J0(4:6,1:size(J2,2)) = J2(1:3,:);

        
            % (TODO: if there was a previously different point with highest APF calculate Jacobian in this previous point) 
            % (TODO: calculate weighting coefficients)
    
        % calculate pseudo inverse
        pinv_J0 = (J0*N)'*((J0*N)*(J0*N)'+ damping_factor^2 * eye(6))^-1; % damping to avoid singularities
    
        % calculate avoidance joints velocities
        q_vel = q_vel + 1 * pinv_J0*(avoid_vel - J0*pinv_J*ee_vel);

    end
    % --------------------------------------------------

    % calculate new joint positions
    q = q + q_vel * Tstep;

    % calculate new EE position
    [robot_transforms] = GeometricPandaMATLAB(q, Tbase);
    ee_point = robot_transforms(1:3,4,8);

    % update goal distance
    current_dist = norm(ee_point'- goal_point(1:3));

    % save joint positions
    joints_positions = [joints_positions q];

    % save EE positions
    EE_positions = [EE_positions ee_point];

    % save goal distance
    goal_distances = [goal_distances current_dist];

    % save visited APF value
    values_APF = [values_APF interpolate_points(ee_point, grid_field, space_resolution)];

    % save joint velocities
    q_velocities = [q_velocities q_vel];

    % save EE velocities
    ee_velocities = [ee_velocities ee_vel];

    % raise iterations count
    Niter = Niter + 1;

end


end