function [joints_positions, EE_positions, goal_distances, q_velocities, ee_velocities, values_APF] = ObstacleGrid_RTConvolution_OnePoint(grid, grid_field, goal_point, Tbase, q, varargin)

% Parse the optional arguments
p = inputParser;
addOptional(p, 'mid_joints', true); % Default value is false
addOptional(p, 'avoid_task', true); % Default value is true

parse(p, varargin{:});

mid_joints = p.Results.mid_joints;
avoid_task = p.Results.avoid_task;

% -----------------------------------------------------------

% function parameters
goal_dist = 0.02; % distance which satisfies ending of optimization
damping_factor = 0.01; % damping factor to avoid inverse Jacobain matrix singularities
Tstep = 0.1; % time step
Nmax = 1000; % max number of iterations
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

% weights for different tasks
wp = 5; % primary task
wm = 5; % mid-joints task
wa = 0.5; % obstacle avoidance task

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
values_APF = [];
ee_velocities = [];

% set directional kernels
sigma = 41;

kernel_length = 61;
kernel_width = 9;
kernel_height = kernel_width;

kernel_x = directional_kernel_3d('x', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
kernel_y = directional_kernel_3d('y', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
kernel_z = directional_kernel_3d('z', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;

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

    % --------------------------------------------------
    % OPTION KINEMATICS CLASSIC END EFFECTOR

%     % calculate ee velocities
%     ee_vel = goal_point(1:3)' - ee_point;
% 
%     % calculate joint velocities using inverse kinematics
%     q_vel = pinv_J * (wp * [ee_vel ; 0 ; 0 ; 0]);

    % --------------------------------------------------

    % EE kinematics using APF  %%%% TODODODODODODO TODO !!!! (FIX THIS,
    % REPULSIVE FIELD NOT WORKING REALLY)

    % calculate ee velocities by interpolating APF
    [dx,dy,dz] = interpolate_derivative(ee_point, grid_field, space_resolution);
    ee_vel = -[dx ; dy ; dz ; 0 ; 0 ; 0]; 

    % calculate joint velocities using inverse kinematics
    q_vel = pinv_J * (wp * ee_vel);   


    % calculate Null Space 
    N = (eye(7)-pinv_J*J);


    % add mid-joints secondary task
    if mid_joints 

        dq_sec = zeros(7,1);

        % option 1 - joints speeds proportional joints difference from mid
        for i = 1:1:7
            dq_sec(i) = wm * (sum(q_range(i,:))/2) - q(i);
        end

        % option 2 - The distance from mechanical joint limits using
        % Sicilliano equation and partial derivatives
            % ... TODO

        % Primary + Null-Space Task
        q_vel = q_vel + N * dq_sec;

    end

    % APF Avoidance Task
    % --------------------------------------------------
 	if avoid_task

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
    
        % get transform
        transform = getTransform(robot_tree,config,'body'+string(robot_tree.NumBodies));
    
        % add base transformation
        transform = Tbase * transform;
    
        % get x,y,z values
        xyz = ceil(transform(1:3,4)*grid.resolution);
        
        
        % GET VELOCITIES USING DIRECTIONAL KERNELS
        % --------------------

        % cut out local part of the grid around joint 4
        cutout = grid.grid(((1:kernel_length)+xyz(2)-ceil(kernel_length/2)), ((1:kernel_width)+xyz(1)-ceil(kernel_width/2)), ((1:kernel_height)+xyz(3)-ceil(kernel_height/2)));

        % calculate x - distance transform
%         C = cutout .* kernel_x; % Perform element-wise multiplication
        dx = 0; %sum(C(:)); % Sum all values 
      
        % calculate y - distance transform
        C = cutout .* kernel_y; % Perform element-wise multiplication
        dy = sum(C(:)); % Sum all values 

        % calculate z - distance transform
%         C = cutout .* kernel_z; % Perform element-wise multiplication
        dz = 0; %sum(C(:)); % Sum all values  

        avoid_vel = -[dx ; dy ; dz ; 0 ; 0 ; 0];
        avoid_vel = wa * avoid_vel; % scale



        % --------------------------------------------------

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
        J0 = zeros(6,7);
        J0(1:3,1:size(J2,2)) = J2(4:6,:);
        J0(4:6,1:size(J2,2)) = J2(1:3,:);

        
            % (TODO: if there was a previously different point with highest APF calculate Jacobian in this previous point) 
            % (TODO: calculate weighting coefficients)
    
        % calculate pseudo inverse
%         pinv_J0 = (J0*N)'*((J0*N)*(J0*N)'+ damping_factor^2 * eye(6))^-1; % damping to avoid singularities
%         pinv_J0 = J0'*(J0*J0'+ damping_factor^2 * eye(6))^-1;
    
        % calculate avoidance joints velocities
%         q_vel = q_vel + pinv_J0*(avoid_vel - J0*pinv_J*ee_vel);
%         q_vel = q_vel + pinv_J0*(avoid_vel);

    % APPROXIMATE SOLUTION
    % -----------------------
    
    % calculate pseudo inverse
    pinv_J0 = J0'*(J0*J0'+ damping_factor^2 * eye(6))^-1;


    % calculate avoidance joints velocities
    q_vel = q_vel + N * pinv_J0*(avoid_vel);

    % -----------------------


    end
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

    % save joint positions
    joints_positions = [joints_positions q];

    % save EE positions
    EE_positions = [EE_positions ee_point];

    % save goal distance
    goal_distances = [goal_distances current_dist];

    % save visited APF value
    if exist('xyz', 'var') == 1 
        values_APF(Niter+1).xyz = xyz'/grid.resolution;
        values_APF(Niter+1).grad = avoid_vel';
    end
    % save joint velocities
    q_velocities = [q_velocities q_vel];

    % save EE velocities
    ee_velocities = [ee_velocities ee_vel];

    % raise iterations count
    Niter = Niter + 1;

end


end