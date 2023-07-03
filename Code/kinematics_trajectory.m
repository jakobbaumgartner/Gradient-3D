function [joints_positions, EE_positions, goal_distances] = kinematics_trajectory(goal_point, Tbase, q)

% function parameters
goal_dist = 0.01; % distance which satisfies ending of optimization
damping_factor = 0.1; % damping factor to avoid inverse Jacobain matrix singularities
Tstep = 0.1; % time step

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

% trajectory calculation loop
while current_dist > goal_dist

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

    % calculate ee velocities
    ee_vel = goal_point(1:3)' - ee_point;

    % ??? normalize EE velocities so that there is constant EE movement
%     ee_vel = ee_vel / norm(ee_vel);

    % calculate joint velocities using inverse kinematics
    q_vel = pinv_J * [ee_vel ; 0 ; 0 ; 0];

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


end




end