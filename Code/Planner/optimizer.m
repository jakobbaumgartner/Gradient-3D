function [q_vel, q_avoid, q_prim] = optimizer(robot_angles, diff, grid_distance, space_resolution)
   
    % This function calculates the joint velocities for a robot given its angles and other parameters.
    %
    % Inputs:
    % robot_angles: a vector of the robot's joint angles
    % diff: a vector of distances from goal pose in task space
    % force_grid: not used in this function
    %
    % Output:
    % q_vel: a vector of the calculated joint velocities
    % q_avoid: a vector of the calculated secondary task joint velocities

    jacobian_option = 'analitic'; % analitic, numeric, geometric
    secondary_option = 'avoidance_base'; % none, avoidance_base

    q_avoid = [];

    % P - primary
    Kpee = 2;
    % P - secondary
    Ksec = 8;


    % JACOBIAN CALCULATION

    if strcmp(jacobian_option, 'analitic')

   
        J = jacobianAnalitic(robot_angles); % Calculate the Jacobian matrix analitically

        pinv_J = pinv(J);

    
    elseif strcmp(jacobian_option,'numeric')

        J_= jacobianNumeric(robot_angles); % Calculate the Jacobian matrix numerically
    
        % Sb - <global - base KS>
        Sb=[cos(robot_angles(3)) 0;
            sin(robot_angles(3)) 0;
            0             1];
    
        S=[Sb         zeros(3,7);
           zeros(7,2) eye(7)];
    
        J=J_*S; % Multiply the Jacobian by the S matrix

        pinv_J = pinv(J); % Calculate the pseudo-inverse of the Jacobian

   
    elseif strcmp(jacobian_option, 'geometric')

        % NOT WORKING

        J = jacobianGeometric(robot_angles); % Calculate the Jacobian matrix analitically

        damping_factor = 0.001;

        pinv_J = J'*(J*J'+ damping_factor^2 * eye(6))^-1;

    end


    % (SECONDARY) VELOCITIES CALCULATION

    if strcmp(secondary_option, 'avoidance_base')
        % calculate joint velocities with accounting for obstacles density grid

        q_prim = pinv_J * diff'; % Calculate the joint velocities using the pseudo-inverse of the Jacobian and the end-effector velocity

        % calculate secondary velocities
        q_avoid = avoidance_base(robot_angles, space_resolution, grid_distance, Ksec);

        % add secondary task
        q_vel = q_prim * Kpee + (eye(9) - pinv_J * J ) * q_avoid;


    else
        % calculate joint velocities, without secondary task
        q_vel = pinv_J * diff' * Kpee; % Calculate the joint velocities using the pseudo-inverse of the Jacobian and the end-effector velocity
    end


end