function [q_vel] = optimizer(robot_angles, diff, force_grid)
    % This function calculates the joint velocities for a robot given its angles and other parameters.
    %
    % Inputs:
    % robot_angles: a vector of the robot's joint angles
    % diff: a vector of distances from goal pose in task space
    % force_grid: not used in this function
    %
    % Output:
    % q_vel: a vector of the calculated joint velocities

    J_= jacobianNumeric(robot_angles); % Calculate the Jacobian matrix numerically

    % Sb - <global - base KS>
    Sb=[cos(robot_angles(3)) 0;
        sin(robot_angles(3)) 0;
        0             1];

    S=[Sb         zeros(3,7);
       zeros(7,2) eye(7)];

    J=J_*S; % Multiply the Jacobian by the S matrix

    % pseudo inverse
    pinv_J = pinv(J); % Calculate the pseudo-inverse of the Jacobian

    % calculate joint velocities
    q_vel = pinv_J * diff'; % Calculate the joint velocities using the pseudo-inverse of the Jacobian and the end-effector velocity

end