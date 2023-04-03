function [robot_angles] = simulate_step(robot_angles, q_vel)
    % This function simulates a step of the robot's movement
    %
    % Inputs:
    % robot_angles: a vector containing the current angles of the robot
    % q_vel: a vector containing the current velocities of the robot
    %
    % Outputs:
    % robot_angles: a vector containing the updated angles of the robot

    time_step = 0.1; % time step for simulation

    % Convert from vw to xy and phi
    robot_angles(1:3) = robot_angles(1:3) + [ q_vel(1) * cos(robot_angles(3) + q_vel(2) * time_step / 2);
                                              q_vel(1) * sin(robot_angles(3) + q_vel(2) * time_step / 2);
                                              q_vel(2) ]' * time_step;

    % Calculate new joint angles 
    robot_angles(4:10) = robot_angles(4:10) + q_vel(4:10)' * time_step;
end