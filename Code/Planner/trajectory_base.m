function q_avoid = trajectory_base(robot_angles, ~, base_goal)

% -------------------------------------------------------------------------
% Code Description:
% This function calculates the trajectory for a robot base using a
% proportional controller to avoid obstacles. It takes the current robot
% angles, the obstacle position, and the desired goal position as inputs.
% It returns the vector of joint velocities for the robot base.

% Inputs:
% - robot_angles: a vector containing the current robot angles [x, y, phi_base, fi1 ... fi7]
%   where x and y are the base position coordinates and phi_base is the
%   base orientation.
% - ~: unused input (placeholder)
% - base_goal: a vector containing the desired goal position for the robot
%   base [x_goal, y_goal].

% Outputs:
% - q_avoid: a vector of joint velocities for the robot base, where the
%   first element represents the linear velocity (v) and the second
%   element represents the angular velocity (w).

% -------------------------------------------------------------------------


% base point pose
x = robot_angles(1);
y = robot_angles(2);
phi_base = robot_angles(3);

% get base position errors
dx = base_goal(1)-x;
dy = base_goal(2)-y;

% get goal orientation
phi_grad = atan2(dy, dx);

% get angle between goal vector and base orientation
phi_error = phi_grad - phi_base;

phi_error = atan2(sin(phi_error), cos(phi_error)); % constrain error to [-pi, pi]
phi_error = wrapToPi(phi_error);

% get angular velocity (P - regulator of angle error)
w = phi_error;


% get linear velocity
v = sqrt(dx^2+dy^2);

% construct vector of joint velocities
q_avoid = [v w zeros(1,7)]';


end