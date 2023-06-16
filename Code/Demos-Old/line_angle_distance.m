function [alpha, d] = line_angle_distance(p1, p2, robot, theta)
   
% This function calculates the angle and distance for a robot to turn perpendicularly away from a line
% Inputs:
% p1: 1x2 vector representing the first point that forms the line
% p2: 1x2 vector representing the second point that forms the line
% robot: 1x2 vector representing the position of the robot
% theta: scalar representing the orientation angle of the robot in radians
% Outputs:
% alpha: scalar representing the angle in radians that the robot should rotate to turn perpendicularly away from the line
% d: scalar representing the distance from the robot to the line

% Calculate slope and y-intercept of line
m = (p2(2) - p1(2)) / (p2(1) - p1(1));
b = p1(2) - m * p1(1);

% Calculate distance from robot to line
d = abs(m * robot(1) - robot(2) + b) / sqrt(m^2 + 1);

% Calculate angle between robot orientation and line
gamma = atan(m) - theta;

% Calculate angle for robot to turn perpendicularly away from line
% alpha = pi/2 - gamma;
if gamma >= 0
    alpha = pi/2 - gamma;
else
    alpha = -pi/2 - gamma;
end

% turn for pi
alpha = alpha + pi;
end