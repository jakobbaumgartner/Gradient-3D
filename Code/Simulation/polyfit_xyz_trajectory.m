function [X, Y, Z] = polyfit_xyz_trajectory(points, degree)
% POLYFIT_XYZ_TRAJECTORY Returns a 3D trajectory using polyfit
%   [X, Y, Z] = polyfit_xyz_trajectory(points, degree) returns three arrays
%   containing the X, Y, and Z coordinates of a 3D trajectory generated
%   using polyfit. The input 'points' is an N-by-3 array containing the
%   X, Y, and Z coordinates of N points along the trajectory. Degree is 
%   the degree of fitted function. 

% Extract X, Y, and Z coordinates from points array
x = points(:, 1);
y = points(:, 2);
z = points(:, 3);

% Define polynomial degree and number of points for interpolation
% degree = 5;  % Change degree to adjust the level of fitting accuracy
npoints = 100; % Change npoints to adjust the number of points in the trajectory

% Generate a vector of evenly-spaced values between min(x) and max(x)
xi = linspace(min(x), max(x), npoints);

% Fit polynomials to X, Y, and Z coordinates separately
px = polyfit(x, x, degree);
py = polyfit(x, y, degree);
pz = polyfit(x, z, degree);

% Evaluate polynomials at each value of xi
yi = polyval(py, xi);
zi = polyval(pz, xi);
xi = polyval(px, xi);

% Return X, Y, and Z arrays
X = xi';
Y = yi';
Z = zi';
end
