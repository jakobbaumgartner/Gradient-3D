function [points X Y Z] = path_lab_1()
% Create three sets of points
points1 = [(130:0.6:160)' (80:1.6:160)' ones(51,1)*100]/100;
points2 = [(180:5.4:310)' (230:1.21:260)' ones(25,1)*110]/100;
points3 = [(315:5:450)' ones(28,1)*260 ones(28,1)*120]/100;

% Combine the three sets of points into a single matrix
points = [points1 ; points2; points3];

% Call the polyfit_xyz_trajectory function to fit a polynomial curve to the points
[X, Y, Z] = polyfit_xyz_trajectory(points);

% Plot the polynomial curve in 3D space
plot3(X*100, Y*100, Z*100, 'r-', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;

end