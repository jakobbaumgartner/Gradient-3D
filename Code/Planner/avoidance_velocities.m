function q_avoid = avoidance_velocities(robot_angles, space_resolution, grid_distance)

    % calculate EE position
    [T] = GeometricRobot(robot_angles);

    % get EE position
    point = T(1:3,4) * space_resolution;
     
    X = [floor(point(1)) floor(point(1))+1];
    Y = [floor(point(2)) floor(point(2))+1];
    Z = [floor(point(3)) floor(point(3))+1];
     
    V = ones(2,2,2);
    for x = 1:1:2 
        for y = 1:1:2
            for z = 1:1:2

                V(y,x,z) = grid_distance(y,x,z);

            end
        end
    end


    % Perform trilinear interpolation using interp3
    Vq = interp3(X,Y,Z,V,point(1),point(2),point(3))



end