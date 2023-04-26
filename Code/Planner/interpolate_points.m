function [values] = interpolate_points(points, grid_distance, space_resolution)
    
    % This function performs trilinear interpolation on a set of 3D points using
    % a nearby grid. The output is the interpolated values for each of the
    % input points.
    %
    % INPUTS:
    % - points: a 3xN matrix of N points in 3D space
    % - grid_distance: a 3D matrix of distances from objects
    % - space_resolution: scalar representing the resolution of the 3D space
    %
    % OUTPUTS:
    % - values: a 1xN vector of interpolated values for each input point

    % convert to grid scale
    points = points * space_resolution;
    center_point = points(:,1)
    
     
    % get nearby grid cells positions
    X = [floor(center_point(1)) floor(center_point(1))+1]
    Y = [floor(center_point(2)) floor(center_point(2))+1]
    Z = [floor(center_point(3)) floor(center_point(3))+1]
     
    % get values of nearby cells
    V = ones(2,2,2);
    for x = 1:1:2 
        for y = 1:1:2
            for z = 1:1:2
                V(y,x,z) = grid_distance(X(x)+1,Y(y)+1,Z(z)+1);
            end
        end
    end


    % interpolation
    values = interp3(X,Y,Z,V,points(1,:),points(2,:),points(3,:));


end