function [values] = interpolate_points(points, center_point, grid_distance)

    % get EE position
    center_point = center_point * space_resolution;
     
    % get nearby grid cells positions
    X = [floor(center_point(1)) floor(center_point(1))+1];
    Y = [floor(center_point(2)) floor(center_point(2))+1];
    Z = [floor(center_point(3)) floor(center_point(3))+1];
     
    % get values of nearby cells
    V = ones(2,2,2);
    for x = 1:1:2 
        for y = 1:1:2
            for z = 1:1:2
                V(y,x,z) = grid_distance(y,x,z);
            end
        end
    end

%     Example:
    %     % Perform trilinear interpolation using interp3
    %     Vq = interp3(X,Y,Z,V,point(1),point(2),point(3))

    % interpolation
    values = interp3(X,Y,Z,V,points(:,1),points(:,2),points(:,3));


end