function [dx,dy,dz] = interpolate_derivative(point, grid_distance, space_resolution)

    % numerical step
    d0 = 0.01;    

    % convert to grid scale
    point = point * space_resolution;    
     
    % get nearby grid cells positions
    X = [floor(point(1)) floor(point(1))+1];
    Y = [floor(point(2)) floor(point(2))+1];
    Z = [floor(point(3)) floor(point(3))+1];
     
    % get values of nearby cells
    V = ones(2,2,2);
    for x = 1:1:2 
        for y = 1:1:2
            for z = 1:1:2               
                
                % check if voxel location is in the range of the distance grid
                if (X(x) <= 0 || Y(y) <= 0 || Z(z) <= 0 || X(x) >= size(grid_distance,1) || Y(y) >= size(grid_distance,2) || Z(z) >= size(grid_distance,3)) 
                    % points beyond grid are set as obstacles during interpolation
                    V(y,x,z) = 1;
                else
                    % points in grid are set to correct voxels
                    V(y,x,z) = grid_distance(X(x)+1,Y(y)+1,Z(z)+1);
                end

            end
        end
    end

    % calculate forward derivative with respect to x
        x2 = point(1)+d0;
    
        % calculate forward difference if point is in defined interpolation area
        if(x2 < X(2))
           x_points = [point(1) point(2) point(3) ; x2 point(2) point(3)];
    
        % if not calculate backward difference
        else
            x1 = point(1)-d0;
            x_points = [x1 point(2) point(3) ; point(1) point(2) point(3)];
        end
    
    
        % interpolation for x
        x_values = interp3(X,Y,Z,V,x_points(:,1),x_points(:,2),x_points(:,3));
    
        dx = (x_values(2) - x_values(1)) / d0;


    % calculate derivative with respect to y
        y2 = point(2) + d0;
        
        % calculate forward difference if point is in defined interpolation area
        if(y2 < Y(2))
            y_points = [point(1) point(2) point(3) ; point(1) y2 point(3)];

        % if not calculate backward difference
        else
            y1 = point(2) - d0;
            y_points = [point(1) y1 point(3) ; point(1) point(2) point(3)];
        end
        
        % interpolation for y
        y_values = interp3(X,Y,Z,V,y_points(:,1),y_points(:,2),y_points(:,3));
        
        dy = (y_values(2) - y_values(1)) / d0;

    
    % calculate derivative with respect to z
        z2 = point(3) + d0;

        % calculate forward difference if point is in defined interpolation area
        if(z2 < Z(2))
            z_points = [point(1) point(2) point(3) ; point(1) point(2) z2];
        
        % if not calculate backward difference
        else
            z1 = point(3) - d0;
            z_points = [point(1) point(2) z1 ; point(1) point(2) point(3)];
        end
        
        % interpolation for z
        z_values = interp3(X,Y,Z,V,z_points(:,1),z_points(:,2),z_points(:,3));
        
        dz = (z_values(2) - z_values(1)) / d0;
        
end