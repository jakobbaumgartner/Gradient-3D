function [rep_values] = REP_field_calculation(grid, kernels, point)

    % TODO: set input values beyond grid as occupied or free using optional
    % argument value

     % convert point to grid indices
     grid_point = round(point * grid.resolution);


    %% FOR EVERY KERNEL
    % --------------------------------------------------------------
    for i = 1:length(kernels)
        
        % get kernel
        kernel = kernels{i};
        
        % size
        kernel_size = size(kernel);

        % cut out of 3d grid area a window of the same size as kernel, 
        % with center in the point [x,y,z]
        half_size = floor(kernel_size / 2);
        x_range = grid_point(2) - half_size(1) : grid_point(2) + half_size(1); % grid goes y-x-z
        y_range = grid_point(1) - half_size(2) : grid_point(1) + half_size(2);
        z_range = grid_point(3) - half_size(3) : grid_point(3) + half_size(3);

        window = grid.grid(x_range, y_range, z_range);

        % multiply kernel and window, to get a weighted window
        weighted_window = window .* kernel;

        % sum all values in window
        rep_value = sum(weighted_window(:));

        % save to list of rep_values
        rep_values(i) = rep_value;
 
    
    
    end



end