function [rep_values] = REP_field_calculation(grid, kernels, point)

    %   REP_VALUES = REP_FIELD_CALCULATION(GRID, KERNELS, POINT) computes a set of
    %   representative values for a 3D grid at a specified point, using a set of 3D kernels.
    %
    %   Inputs:
    %   - GRID: A structure with two fields:
    %       - 'grid': A 3D matrix representing the grid (y-x-z).
    %       - 'resolution': A scalar representing the resolution of the grid.
    %   - KERNELS: A cell array of 3D matrices, each representing a kernel.
    %   - POINT: A 1x3 vector specifying the coordinates of the point in the grid
    %            at which to compute the representative values.
    %
    %   Outputs:
    %   - REP_VALUES: A 1xN vector of representative values, where N is the number
    %                 of kernels. The i-th element of REP_VALUES is the representative
    %                 value computed using the i-th kernel in KERNELS.

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

        % Calculate x, y, and z ranges for the window while ensuring they are positive and within grid boundaries.
        half_size = floor(kernel_size / 2);
        x_range = max(grid_point(2) - half_size(1), 1) : min(grid_point(2) + half_size(1), size(grid.grid, 2)); % grid goes y-x-z
        y_range = max(grid_point(1) - half_size(2), 1) : min(grid_point(1) + half_size(2), size(grid.grid, 1));
        z_range = max(grid_point(3) - half_size(3), 1) : min(grid_point(3) + half_size(3), size(grid.grid, 3));

        window = grid.grid(x_range, y_range, z_range);

        % fix kernel to be only the part inside the grid
        x_start = 1 + (1 - (grid_point(2) - half_size(1) > 0)) * (1 - (grid_point(2) - half_size(1)));
        x_end = kernel_size(2) - ((grid_point(2) + half_size(1)) - size(grid.grid, 2) > 0) * ((grid_point(2) + half_size(1)) - size(grid.grid, 2));
        
        y_start = 1 + (1 - (grid_point(1) - half_size(2) > 0)) * (1 - (grid_point(1) - half_size(2)));
        y_end = kernel_size(1) - ((grid_point(1) + half_size(2)) - size(grid.grid, 1) > 0) * ((grid_point(1) + half_size(2)) - size(grid.grid, 1));
        
        z_start = 1 + (1 - (grid_point(3) - half_size(3) > 0)) * (1 - (grid_point(3) - half_size(3)));
        z_end = kernel_size(3) - ((grid_point(3) + half_size(3)) - size(grid.grid, 3) > 0) * ((grid_point(3) + half_size(3)) - size(grid.grid, 3));
        
        kernel = kernel(y_start:y_end, x_start:x_end, z_start:z_end);
        
        % fix the window size to match the kernel size
        window = window(1:size(kernel,1), 1:size(kernel,2), 1:size(kernel,3));

        % multiply kernel and window, to get a weighted window
        weighted_window = window .* kernel;

        % sum all values in window
        rep_value = sum(weighted_window(:));

        % save to list of rep_values
        rep_values(i) = rep_value;
 
    
    end

end