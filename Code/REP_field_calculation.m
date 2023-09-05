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