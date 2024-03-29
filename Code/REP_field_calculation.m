function [rep_values, logs_rep_values] = REP_field_calculation(grid, kernels, point, varargin)

    %   REP_VALUES = REP_FIELD_CALCULATION(GRID, KERNELS, POINT, VARARGIN) computes a set of
    %   representative values for a 3D grid at a specified point, using a set of 3D kernels.
    %
    %   Inputs:
    %   - GRID: A structure with two fields:
    %       - 'grid': A 3D matrix representing the grid (y-x-z).
    %       - 'resolution': A scalar representing the resolution of the grid.
    %   - KERNELS: A cell array of 3D matrices, each representing a kernel.
    %   - POINT: A 1x3 vector specifying the coordinates of the point in the grid
    %            at which to compute the representative values.
    %   - VARARGIN: Optional input arguments:
    %       - 'interpolation_mode': A boolean (true or false) to enable or disable interpolation.
    %       - 'box_value': Use this parameter to set occupation of the cells that
    %         are beyond the inputed grid space.
    %
    %   Outputs:
    %   - REP_VALUES: A 1xN vector of representative values, where N is the number
    %                 of kernels. The i-th element of REP_VALUES is the representative
    %                 value computed using the i-th kernel in KERNELS.
    %   - LOGS_REP_VALUES: A list of objects containing the indexes of the cells used to calculate the representative values.


    %% SETTINGS:
    % --------------------------------------------------------------

    % parse optional input arguments
    p = inputParser;
    addParameter(p, 'interpolation_mode', true);
    addParameter(p, 'box_value', 0);
    parse(p, varargin{:});

    % assign values from input parser
    interpolation_mode = p.Results.interpolation_mode;
    box_value = p.Results.box_value;

    % --------------------------------------------------------------    

    % init logs variable
    logs_rep_values = struct('indexes', []);
    
    %% USE INTERPOLATION
    if interpolation_mode
        
        center_point = point * grid.resolution;
         
        % get nearby grid cells indexes
        X = [floor(center_point(1)) ceil(center_point(1))];
        
        % if both indexes are the same, add 1 to the second one (to avoid interpolation error)
        if(X(1) == X(2))
            X(2) = X(2) + 1;
        end
        Y = [floor(center_point(2)) ceil(center_point(2))];

        % if both indexes are the same, add 1 to the second one (to avoid interpolation error)
        if(Y(1) == Y(2))
            Y(2) = Y(2) + 1;
        end

        % if both indexes are the same, add 1 to the second one (to avoid interpolation error)
        Z = [floor(center_point(3)) ceil(center_point(3))];
        if(Z(1) == Z(2))
            Z(2) = Z(2) + 1;
        end


        % FULL 3D INTERPOLATION
        % --------------------------------------------------------------

        % get values of nearby cells
        Vx = ones(2,2,2);
        Vy = ones(2,2,2);
        Vz = ones(2,2,2);

        for x = 1:1:2 
            for y = 1:1:2
                for z = 1:1:2               
                    
                        % set value of the point
                        vect_val = calculateFieldVector([X(x),Y(y),Z(z)]);
                        Vx(y,x,z) = vect_val(1);
                        Vy(y,x,z) = vect_val(2);
                        Vz(y,x,z) = vect_val(3);
                           
                end
            end
        end

        % interpolation
        rep_values = [interp3(X,Y,Z,Vx,center_point(1),center_point(2),center_point(3)), interp3(X,Y,Z,Vy,center_point(1),center_point(2),center_point(3)), interp3(X,Y,Z,Vz,center_point(1),center_point(2),center_point(3))];
      

    %% APPROXIMATE POINT VALUE ONLY
    else

        % convert point to grid indices
        grid_point = round(point * grid.resolution);

        % get field value
        rep_values = calculateFieldVector(grid_point);


    end

    %% CALCULATE FIELD VECTOR FUNCTION
    % --------------------------------------------------------------
    function rep_values = calculateFieldVector(grid_point)


        % FOR EVERY KERNEL
        % --------------------------------------------------------------
        for i = 1:length(kernels)
            
            % get kernel
            kernel = kernels{i};
            
            % size
            kernel_size = size(kernel);
            half_size = floor(kernel_size / 2);
    
            % Calculate x, y, and z ranges for the window while ensuring they are positive and within grid boundaries.
            x_cells = round((grid_point(2) - half_size(1)) : (grid_point(2) + half_size(1)) -1) +1; % grid goes y-x-z
            y_cells = round((grid_point(1) - half_size(2)) : (grid_point(1) + half_size(2)) -1) +1;
            z_cells = round((grid_point(3) - half_size(3)) : (grid_point(3) + half_size(3)) -1) +1;

            % init every cells in window to occupued by default
            window = ones(kernel_size)*box_value;

            % create indexes for cutout
            x_valid_index = find(x_cells>=1,1):find(x_cells<=size(grid.grid,3),1,'last');
            y_valid_index = find(y_cells>=1,1):find(y_cells<=size(grid.grid,3),1,'last');
            z_valid_index = find(z_cells>=1,1):find(z_cells<=size(grid.grid,3),1,'last');

            % read which cells are occupued / free
            window(x_valid_index, y_valid_index, z_valid_index) = grid.grid(x_cells(x_valid_index), y_cells(y_valid_index), z_cells(z_valid_index));
    
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

            % save cell positions to logs
            logs_rep_values.indexes  = [logs_rep_values.indexes; {x_cells, y_cells, z_cells}]; % [x_cells, y_cells, z_cells] one line per kernel

            % save kernel weights to logs
            logs_rep_values.kernels{i} = kernel;
     
        
        end



    end

end