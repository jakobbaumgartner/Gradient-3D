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
    %
    % TODO: box_mode add walls around the grid space

    %% SETTINGS:
    % --------------------------------------------------------------

    % use interpolation
    interpolation_mode = true; % true / false

    % set values outside the known grid to
    box_value = 0;


    % --------------------------------------------------------------    
    
    %% USE INTERPOLATION
    if interpolation_mode
        
        center_point = point * grid.resolution
         
        % get nearby grid cells indexes
        X = [floor(center_point(1)) ceil(center_point(1))];
        Y = [floor(center_point(2)) ceil(center_point(2))];
        Z = [floor(center_point(3)) ceil(center_point(3))];

        % 1D INTERPOLATION IF TWO POINTS ARE LOCATED EXACTLY IN THE
        % CENTER OF GRID CELLS
        % --------------------------------------------------------------
        no_interp_dist = 0.0001; 

        % if x and y and z are center
        if mod(center_point(1),1) < no_interp_dist && mod(center_point(2),1) < no_interp_dist && mod(center_point(3),1) < no_interp_dist
            rep_values = calculateFieldVector(center_point)
 
        % if x and y are in center
        elseif mod(center_point(1),1) < no_interp_dist && mod(center_point(2),1) < no_interp_dist
            % 1D interpolate - only z

            Vx = ones(1,2);
            Vy = ones(1,2);
            Vz = ones(1,2);

            % get vector field values for x,y,z
            for j = 1:1:length(Z)
                vect_val = calculateFieldVector([center_point(1),center_point(2),Z(j)]);
                Vx(j) = vect_val(1);
                Vy(j) = vect_val(2);
                Vz(j) = vect_val(3);
            end

            % move sampled points coordinates to the mid of the cells
            Z = [floor(center_point(3)) ceil(center_point(3))]+0.5;

            % interpolate
            rep_values = [interp1(Z,Vx,center_point(3)), interp1(Z,Vy,center_point(3)), interp1(Z,Vz,center_point(3))];
            
            % check if value is NaN (probably because both interp. value
            % are 0) and set it to 0
            rep_values(isnan(rep_values)) = 0;

        % if y and z are in center
        elseif mod(center_point(2),1) < no_interp_dist && mod(center_point(3),1) < no_interp_dist
            % 1D interpolate - only x

            Vx = ones(1,2);
            Vy = ones(1,2);
            Vz = ones(1,2);

            % get vector field values for x,y,z
            for j = 1:1:length(X)
                vect_val = calculateFieldVector([X(j),center_point(2),center_point(3)]);
                Vx(j) = vect_val(1);
                Vy(j) = vect_val(2);
                Vz(j) = vect_val(3);
            end

            % move sampled points coordinates to the mid of the cells
            X = [floor(center_point(1)) ceil(center_point(1))] + 0.5;

            % interpolate
            rep_values = [interp1(X,Vx,center_point(1)), interp1(X,Vy,center_point(1)), interp1(X,Vz,center_point(1))];

            % check if value is NaN (probably because both interp. value
            % are 0) and set it to 0
            rep_values(isnan(rep_values)) = 0;
       
        % if x and z are in exact center cells
        elseif mod(center_point(1),1) < no_interp_dist && mod(center_point(3),1) < no_interp_dist
            % 1D interpolate - only y

            Vx = ones(1,2);
            Vy = ones(1,2);
            Vz = ones(1,2);

            % get vector field values for x,y,z
            for j = 1:1:length(Y)
                vect_val = calculateFieldVector([center_point(1),Y(j),center_point(3)]);
                Vx(j) = vect_val(1);
                Vy(j) = vect_val(2);
                Vz(j) = vect_val(3);
            end

            % move sampled points coordinates to the mid of the cells
            Y = [floor(center_point(2)) ceil(center_point(2))] + 0.5;

            % interpolate
            rep_values = [interp1(Y,Vx,center_point(2)), interp1(Y,Vy,center_point(2)), interp1(Y,Vz,center_point(2))];

            % check if value is NaN (probably because both interp. value
            % are 0) and set it to 0
            rep_values(isnan(rep_values)) = 0;

        else
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


        end



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
            x_cells = round((grid_point(2) - half_size(1)) : (grid_point(2) + half_size(1))) % grid goes y-x-z
            y_cells = round((grid_point(1) - half_size(2)) : (grid_point(1) + half_size(2)));
            z_cells = round((grid_point(3) - half_size(3)) : (grid_point(3) + half_size(3)));

            % init every cells in window to occupued by default
            window = ones(kernel_size)*box_value;

            % create indexes for cutout
            x_valid_index = find(x_cells>=1,1):find(x_cells<=size(grid.grid,3),1,'last')
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
     
        
        end



    end

end