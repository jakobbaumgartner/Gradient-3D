function display_grid(grid, min_display_value, space_resolution)
    % This function displays a 3D space matrix as a 3D plot, where each voxel 
    % is represented by a cube. Only voxels with a value above the 
    % min_display_value threshold are displayed.

    % create a new figure
    figure()

    % set the default line of sight for the 3D plot
    view(3)

    % loop through every element of the space matrix
    for x = 1:size(grid, 1)
        for y = 1:size(grid, 2)
            for z = 1:size(grid, 3)
                % get the start position of the voxel (bottom left corner)
                start = [(x-1), (y-1), (z-1)];
                
                % get the value of the current voxel
                voxel_value = grid(x,y,z);

                % draw the voxel as a cube if its value is above the 
                % min_display_value threshold
                if voxel_value > min_display_value
                    % draw a cube at the position defined by start, with 
                    % dimensions [1 1 1] and color black ('k'). The value 
                    % of the voxel is used to set the color of the cube.
                    voxel(start * space_resolution, [1 1 1] * space_resolution, 'k', voxel_value);
                end
                
                % keep the same plot for subsequent voxel cubes
                hold on
            end
        end
    end

end

