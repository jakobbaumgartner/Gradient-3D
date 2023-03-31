function display_grid(grid, min_display_value)

    % create a new figure
    figure()

    % view - uses the default line of sight for 2-D or 3-D plots. Specify dim as 2 for the default 2-D view or 3 for the default 3-D view.
    view(3)

    % loop through every element of the space matrix
    for x = 1:size(grid, 1)
        for y = 1:size(grid, 2)
            for z = 1:size(grid, 3)
                % get the start position and size of the voxel
                start = [(x-1), (y-1), (z-1)];
                
                % value of current voxel
                voxel_value = grid(x,y,z);

                % draw the voxel (if its value is above min display value)
                if voxel_value > min_display_value
                    voxel(start, [1 1 1], 'k', voxel_value);
                end
                
                % keep same plot
                hold on
            end
        end
    end

end