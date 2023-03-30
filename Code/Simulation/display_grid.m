function display_grid(grid, min_display_value, view_angle)

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

    % add floor

        % Define the x and y ranges
        x = linspace(0, 1, size(grid,1))*size(grid,1);
        y = linspace(0, 1, size(grid,2))*size(grid,2);
        
        % Create a grid of points from x and y
        [X,Y] = meshgrid(x,y);
        
        % Define the function to plot
        Z = zeros(size(grid,1),size(grid,2));
            
        % Plot the surface
        surf(X,Y,Z, 'FaceAlpha',0.5, 'FaceColor','y');
%         set(hSurface,'FaceColor',[1 0 0],'FaceAlpha',0.5);


    % set fixed axis size
    axis([0 size(grid,1) 0 size(grid,2) 0 size(grid,3)])

    % set viewing angle
    view(view_angle);


end