function grid = add_box(grid, resolution, x1, x2, y1, y2, z1, z2)
    % Adds a rectangular box to a 3D grid.
    
    % Round up the coordinates of the box to the nearest grid cell
    x_range = ceil(x1/resolution):ceil(x2/resolution);
    y_range = ceil(y1/resolution):ceil(y2/resolution);
    z_range = ceil(z1/resolution):ceil(z2/resolution);
    
    % Set the corresponding cells in the grid to 1
    grid(x_range, y_range, z_range) = 1;

end
