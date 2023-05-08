function [grid_occupancy, space_resolution] = map_lab1()

    % call the create_3d_space function with the desired dimensions
    space_resolution = 10; % 10x10x10 cm voxels
    space_height = 2; 
    space_width = 5;
    space_length = 5;
    
    % create grid obstacle
    grid_occupancy = create_3d_grid(space_resolution, space_height, space_width, space_length);
    
    % add square table
    grid_occupancy = add_box(grid_occupancy, space_resolution, 200, 210, 100, 200, 1, 50);
    grid_occupancy = add_box(grid_occupancy, space_resolution, 290, 300, 100, 200, 1, 50);
    grid_occupancy = add_box(grid_occupancy, space_resolution, 200, 300, 100, 110, 1, 50);
    grid_occupancy = add_box(grid_occupancy, space_resolution, 200, 300, 190, 200, 1, 50); 
    
    % % add counter 1
    grid_occupancy = add_box(grid_occupancy, space_resolution, 90, 100, 1, 300, 1, 50);
    grid_occupancy = add_box(grid_occupancy, space_resolution, 1, 100, 1, 10, 1, 50);
    
    
    % add counter 2
    grid_occupancy = add_box(grid_occupancy, space_resolution, 100, 400, 300, 310, 1, 50);
    grid_occupancy = add_box(grid_occupancy, space_resolution, 390, 400, 300, 400, 1, 50);






end