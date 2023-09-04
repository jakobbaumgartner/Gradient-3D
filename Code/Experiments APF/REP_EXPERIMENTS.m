% REPULSIVE FIELD GENERATION
close all
%% SET POINT
point = [0.7,0.7,1]

%% SELECT MAP
map_selection = 'roof'
% -----------------------------------------------------------

% create grid
grid = OctoGrid(2,2,2,100);

if (matches( map_selection , 'wall' ))

    % add the wall
    grid.addBox(0.2,0.8,0,1.6,0.1,1.5)

elseif (matches( map_selection , 'cross' ))

    % add the wall
    grid.addBox(0.2,0.8,0,1.6,0.1,1.5)
    
    % add a second wall
    grid.addBox(0.8,0.2,0,0.1,1.6,1.5)

elseif (matches( map_selection, 'low_wall'))

    % add the wall
    grid.addBox(0.2,0.8,0,1.6,0.1,0.75)

elseif (matches( map_selection, 'four_pillars'))

    % pillars
    grid.addBox(1.1,0.6,0,0.2,0.2,0.75)
    grid.addBox(1.1,1,0,0.2,0.2,0.75)
    grid.addBox(0.7,0.6,0,0.2,0.2,0.75)
    grid.addBox(0.7,1,0,0.2,0.2,0.75)

elseif (matches(map_selection, 'roof'))

    % roof
    grid.addBox(0.8,0.8,1.2,0.8,0.8,0.2)



end


%% GET REPULSIVE KERNEL
% -----------------------------------------------------------

% set directional kernels
sigma = 11;
kernel_length = 61;
kernel_width = 9;
kernel_height = kernel_width;

% form kernels
kernel_x = directional_kernel_3d('x', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
kernel_y = directional_kernel_3d('y', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
kernel_z = directional_kernel_3d('z', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;

kernels = {kernel_x, kernel_y, kernel_z};

%% GET REP FIELD VALUES
% -----------------------------------------------------------

[rep_values] = REP_field_calculation(grid, kernels, point)

% generate vectors from values
rep_vectors = eye(3) .* rep_values';

%% DISPLAY REP VECTORS
% -----------------------------------------------------------
REP_field_simulation(grid, point, kernels)
