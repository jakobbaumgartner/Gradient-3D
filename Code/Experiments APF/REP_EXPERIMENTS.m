% REPULSIVE FIELD GENERATION
close all
%% SET POINT
point = [0.7,0.7,1]

%% SELECT MAP
map_selection = 'column'
% -----------------------------------------------------------

[grid] = MAPS(map_selection)

%% GET REPULSIVE KERNEL
% -----------------------------------------------------------

kernels = REP_kernels();

%% DISPLAY REP VECTORS
% -----------------------------------------------------------
REP_field_simulation(grid, point, kernels)
