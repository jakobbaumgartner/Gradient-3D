% REPULSIVE FIELD GENERATION
close all
%% SET POINT
point = [0.7,0.7,1]

%% SELECT MAP
map_selection = 'corridor'
% -----------------------------------------------------------

[grid] = MAPS(map_selection)

%% GET REPULSIVE KERNEL
% -----------------------------------------------------------

kernels = REP_kernels();

%% GET REP FIELD VALUES
% -----------------------------------------------------------

[rep_values] = REP_field_calculation(grid, kernels, point)

% generate vectors from values
rep_vectors = eye(3) .* rep_values';

%% DISPLAY REP VECTORS
% -----------------------------------------------------------
REP_field_simulation(grid, point, kernels)
