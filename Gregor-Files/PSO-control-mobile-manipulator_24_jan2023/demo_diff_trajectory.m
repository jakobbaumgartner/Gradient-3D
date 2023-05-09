%%
% DEMO DIFF PSO WITH TRAJECTORY

% create trajectory data

traj_gen = trajectorys();
[eeTraj, baseTraj, startingState] = traj_gen.trajectoryOrientation(0); % 1 - show traj plot, 0 - don't show traj plot

% run PSO

PSO = PSO_optimizer();


[path_parameters, convergence_success, convergence_times, convergence_runs] = PSO.PSO_optimization_diff_path(eeTraj, baseTraj, startingState)