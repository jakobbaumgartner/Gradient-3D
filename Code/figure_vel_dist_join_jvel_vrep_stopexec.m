% Set the maximum step for the x-axis
Nmax = 50; % You can adjust this value as needed

figure()

%% Subplot 1: distance, velocity, angular_vel, exec_Stop
subplot(2,2,1)

% distance
plot(output.goal_distances*10, 'DisplayName', 'Position error')
title('Primary Task', 'Interpreter', 'latex')
hold on

% orientation
plot(output.goal_orientation, 'DisplayName', 'Orientation error')

% exec_Stop
plot((output.exec_slowdown)*100, 'DisplayName', 'Primary Task Slowdown')
hold off

% Add legend
legend('show', 'Interpreter', 'latex')

% Add labels
xlabel('Step', 'Interpreter', 'latex')
ylabel('Pos. [dm] Orient. [1] Slowd. [\%]', 'Interpreter', 'latex')

% Set x-axis limits
xlim([0 Nmax])

%% Subplot 2: joint velocities (theta dot with index)
subplot(2,2,2)
hold on
for i = 1:size(output.q_velocities, 1)
    plot(output.q_velocities(i, :)', 'DisplayName', ['$\dot{\theta}_{' num2str(i) '}$'])
end
title('$\dot{\theta}$ (Joint Velocities)', 'Interpreter', 'latex')
hold off

% Add legend
legend('Interpreter', 'latex', 'Location', 'east')

% Add labels
xlabel('Step', 'Interpreter', 'latex')
ylabel('$\dot{\theta}$ [rad/s]', 'Interpreter', 'latex')

% Set x-axis limits
xlim([0 Nmax])

%% Subplot 3: joint positions (theta with index)
subplot(2,2,3)
hold on
for i = 1:size(output.joints_positions, 1)
    plot(output.joints_positions(i, :)', 'DisplayName', ['$\theta_{' num2str(i) '}$'])
end
title('$\theta$ (Joint Positions)', 'Interpreter', 'latex')
hold off

% Add legend
legend('Interpreter', 'latex', 'Location', 'east', 'Interpreter', 'latex')

% Add labels
xlabel('Step', 'Interpreter', 'latex')
ylabel('$\theta$ [rad]', 'Interpreter', 'latex')

% Set x-axis limits
xlim([0 Nmax])

%% Subplot 4: repulsive velocities (v_rep with index)
subplot(2,2,4)
% Assuming the cell array is stored in a variable called 'POI_values'
numTimesteps = 50; % Number of timesteps to process
numVectors = 7; % Number of velocity vectors per timestep

% Preallocate a matrix to store the norms for efficiency
velocityNorms = zeros(numTimesteps, numVectors);

for t = 1:numTimesteps
    % Extract the 3x7 matrix for the current timestep
    currentMatrix = output.POI_values{t};
    
    for v = 1:numVectors
        % Extract the velocity vector for the current point/object
        velocityVector = currentMatrix(:, v);
        
        % Calculate the norm (magnitude) of the velocity vector
        velocityNorms(t, v) = norm(velocityVector);
    end
end

% Plotting the repulsive velocities norms with index
hold on
for v = 1:numVectors
    plot(velocityNorms(:, v), 'DisplayName', ['$v_{rep' num2str(v) '}$'])
end
title('$v_{rep}$ Repulsive Velocities', 'Interpreter', 'latex')
hold off

% Add legend
legend('Interpreter', 'latex', 'Location', 'east', 'Interpreter', 'latex')

% Add labels
xlabel('Step', 'Interpreter', 'latex')
ylabel('$v_{rep}$ [1]', 'Interpreter', 'latex')
  
% Set x-axis limits
xlim([0 Nmax])
