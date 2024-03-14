figure()

%% distance,velocity, angular_vel, exec_Stop, avoidance_total
subplot(2,2,1)

% distance

% velocity


% angular_vel


% exec_Stop

% avoidance_total


%% joint velocities
subplot(2,2,2)

plot((output.q_velocities)')


%% joint positions

subplot(2,2,3)

plot((output.joints_positions)')



%% repulsive velocities

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

% Now, velocityNorms contains the norms of the velocity vectors for each object at each timestep
disp(velocityNorms);


plot(velocityNorms)

