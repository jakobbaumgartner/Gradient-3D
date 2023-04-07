%% ONE POINT
% -------------------------------------------------------------------------------------------

% Define maksimal distance from goal at which optimization stops
max_dist = 0.01;

% Define goal position for the robot to reach
goal = [130 73 100 0 0 0]/100;

% Define initial robot states
robot_states = [1.50 0 pi/2 0 pi/4 0 -pi/3 0 1.8675 0];

% Create a history of robot states
robot_states_hist = [robot_states];

% Create a history of robot end effector positions
robot_ee_positions = [];

% Create a history of robot the differences between goal and robot position
diff_hist = [];
norm_diff_hist = [];

% Initiate diff value to 10 (so that loop runs)
diff = 10;

% Create a loop that will continue until small enough distance is reached
while norm(diff) > max_dist

   % Calculate the transformation matrix for the robot's current position
   [T] = GeometricRobot(robot_states);

   % Save EE positions
   robot_ee_positions = [robot_ee_positions ; T(1:3,4)'];

   % Calculate the difference between the goal and the robot's current position
   diff = [goal - [T(1:3,4)' 0 0 0]];

   % Use an optimizer to determine the optimal velocities for the robot
   [q_vel] = optimizer(robot_states, diff, []);

   % Simulate a step for the robot using the optimal velocities
   [robot_states] = simulate_step(robot_states, q_vel);

   % Add the current robot state to the history list
   robot_states_hist = [robot_states_hist; robot_states];

   % Add the current difference to the difference history list
   diff_hist = [diff_hist; diff];
   norm_diff_hist = [norm_diff_hist ; norm(diff)];

end

% Plot the end-effector positions of the robot
figure()
plot3(robot_ee_positions(:,1)*100,robot_ee_positions(:,2)*100,robot_ee_positions(:,3)*100)
hold on

% Use the GeometricRobot function to calculate the transformation matrices for
% different time steps in the robot's movement, and display the robot at those
% time steps using the showRobot function
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(1,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(5,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(10,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(15,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(20,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(25,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)
[T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_states_hist(30,:));
showRobot(Abase, A01, A12, A23, A34, A45, A56, A67,10*10, "#A2142F", true)

% Plot the robot's states over time (to see joints movement)
figure()
plot(robot_states_hist)




% -------------------------------------------------------------------------------------------