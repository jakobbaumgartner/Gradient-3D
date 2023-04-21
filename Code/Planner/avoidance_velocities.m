function q_avoid = avoidance_velocities(robot_angles, space_resolution, grid_distance)

    % avoidance_velocities - calculates the avoidance velocities for a robot
    %
    % Inputs:
    %   robot_angles - vector of robot joint angles 
    %                  [x_base y_base phi_base f1 ... fi7]
    %
    %   space_resolution - resolution of the obstacle distance-density space grid
    %   grid_distance - 3D matrix representing the distance-density grid
    %
    % Outputs:
    %   q_avoid - avoidance velocities for the robot


    % set numerical step size
    dd=0.005; % cca 5mm
    df=0.01; % ccca 0.5 stopinje
    delta=[dd df df df df df df df df]; % [v w dq1 ... dq7]
    dq_zeros=zeros(1,9);

    % BASE ONLY
   
    
    % for every joint
    for i = 1:1:2

            % pick specific joint positive or negative velocities
            q_vel1 = dq_zeros;
            q_vel1(i)=delta(i);

            q_vel2 = dq_zeros;
            q_vel2(i) = - delta(i);

            % simulate robot changes with these velocities
            rob_state1 = simulate_step(robot_angles, q_vel1);
            rob_state2 = simulate_step(robot_angles, q_vel2);

            % calculate positions of base point after changes
            [~, Abase1] = GeometricRobot(rob_state1);
            point_base1 = Abase1(1:3,4);
            [~, Abase2] = GeometricRobot(rob_state2);
            point_base2 = Abase2(1:3,4);





        
        

        


    end



end