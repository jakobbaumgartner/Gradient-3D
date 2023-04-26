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
    dd=0.005 * 10; % cca 5mm
    df=0.01 * 10; % ccca 0.5 stopinje
    delta=[dd df df df df df df df df]; % [v w dq1 ... dq7]
    dq_zeros=zeros(9,1);

    % initialize avoidance velocities vector
    q_avoid = zeros(9,1);


    % BASE ONLY
       
    % for every joint
    for i = 1:1:2

            % pick specific (each) joint positive or negative velocities
            q_vel1 = dq_zeros;
            q_vel1(i)=delta(i);

            q_vel2 = dq_zeros;
            q_vel2(i) = - delta(i);

            % patch
            if (i == 2)
                q_vel1(1) = delta(1);
                q_vel2(1) = delta(1);
            end


            % simulate robot changes with these velocities
            rob_state1 = simulate_step(robot_angles, q_vel1);
            rob_state2 = simulate_step(robot_angles, q_vel2);

            % calculate positions of base point after changes
            [~, Abase] = GeometricRobot(robot_angles);
            point_base = Abase(1:3,4);
            [~, Abase1] = GeometricRobot(rob_state1);
            point_base1 = Abase1(1:3,4);
            [~, Abase2] = GeometricRobot(rob_state2);
            point_base2 = Abase2(1:3,4);

            points = [point_base point_base1  point_base2];

            % check if robot point is on grid
            if (min(point_base) > 0)

                % use interpolation to get distance field values in points
                [values] = interpolate_points(points, grid_distance, space_resolution);
    
                % calculate numerial derivatives
    
                % if we are crossing cell border use mid and moved points for
                % interpolation, otherwise use both moved points
                if(~isnan(values(2)) && ~isnan(values(3)))
    
                    q_avoid(i) = (values(3)-values(2))/(2*delta(i));
    
                elseif (~isnan(values(1)) && ~isnan(values(2)))
    
                    q_avoid(i) = (values(1)-values(2))/(delta(i));
    
    
                elseif (~isnan(values(1)) && ~isnan(values(3)))
    
                    q_avoid(i) = (values(3)-values(1))/(delta(i));
    
                else
                    display("ERROR IN INTERPOLATION.")
                end

            end

     


    end

        
        

        


end