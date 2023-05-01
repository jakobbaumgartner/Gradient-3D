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
       
  

        
        

        


end