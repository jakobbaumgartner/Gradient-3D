function q_avoid = avoidance_base(robot_angles, space_resolution, grid_distance, Kp)

    % avoidance_base - function that calculates the joint velocities for a robot
    % to avoid obstacles.
    %
    % Inputs:
    %   robot_angles - 1x9 vector of the robot's current position 
    %   [ x y phi fi1 ... fi7]
    %
    %   space_resolution - scalar value of the resolution of the gradient
    %   grid
    %
    %   grid_distance - matrix of the distance density from nearby obstacles
    %
    %   Kp - P constant for angular velocity regulator
    %
    % Outputs:
    %   q_avoid - 8x1 vector of joint velocities

    % base point pose
    x = robot_angles(1);
    y = robot_angles(2);
    z = 0.2; % set to 20 cm above ground
    phi_base = robot_angles(3);
    
    % get gradient field vector
    [dx, dy, dz] = interpolate_derivative([x,y,z], grid_distance, space_resolution);
    
    % get (negative) gradient field vector orientation
    phi_grad = atan2(dx, dy);
    
    % get angle between gradient field vector and base orientation
    phi_error = phi_grad - phi_base;

    phi_error = atan2(sin(phi_error), cos(phi_error)); % constrain error to [-pi, pi]
    phi_error = wrapToPi(phi_error);
    
    % get angular velocity (P - regulator of angle error)
    w = Kp * phi_error;

    
    % get linear velocity
    v = sqrt(dx^2+dy^2);
    
    % construct vector of joint velocities
    q_avoid = [0 w zeros(1,7)]';

    % limit the array values to between -1 and 1
%     q_avoid(q_avoid < -1) = -1;
%     q_avoid(q_avoid > 1) = 1;

end