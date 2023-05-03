function q_avoid = avoidance_base(robot_angles, space_resolution, grid_distance)

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
    phi_error = wrapToPi(phi_error)
    
    % get angular velocity (P - regulator of angle error)
    Kp = -0.6;
    w = Kp * phi_error;

    
    % get linear velocity
    v = sqrt(dx^2+dy^2);
    
    % construct vector of joint velocities
    q_avoid = [v w zeros(1,7)]';

    % limit the array values to between -1 and 1
%     q_avoid(q_avoid < -1) = -1;
%     q_avoid(q_avoid > 1) = 1;

end