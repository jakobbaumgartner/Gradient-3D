function q_avoid = trajectory_base(robot_angles, space_resolution, base_goal)

    % base point pose
    x = robot_angles(1);
    y = robot_angles(2);
    phi_base = robot_angles(3);
    
    % get base position errors
    dx = base_goal(1)-x;
    dy = base_goal(2)-y;
    
    % get goal orientation
    phi_grad = atan2(dy, dx);
%     
    % get angle between goal vector and base orientation
    phi_error = phi_grad - phi_base;

    phi_error = atan2(sin(phi_error), cos(phi_error)); % constrain error to [-pi, pi]
    phi_error = wrapToPi(phi_error);
    
    % get angular velocity (P - regulator of angle error)
    w = phi_error;

    
    % get linear velocity
    v = sqrt(dx^2+dy^2);
    
    % construct vector of joint velocities
    q_avoid = [v w zeros(1,7)]'


end