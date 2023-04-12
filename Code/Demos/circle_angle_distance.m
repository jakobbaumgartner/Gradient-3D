function [rotation_angle, distance] = circle_angle_distance(center, location, orientation)
    % ROBOT_TURN calculates the angle the robot needs to rotate to face
    % perpendicularly away from the center of a circle and the distance from
    % the center to the robot.
    %
    % Inputs:
    %   center: a 2-element vector representing the x and y coordinates of
    %           the center of the circle.
    %   location: a 2-element vector representing the x and y coordinates of
    %             the robot's location.
    %   orientation: a scalar representing the robot's orientation angle in
    %                radians.
    %
    % Outputs:
    %   rotation_angle: the angle in radians that the robot needs to rotate
    %                   to face perpendicularly away from the center of the
    %                   circle.
    %   distance: the distance between the center of the circle and the
    %             robot's location.

    % Calculate the vector from the center to the robot location
    vec = location - center;
    % Calculate the distance from the center to the robot
    distance = norm(vec);
    % Calculate the angle between the vector and the x-axis
    angle = atan2(vec(2), vec(1));

    % Calculate the angle the robot needs to rotate to face away from the center
    rotation_angle = angle - orientation;

    % Wrap rotation_angle to [-pi, pi]
    rotation_angle = wrapToPi(rotation_angle);

end