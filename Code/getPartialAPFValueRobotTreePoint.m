function [value, transform] = getPartialAPFValueRobotTreePoint(APF, resolution, robot_tree, Tbase, joints)

    % Function: getPartialAPFValueRobotTreePoint
    %
    % Description:
    %   This function calculates the value and transform for a given point in the
    %   robot tree using the attractive potential field (APF) method. It takes the
    %   APF, resolution, robot tree, base transformation, and joint values as inputs.
    %
    % Input:
    %   - APF: The attractive potential field.
    %   - resolution: The resolution for interpolating APF values.
    %   - robot_tree: The robot tree structure.
    %   - Tbase: The base transformation.
    %   - joints: The joint values for the robot tree.
    %
    % Output:
    %   - value: The interpolated APF value at the given point.
    %   - transform: The transform for the given point in the robot tree.
    %
    % Usage:
    %   [value, transform] = getPartialAPFValueRobotTreePoint(APF, resolution,
    
    
    % set configuration to a robot tree
    config = homeConfiguration(robot_tree); % Create an empty struct
    
    % Assign values to joints, there will be 1-7 joints for Panda
    for i = 1:1:robot_tree.NumBodies
        config(i).JointPosition  = joints(i); % set i-th joint
    end

    % get transform
    transform = getTransform(robot_tree,config,'body'+string(robot_tree.NumBodies));

    % add base transformation
    transform = Tbase * transform;

    % get x,y,z values
    xyz = transform(1:3,4);

    % interpolate APF value
    value = interpolate_points(xyz, APF, resolution);


end