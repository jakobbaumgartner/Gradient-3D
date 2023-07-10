function [value, transform] = getPartialAPFValueRobotTreePoint(APF, resolution, robot_tree, Tbase, joints)

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