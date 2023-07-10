function [outputArg1,outputArg2] = APF_trajectory(inputArg1,inputArg2)

    % parameters
    % ---------------------------------------------------------
    num_of_segments = 7;
    points_per_segment = [1 1 1 1 1 1 1];
   
    % this is a classic (Spong?) DH convention parameters table, on official Franka
    % Emika site there is a different (Craig) convention (that is a pain in the
    % arse)
    % | a | alpha | d | theta-0 (ignored) |
    dhparams = [0 -pi/2 0.333 0 ;
                0 pi/2 0 0 ;
                0.0825 pi/2 0.316 0 ;
                -0.0825 -pi/2 0 0 ;
                0 pi/2 0.384 0 ;
                0.088 pi/2 0 0 ;
                0 0 0.107 0];
    % ---------------------------------------------------------

    % list of transformations
    transformations_list = {};

    % current tree
    robot_tree = rigidBodyTree;

    % previous body name
    previous_body = 'base'


    % calculate positions in 3D space for a number of points on a robot frame using geometric transformations
    for i = 1:1:num_of_segments % for every segment

        % for a number of points on segments
%         for j = 1:1:points_per_segment(i)

            % new attaching joint
            new_joint = rigidBodyJoint('jnt'+string(i),'revolute');
    
            % create new segment
            new_segment = rigidBody('body'+string(i));
    
            % set parameters
            segment_param = dhparams(i,:);          % change this to get more segments
    
            % set transformation from last segment
            setFixedTransform(new_joint, segment_param,'dh');
    
            % attach joint to body
            new_segment.Joint = new_joint;
    
            % add body to the tree
            addBody(robot_tree,new_segment,previous_body);

            % save current fixing body name
            previous_body = 'body'+string(i)


%         end
    end

    % get APF value in every of calculated 3D points
    
    % if A - closest point only

        % find in which point APF is the highest

        % calculate Jacobian in that point

        % if there was a previously different point with highest APF calculate Jacobian in this previous point

        % calculate weighting coefficients 

        % calculate avoidance joints velocities

    
    % if B - all points on the robot

        % calculate APF in every point

        % calculate Jacobian in every point

        % calculate weighting coefficients

        % calculate avoidance joints velocities




end