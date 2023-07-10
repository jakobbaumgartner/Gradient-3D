function transformations_list = APF_trajectory()

    % parameters
    % ---------------------------------------------------------
    num_of_segments = 7;
    points_per_segment = 5*[1 1 1 1 1 1 1];
   
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
    transformations_list = [];

    % current tree
    robot_tree = rigidBodyTree;

    % previous body name
    previous_body = 'base'


    % calculate positions in 3D space for a number of points on a robot frame using geometric transformations
    for i = 1:1:num_of_segments % for every segment

        % for a number of points on segments
        for j = 1:1:points_per_segment(i)

            % new attaching joint
            new_joint = rigidBodyJoint('jnt'+string(i),'revolute');
    
            % create new segment
            new_segment = rigidBody('body'+string(i));
    
            % set parameters
            segment_param = [dhparams(i,1) * j / points_per_segment(i) , dhparams(i,2) , dhparams(i,3) * j / points_per_segment(i) , dhparams(i,4)] ;          % change this to get more segments
    
            % set transformation from last segment
            setFixedTransform(new_joint, segment_param,'dh');
    
            % attach joint to body
            new_segment.Joint = new_joint;

            
            if j > 1 % if this is not the first partial segment replace old segment
                replaceBody(robot_tree, previous_body, new_segment)
                
            else % if it is first partial form of segment, add new segment
                % add body to the tree
                addBody(robot_tree, new_segment, previous_body);
                

            end

            % save current fixing body name
            previous_body = 'body' + string(i)

            % save configuration
            save_conf = {};
            save_conf.tree = copy(robot_tree);
            save_conf.segm_num = i;
            save_conf.segm_part = j;
            transformations_list = [transformations_list save_conf ];


        end
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