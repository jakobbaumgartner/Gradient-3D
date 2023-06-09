function transformations_list = getPartialTransformationsListPanda(varargin)

    % transformations_list = getPartialTransformationsListPanda(varargin) 
    %
    % Function returns an array of struct data. Every struct represents a
    % MATLAB Robot Toolbox robot tree representation.
    %
    % INPUT:
    % optional: 
    %   num of points per robot body segment: ('points_per_segment', 5*[1 1 1 1 1 1 1])
    %
    % OUTPUT:
    % array of structs: 
    %  [{tree, segm_num, segm_part}, {...},{...},{...} ...]
    %       -> tree is Robot Toolbox tree
    %       -> segm_num is a number of a segment transformed point is on (1-7)
    %       -> segm_part is a number of a point on a current segment

    % Parse the optional arguments
    p = inputParser;
    addOptional(p, 'points_per_segment', 5*[1 1 1 1 1 1 1]); % Default value is 5*[1 1 1 1 1 1 1]
    parse(p, varargin{:});
    
    points_per_segment = p.Results.points_per_segment;

    % PARAMETERS
    % ---------------------------------------------------------
    num_of_segments = 7;
   
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


    % TRANSFORMATIONS
    % ---------------------------------------------------------

    % list of transformations
    transformations_list = [];

    % current tree
    robot_tree = rigidBodyTree;

    % previous body name
    previous_body = 'base';

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
                addBody(robot_tree, new_segment, previous_body); % add body to the tree
            end

            % save current fixing body name
            previous_body = 'body' + string(i);

            % save configuration
            save_conf = {};
            save_conf.tree = copy(robot_tree);
            save_conf.segm_num = i;
            save_conf.segm_part = j;
            transformations_list = [transformations_list save_conf ];

        end
    end

end