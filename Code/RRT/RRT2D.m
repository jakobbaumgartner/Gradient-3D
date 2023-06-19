function [rrt_path, tree] = RRT2D(grid, start_node, goal_node)

tic()

% parameters
fixed_distance = true; 
distance_step = 2; % define a fixed distance step
goal_distance = 2;
max_branches = 10000; % max number of branches

% grid dimensions
search_size = size(grid);

% Tree
% Every node contatins its coordinates and parent node
% |PARENT_ID|COORDINATES|
tree = [0 start_node];

% Path
rrt_path = [];


branch_count = 0;

% RRT loop runs until interrupted
while true
    % Pick a random seed point 
    seed = rand(1,2) .* search_size;

    % Get nearest graph point (euclidian distance)
    [dist, index] = min(sum((tree(:,2:3)-seed)'.^2)');

    % Generate new node
    if fixed_distance

        % calculate direction vector from nearest node to seed point
        direction = seed - tree(index, 2:3);
        direction = direction / norm(direction); % normalize direction vector
        
        % new node is a fixed distance from nearest node in direction of seed
        new_node = tree(index, 2:3) + direction * distance_step;

    else

    end

    % Check that new node is on grid
    if (new_node(1) > 1 && new_node(2) > 1 && new_node(1) <= size(grid,1) && new_node(2) <= size(grid,2))

%         new_node
        
        % Check that new branch doesn't intersect with obstacles
        intersects = checkObstacleIntersection(grid, tree(index, 2:3), new_node);

    else

        intersects = true;

    end
       


    % Save new node to existing tree
    if (~intersects)

        % Save node
        tree = [tree ; index new_node];

        % Update branches count
        branch_count = branch_count + 1;
    end

    % Check if goal reached
    if (norm(goal_node-new_node) < goal_distance)

        % Extract path
        rrt_path = [goal_node];

        while index > 0

            rrt_path = [tree(index, 2:3) ; rrt_path];
            index = tree(index,1);

        end

        % print results summary
        display("PATH FOUND. Nodes num.: " + length(rrt_path) + " Time: " + toc())

        break;
    
    end

    % Check if max number of branches 
    if branch_count >= max_branches

        % print results summary
        display("PATH NOT FOUND. Time: " + toc())

        break;
    end


end

end

function obstacleDetected  = checkObstacleIntersection(grid, start_node, end_node)

    


end