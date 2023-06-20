function [rrt_path, tree] = RRT2D(grid, start_node, goal_node)

tic()

% parameters
fixed_distance = true; 
distance_step = 1.5; % define a fixed distance step
goal_distance = 1;
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
    if (new_node(1) > 2 && new_node(2) > 2 && new_node(1) <= size(grid,1)-1 && new_node(2) <= size(grid,2)-1)

%         new_node
        
        % Check that new branch doesn't intersect with obstacles
%         obstacleCheck = isObstacleFree(grid, tree(index,2), tree(index,3), new_node(1), new_node(2));
        obstacleCheck = sum(sum(grid(ceil(new_node(1))-1:ceil(new_node(1))+1, ceil(new_node(2))-1:ceil(new_node(2))+1))) > 0;

    else

        obstacleCheck = true;

    end
       


    % Save new node to existing tree
    if (~obstacleCheck)

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

function obstacleCheck = isObstacleFree(obstacleGrid, x0, y0, x1, y1)
    dx = abs(x1-x0);
    dy = abs(y1-y0);
    
    if x0 < x1
        sx = 1;
    else
        sx = -1;
    end

    if y0 < y1
        sy = 1;
    else
        sy = -1;
    end

    err = dx - dy;

    while true
        
        % Check if current point is on grid
          if x0 < 1 || y0 < 1 || x0 > size(obstacleGrid,1) || y0 > size(obstacleGrid,2) 
            obstacleCheck = false;
            return;
          end

        % Check if current point is an obstacle
        if obstacleGrid(ceil(x0), ceil(y0)) == 1
            obstacleCheck = false;
            return;
        end

        if x0 == x1 && y0 == y1
            break;
        end

        e2 = 2 * err;
        
        if e2 > -dy
            err = err - dy;
            x0 = x0 + sx;
        end

        if e2 < dx
            err = err + dx;
            y0 = y0 + sy;
        end
    end
    
    obstacleCheck = true;
end
