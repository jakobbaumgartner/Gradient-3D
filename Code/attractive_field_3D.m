function [grid_attractive] = attractive_field_3D(grid, goal_position, scale, varargin)

    % Create an input parser
    parser = inputParser;
    parser.CaseSensitive = false;

    % Define optional parameter: K
    addOptional(parser, 'K', 1, @(x) validateattributes(x, {'numeric'}, {'scalar'}));

    % Parse the input arguments
    parse(parser, varargin{:});

    % Extract values from the input parser
    K = parser.Results.K;
    
    % Get dimensions of the grid
    [x, y, z] = size(grid); 
        
    % Initialize the attractive field grid
    grid_attractive = zeros(x, y, z);
    
    % Calculate the attractive field for each point in the grid
    for i = 1:x
        for j = 1:y
            for k = 1:z
                distance = norm([i, j, k] / scale - goal_position); % Calculate Euclidean distance to the goal

                % Linear function for attractive field
                grid_attractive(i, j, k) = K * distance;
            end
        end
    end
end