function [grid_attractive] = attractive_field_3D(grid, goal_position, scale, varargin)

    % Function: attractive_field_3D
    %
    % Description:
    % This function calculates the attractive field in a 3D grid based on a
    % goal position. The attractive field represents the strength of
    % attraction towards the goal at each point in the grid.
    %
    % Input:
    % - grid: 3D grid matrix
    % - goal_position: 3-element vector representing the goal position
    % - scale: scaling factor for the grid coordinates
    % - (optional) Name-Value pairs:
    % - K: scaling factor for the attractive field (default: 1)
    %
    % Output:
    % - grid_attractive: 3D grid matrix representing the attractive field
    %
    % Usage:
    % grid_attractive = attractive_field_3D(grid, goal_position, scale)
    % grid_attractive = attractive_field_3D(grid, goal_position, scale, 'K', 2)

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