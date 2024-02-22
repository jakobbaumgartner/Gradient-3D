function [grid] = MAPS(map_selection)

    % MAPS - Generate 3D grids with various map configurations
    %
    % Usage:
    %    [grid] = MAPS(map_selection)
    %
    % Description:
    %    This function generates 3D grids with different map configurations based on the input map_selection.
    %    The available map configurations include 'wall', 'cross', 'low_wall', 'four_pillars', 'roof', and 'corridor'.
    %
    % Inputs:
    %    map_selection - A string specifying the desired map configuration.
    %
    % Outputs:
    %    grid - An OctoGrid object representing the generated 3D grid with the selected map configuration.
    %
    % Dependencies:
    %    This function requires the OctoGrid class to be available in the MATLAB environment.
    
    % create grid
    grid = OctoGrid(2,2,2,10);
    
    if (matches( map_selection , 'wall' ))
    
        % the wall
        grid.addBox(0.2,1.1,0,1.6,0.1,1.5)

    elseif (matches( map_selection, 'column' ))

        % column
        grid.addBox(1.15,1,0,0.2,0.2,1.5)
    
    elseif (matches( map_selection , 'cross' ))
    
        % first wall
        grid.addBox(0.2, 0.8, 0, 1.2, 0.1 ,1.5)
        
        % second wall
        grid.addBox(0.8,0.2,0,0.1,1.6,1.5)
    
    elseif (matches( map_selection, 'low_wall'))
    
        % add the wall
        grid.addBox(0.2,0.8,0,1.6,0.1,0.75)
    
    elseif (matches( map_selection, 'four_pillars'))
    
        % pillars
        grid.addBox(1.1,0.6,0,0.2,0.2,0.75)
        grid.addBox(1.1,1,0,0.2,0.2,0.75)
        grid.addBox(0.7,0.6,0,0.2,0.2,0.75)
        grid.addBox(0.7,1,0,0.2,0.2,0.75)
    
    elseif (matches(map_selection, 'roof'))
    
        % roof
        grid.addBox(1.2,0.8,0.7,0.4,0.8,0.2)
    
    elseif (matches(map_selection, 'corridor'))
    
        % first wall
        grid.addBox(0.2,0.8,0,1.6,0.1,1.5)
    
        % second wall
        grid.addBox(0.2,1.1,0,1.6,0.1,1.5)
    
    end
end