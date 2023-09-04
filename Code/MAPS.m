function [grid] = MAPS(map_selection)
% create grid
grid = OctoGrid(2,2,2,100);

if (matches( map_selection , 'wall' ))

    % the wall
    grid.addBox(0.2,0.8,0,1.6,0.1,1.5)

elseif (matches( map_selection , 'cross' ))

    % first wall
    grid.addBox(0.2,0.8,0,1.6,0.1,1.5)
    
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
    grid.addBox(0.8,0.8,1.2,0.8,0.8,0.2)

elseif (matches(map_selection, 'corridor'))

    % first wall
    grid.addBox(0.2,0.8,0,1.6,0.1,1.5)

    % second wall
    grid.addBox(0.2,1.2,0,1.6,0.1,1.5)

end
end