function REP_field_display(grid, point, vectors_list)

    figure()

    % scaling factor
    scale = 100;
    
    % display grid
    grid.showGridVol3D(grid.grid,'floor',true,'height',false);

    hold on


    % display vectors
    for i = 1:length(vectors_list)
      vector = -vectors_list(:,i);
      quiver3(point(1)*grid.resolution, point(2)*grid.resolution, point(3)*grid.resolution, vector(1)*scale, vector(2)*scale, vector(3)*scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 1)

    end

    hold off



end