function REP_field_simulation(grid, point, kernels)

    % generate initial REP field
    [rep_values] = REP_field_calculation(grid, kernels, point);
    rep_vectors = eye(3) .* rep_values';

    % create figure
    f = figure();

    % display grid
    grid.showGridVol3D(grid.grid,'floor',true,'height',false);

    hold on

    % scaling factor
    scale = 100;

    q_list = {};

    % display initial vectors
    for i = 1:length(rep_vectors)
        vector = -rep_vectors(:,i);
        q = quiver3(point(1)*grid.resolution, point(2)*grid.resolution, point(3)*grid.resolution, vector(1)*scale, vector(2)*scale, vector(3)*scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
        
        q_list{i} = q;
    end

    

    % create sliders
    sx = uicontrol('Style', 'slider', 'Min', 0, 'Max', size(grid.grid, 2)/grid.resolution, 'Value', point(1), 'Units', 'normalized', 'Position', [0.1 0.01 0.8 0.05], 'Callback', @(src, event) update_vector(src, event, grid, kernels, point, q, scale));
    sy = uicontrol('Style', 'slider', 'Min', 0, 'Max', size(grid.grid, 1)/grid.resolution, 'Value', point(2), 'Units', 'normalized', 'Position', [0.1 0.06 0.8 0.05], 'Callback', @(src, event) update_vector(src, event, grid, kernels, point, q, scale));
    sz = uicontrol('Style', 'slider', 'Min', 0, 'Max', size(grid.grid, 3)/grid.resolution, 'Value', point(3), 'Units', 'normalized', 'Position', [0.1 0.11 0.8 0.05], 'Callback', @(src, event) update_vector(src, event, grid, kernels, point, q, scale));

    hold off

    % function to update vector when slider is moved
    function update_vector(src, event, grid, kernels, point, q, scale)
        point = [sx.Value sy.Value sz.Value];


        point
        [rep_values] = REP_field_calculation(grid, kernels, point)

        rep_vectors = eye(3) .* rep_values';

        for j = 1:length(rep_values)
            vector = -rep_vectors(:,j);

            q = q_list{j};

            q.XData = point(1)*grid.resolution;
            q.YData = point(2)*grid.resolution;
            q.ZData = point(3)*grid.resolution;
            q.UData = vector(1)*scale;
            q.VData = vector(2)*scale;
            q.WData = vector(3)*scale;
        end
    end
end
