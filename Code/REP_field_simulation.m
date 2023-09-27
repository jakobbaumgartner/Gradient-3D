function REP_field_simulation(grid, point, kernels)

    % REP_field_simulation: Simulates the REP (Reactive Electromagnetic Power) field and displays it using a graphical representation.
    %
    % Parameters:
    %   grid : The grid structure containing the grid data and methods to display it.
    %   point : A 3-element vector specifying the initial point in the grid (x, y, z).
    %   kernels : A cell array containing the kernels to be used in the REP field calculation.
    %
    % The function initializes by calculating the initial REP field and vectors based on the given point and kernels.
    % It then creates a 3D visualization of the grid and the initial REP vectors at the specified point.
    % The function also adds a slider UI for each coordinate (x, y, z) to allow the user to change the point dynamically.
    % As the point is changed using the sliders, the REP vectors and the total vector field (sum of all vectors) at the new point are updated dynamically in the visualization.
    %
    % The REP field calculation is performed using the REP_field_calculation function which needs to be defined separately.

    % generate initial REP field
    [rep_values] = REP_field_calculation(grid, kernels, point);
    rep_vectors = eye(3) .* rep_values';

    % sum of vector components
    rep_sum = sum(rep_vectors')

    % create figure
    f = figure();

    % display grid
    grid.showGridVol3D(grid.grid, 'floor',false, 'height', true);

    hold on

    % scaling factor
    scale = 100;

    q_list = {};

    % display initial vectors
    for i = 1:length(rep_vectors)
        vector = rep_vectors(:,i);
        q = quiver3(point(1)*grid.resolution, point(2)*grid.resolution, point(3)*grid.resolution, vector(1)*scale, vector(2)*scale, vector(3)*scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
        
        q_list{i} = q;
    end

    % display total vector field
    qs = quiver3(point(1)*grid.resolution, point(2)*grid.resolution, point(3)*grid.resolution, rep_sum(1)*scale, rep_sum(2)*scale, rep_sum(3)*scale, 'g', 'LineWidth', 2, 'MaxHeadSize', 1);


    % plot point at the base of the vector
    p = scatter3(point(1)*grid.resolution, point(2)*grid.resolution, point(3)*grid.resolution, 'MarkerEdgeColor','k', 'MarkerFaceColor','k');
    

    % create sliders
    sx = uicontrol('Style', 'slider', 'Min', 0, 'Max', size(grid.grid, 2)/grid.resolution, 'Value', point(1), 'Units', 'normalized', 'Position', [0.05 0.11 0.4 0.04], 'Callback', @(src, event) update_vector(src, event, grid, kernels, point, q, scale));
    sy = uicontrol('Style', 'slider', 'Min', 0, 'Max', size(grid.grid, 1)/grid.resolution, 'Value', point(2), 'Units', 'normalized', 'Position', [0.05 0.06 0.4 0.04], 'Callback', @(src, event) update_vector(src, event, grid, kernels, point, q, scale));
    sz = uicontrol('Style', 'slider', 'Min', 0, 'Max', size(grid.grid, 3)/grid.resolution, 'Value', point(3), 'Units', 'normalized', 'Position', [0.05 0.01 0.4 0.04], 'Callback', @(src, event) update_vector(src, event, grid, kernels, point, q, scale));

    % create labels
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01 0.11 0.03 0.04], 'String', 'X:', 'HorizontalAlignment', 'right');
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01 0.06 0.03 0.04], 'String', 'Y:', 'HorizontalAlignment', 'right');
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01 0.01 0.03 0.04], 'String', 'Z:', 'HorizontalAlignment', 'right');
    
    hold off


    %% function to update vector when slider is moved
    function update_vector(src, event, grid, kernels, point, q, scale)

        point = [sx.Value sy.Value sz.Value]

        [rep_values] = REP_field_calculation(grid, kernels, point)

        rep_vectors = eye(3) .* rep_values';

        % sum of vector components
        rep_sum = sum(rep_vectors')

        % update arrows
        for j = 1:length(rep_values)
            vector = rep_vectors(:,j);

            q = q_list{j};

            q.XData = point(1)*grid.resolution;
            q.YData = point(2)*grid.resolution;
            q.ZData = point(3)*grid.resolution;
            q.UData = vector(1)*scale;
            q.VData = vector(2)*scale;
            q.WData = vector(3)*scale;
        end

        % update total field arrow
        qs.XData = point(1)*grid.resolution;
        qs.YData = point(2)*grid.resolution;
        qs.ZData = point(3)*grid.resolution;
        qs.UData = rep_sum(1)*scale;
        qs.VData = rep_sum(2)*scale;
        qs.WData = rep_sum(3)*scale;

        % update base point
         p.XData = point(1)*grid.resolution;
         p.YData = point(2)*grid.resolution;
         p.ZData = point(3)*grid.resolution;

    end
end
