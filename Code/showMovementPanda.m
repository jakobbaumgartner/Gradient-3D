function [f] = showMovementPanda(grid, grid_repulsive, control_points, joints_positions_APF, EE_positions_APF, values_APF, Tbase)

    % set resolution
    scale_view = grid.resolution;
    
    % create a figure
    f = figure;

    % move figure to external window
    set(f, 'WindowStyle', 'normal');  % Change 'docked' to 'normal' to move to external window
    
    % add slider to the bottom of the figure
    s = uicontrol('Style', 'slider',...
        'Min',1,'Max',100,'Value',100,...
        'Position', [150 20 300 20],...
        'Callback', @sliderCallback); 

        % Add checkbox for APF vectors
    cv = uicontrol('Style', 'checkbox',...
        'String', 'APF Field',...
        'Position', [20 20 100 20],...
        'Value', 1,...
        'Callback', @checkboxCallbackAPF); 

    % Add checkbox for Obstacles grid
    co = uicontrol('Style', 'checkbox',...
        'String', 'Obstacle field',...
        'Position', [20 40 100 20],...
        'Value', 1,...
        'Callback', @checkboxCallbackObstacles); 

    % Add checkbox for Repulsive field
    cr = uicontrol('Style', 'checkbox',...
        'String', 'Repulsive field',...
        'Position', [20 60 100 20],...
        'Value', 0,...
        'Callback', @checkboxCallbackRepulsive); 

    % On/off APF vectors
    show_arrows = 1;
    
    % Repulsive field handle
    HRepulsive = [];
    
    % display grid
    HObstacles = grid.showGridVol3D(grid.grid,'floor',true,'height',true);
    hold on
    axis equal

    HRepulsive = [];

    % display control points
    for i = 1:1:size(control_points,1)
        scatter3(control_points(i,1)*scale_view,control_points(i,2)*scale_view,control_points(i,3)*scale_view,'r')
    end
    
    % draw EE trajectory
    plot3(EE_positions_APF(1,:)*grid.resolution,EE_positions_APF(2,:)*grid.resolution,EE_positions_APF(3,:)*grid.resolution,'red')
    
    % draw starting pose
    j = size(joints_positions_APF,2); 
    
    % calculate transforms from joint positions
    [transforms] = GeometricPandaMATLAB(joints_positions_APF(:,j), Tbase);
    
    hold on;
    
    % figure handles
    points_handles = gobjects(length(transforms),1);
    lines_handles = gobjects(length(transforms),1);
    APF_handles = gobjects(100,1);
    
    for i = 1:length(transforms)
    %         % Plot the transformation frame with thicker lines
    %         plotTransforms(transforms(1:3, 4, i)'*scale_view, rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);
    
        % Plot the joint point
        points_handles(i) = plot3(transforms(1, 4, i)*scale_view, transforms(2, 4, i)*scale_view, transforms(3, 4, i)*scale_view, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [38/255, 151/255, 224/255], 'MarkerEdgeColor', [38/255, 151/255, 224/255]);
    
        % Connect the joint points with thicker lines
        if i > 1
            % Get the previous and current joint positions
            prevPos = transforms(1:3, 4, i-1);
            currPos = transforms(1:3, 4, i);
    
            % Plot a line between the joint positions with thicker lines
            lines_handles(i) = line([prevPos(1), currPos(1)]*scale_view, [prevPos(2), currPos(2)]*scale_view, [prevPos(3), currPos(3)]*scale_view, 'Color', 'k', 'LineWidth', 2);
        end
    end
    
    % set display properties
    axis equal
    view([-180.9 45.0])
    hold off;


    % draw APF field
    % --------------------------------------------------------------
    arrow_length = 50; % adjust the length to your preference

    j = size(values_APF,2);
    for i = 1:1:size(values_APF(1).xyz,1)        
        % plot the arrow
        hold on
        APF_handles(i) = quiver3(values_APF(j).xyz(i,1)*grid.resolution, values_APF(j).xyz(i,2)*grid.resolution, values_APF(j).xyz(i,3)*grid.resolution, values_APF(j).grad(i,1) * arrow_length, values_APF(j).grad(i,2) * arrow_length, values_APF(j).grad(i,3) * arrow_length, arrow_length/2, 'LineWidth', 2, 'MaxHeadSize', 1);
    end

    % --------------------------------------------------------------


    % Set the limits
    xlim([0 200]);   % X-axis limits
    ylim([0 200]);   % Y-axis limits
    zlim([-50 200]);    % Z-axis limits
    axis equal


    % function to changed plot time
    function upPlot(sliderValue)
  
        % get figure time from slider
        j = round(sliderValue * size(joints_positions_APF,2) / 100);

        % calculate transforms from joint positions
        [transforms] = GeometricPandaMATLAB(joints_positions_APF(:,j), Tbase);
    
        for k = 1:length(transforms)
    %         % Plot the transformation frame with thicker lines
    %         plotTransforms(transforms(1:3, 4, i)'*scale_view, rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);
        
            % Plot the joint point
            points_handles(k).XData = transforms(1, 4, k)*scale_view;
            points_handles(k).YData = transforms(2, 4, k)*scale_view;
            points_handles(k).ZData = transforms(3, 4, k)*scale_view;

            % Connect the joint points with thicker lines
            if k > 1
                % Get the previous and current joint positions
                prevPos = transforms(1:3, 4, k-1);
                currPos = transforms(1:3, 4, k);
        
                % Plot a line between the joint positions with thicker lines
                lines_handles(k).XData = [prevPos(1), currPos(1)]*scale_view;
                lines_handles(k).YData = [prevPos(2), currPos(2)]*scale_view;
                lines_handles(k).ZData = [prevPos(3), currPos(3)]*scale_view;
            end
        end

            % draw APF field
    % --------------------------------------------------------------
    arrow_length = 50; % adjust the length to your preference


       
        % update quiver arrows
        for i = 1:1:size(values_APF(j).xyz,1)
            % plot the arrow
            hold on
       
            APF_handles(i).XData = values_APF(j).xyz(i,1)*grid.resolution;
            APF_handles(i).YData = values_APF(j).xyz(i,2)*grid.resolution;
            APF_handles(i).ZData = values_APF(j).xyz(i,3)*grid.resolution;
            APF_handles(i).WData = values_APF(j).grad(i,1) * arrow_length * show_arrows;
            APF_handles(i).VData = values_APF(j).grad(i,2) * arrow_length * show_arrows;
            APF_handles(i).UData = values_APF(j).grad(i,3) * arrow_length * show_arrows;
        
        
        
        end
    % --------------------------------------------------------------

    end

    function setAPFArrows(checkboxValue)
            % Check if the checkbox is checked
        if checkboxValue == 1
            show_arrows = 1;
            % Execute some code here
        else
            show_arrows = 0;
            % Execute some code here
        end    
    
    end

    function setObstaclesGrid(checkboxValue)
        % Check if the checkbox is checked
        if checkboxValue == 1
            % display grid
            HObstacles = grid.showGridVol3D(grid.grid,'floor',false,'height',true);
        else
            for i = 1:1:length(HObstacles.handles)
                delete(HObstacles.handles(i))
            end
        end    
    
    end

    function setRepulsiveGrid(checkboxValue)
        % Check if the checkbox is checked
        if checkboxValue == 1
            % display grid
            HRepulsive = grid.showGridVol3D(grid_repulsive,'floor',false,'height',false);
        else
            for i = 1:1:length(HRepulsive.handles)
                delete(HRepulsive.handles(i))
            end
        end    
    
    end

    function sliderCallback(source,event)
        % This function will be called whenever the slider's value is changed.
        % You can add the code here that will be executed whenever the slider is moved.
        % For example, this might involve redrawing the robot with different joint positions based on the slider's current value.
        sliderValue = source.Value;  % Get the current value of the slider
    
        upPlot(sliderValue)
    
    end

    function checkboxCallbackAPF(source,event)
        % This function will be called whenever the checkbox's state is changed.
        % You can add the code here that will be executed whenever the checkbox is checked or unchecked.
        checkboxValue = source.Value;  % Get the current state of the checkbox

        setAPFArrows(checkboxValue)
    end

    function checkboxCallbackObstacles(source,event)
        % This function will be called whenever the checkbox's state is changed.
        % You can add the code here that will be executed whenever the checkbox is checked or unchecked.
        checkboxValue = source.Value;  % Get the current state of the checkbox

        setObstaclesGrid(checkboxValue)

    end

    function checkboxCallbackRepulsive(source,event)
        % This function will be called whenever the checkbox's state is changed.
        % You can add the code here that will be executed whenever the checkbox is checked or unchecked.
        checkboxValue = source.Value;  % Get the current state of the checkbox

        setRepulsiveGrid(checkboxValue)

    end

end

