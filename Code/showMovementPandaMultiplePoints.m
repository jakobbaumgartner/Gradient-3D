function [f] = showMovementPandaMultiplePoints(grid, Tbase, control_points, output)

    % APF field vector length
    arrow_length = 5; % adjust the length to your preference

    % show floor
    floor = false

    %% COLORMAP
    % ---------------------------------------------------------------
    cMap = interp1(0:1,[0 1 0; 1 0 0],linspace(0,1,256));
    cMap = cMap.^(1/2.4); % linearize

    %% FIGURE AND CONTROLS
    % ---------------------------------------------------------------

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

    % Create a "Play" button
    play_button = uicontrol('Style', 'pushbutton', 'String', 'Play', ...
        'Position', [460, 20, 60, 20], 'Callback', @playButtonCallback);


    % On/off APF vectors
    show_arrows = 1;
    
    % display grid
    HObstacles = grid.showGridVol3D(grid.grid,'floor',floor,'height',true);
    hold on
    axis equal

    %% CONTROL POINTS AND TRAJECTORY
    % ---------------------------------------------------------------

    % display goal points 
    for i = 1:1:size(control_points,1)
        scatter3(control_points(i,1)*grid.resolution,control_points(i,2)*grid.resolution,control_points(i,3)*grid.resolution,'r')
    end

    % draw EE trajectory
    plot3(output.EE_positions(1,:)*grid.resolution,output.EE_positions(2,:)*grid.resolution,output.EE_positions(3,:)*grid.resolution,'red')

    % draw starting pose of the robot
    j = size(output.joints_positions,2); 
    
    % calculate transforms from joint positions
    [transforms] = GeometricPandaMATLAB(output.joints_positions(:,j), Tbase);
    
    hold on;
    
    % figure handles
    points_handles = gobjects(length(transforms),1);
    lines_handles = gobjects(length(transforms),1);
    field_vectors_handles = gobjects(size(output.POI_values{1},2),1);
    field_poi_dots_handles = gobjects(size(output.POI_values{1},2),1);


    
    for i = 1:length(transforms)
        % Plot the transformation frame with thicker lines
%         plotTransforms(transforms(1:3, 4, i)'*grid.resolution, rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);
    
        % Plot the joint point
        points_handles(i) = plot3(transforms(1, 4, i)*grid.resolution, transforms(2, 4, i)*grid.resolution, transforms(3, 4, i)*grid.resolution, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [38/255, 151/255, 224/255], 'MarkerEdgeColor', [38/255, 151/255, 224/255]);
    
        % Connect the joint points with thicker lines
        if i > 1
            % Get the previous and current joint positions
            prevPos = transforms(1:3, 4, i-1);
            currPos = transforms(1:3, 4, i);
    
            % Plot a line between the joint positions with thicker lines
            lines_handles(i) = line([prevPos(1), currPos(1)]*grid.resolution, [prevPos(2), currPos(2)]*grid.resolution, [prevPos(3), currPos(3)]*grid.resolution, 'Color', 'k', 'LineWidth', 2);
        end
    end
    
    % set display properties
    axis equal
    view([-180.9 45.0])
    hold off;


    % draw APF field
    % ---------------------------------------------------------------

    j = size(output.POI_values,2);
    for i = 1:1:size(output.POI_values{1},2)        
        % plot the arrow
        hold on

        % calculate color
        apf_magnitude = norm([output.POI_values{j}(1,i),output.POI_values{j}(2,i),output.POI_values{j}(3,i)]);
        apf_magnitude_remapped = round(min(apf_magnitude,1)*255 + 1);
        apf_color = cMap(apf_magnitude_remapped,:);

        APF_handles(i) = quiver3(output.POI_locations{j}(1,i)*grid.resolution, output.POI_locations{j}(2,i)*grid.resolution, output.POI_locations{j}(3,i)*grid.resolution, output.POI_values{j}(1,i)*arrow_length, output.POI_values{j}(2,i)*arrow_length, output.POI_values{j}(3,i)*arrow_length, arrow_length/2,  'LineWidth', 2, 'MaxHeadSize', 1, 'Color', apf_color);
   
        field_poi_dots_handles(i) = scatter3(output.POI_locations{j}(1,i)*grid.resolution, output.POI_locations{j}(2,i)*grid.resolution, output.POI_locations{j}(3,i)*grid.resolution, 'filled', 'MarkerFaceColor', apf_color, 'MarkerEdgeColor', apf_color);
        
    end


    % Set the limits
    xlim([0 200]);   % X-axis limits
    ylim([0 200]);   % Y-axis limits
    zlim([-50 200]);    % Z-axis limits
    axis equal


    %% UPDATE TIME
    % ---------------------------------------------------------------

    function upPlot(sliderValue)
  
        % get figure time from slider
        j = max(1,round(sliderValue * (size(output.joints_positions,2)-1) / 100))

        % calculate transforms from joint positions
        [transforms] = GeometricPandaMATLAB(output.joints_positions(:,j), Tbase);
    
        for k = 1:length(transforms)
    %         % Plot the transformation frame with thicker lines
    %         plotTransforms(transforms(1:3, 4, i)'*grid.resolution, rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);
        
            % Plot the joint point
            points_handles(k).XData = transforms(1, 4, k)*grid.resolution;
            points_handles(k).YData = transforms(2, 4, k)*grid.resolution;
            points_handles(k).ZData = transforms(3, 4, k)*grid.resolution;

            % Connect the joint points with thicker lines
            if k > 1
                % Get the previous and current joint positions
                prevPos = transforms(1:3, 4, k-1);
                currPos = transforms(1:3, 4, k);
        
                % Plot a line between the joint positions with thicker lines
                lines_handles(k).XData = [prevPos(1), currPos(1)]*grid.resolution;
                lines_handles(k).YData = [prevPos(2), currPos(2)]*grid.resolution;
                lines_handles(k).ZData = [prevPos(3), currPos(3)]*grid.resolution;
            end
        end

        % update APF field
        % --------------------------------------------------------------    
    
        for i = 1:1:size(output.POI_values{1},2)

            hold on

            % calculate color
            apf_magnitude = norm([output.POI_values{j}(1,i),output.POI_values{j}(2,i),output.POI_values{j}(3,i)]);
            apf_magnitude_remapped = round(min(apf_magnitude,1)*255 + 1);
            apf_color = cMap(apf_magnitude_remapped,:);
       
            APF_handles(i).Color = apf_color;
            APF_handles(i).XData = output.POI_locations{j}(1,i)*grid.resolution;
            APF_handles(i).YData = output.POI_locations{j}(2,i)*grid.resolution;
            APF_handles(i).ZData = output.POI_locations{j}(3,i)*grid.resolution;
            APF_handles(i).UData = output.POI_values{j}(1,i) * arrow_length * show_arrows;
            APF_handles(i).VData = output.POI_values{j}(2,i) * arrow_length * show_arrows;
            APF_handles(i).WData = output.POI_values{j}(3,i) * arrow_length * show_arrows;



            field_poi_dots_handles(i).XData = output.POI_locations{j}(1,i)*grid.resolution;
            field_poi_dots_handles(i).YData = output.POI_locations{j}(2,i)*grid.resolution;
            field_poi_dots_handles(i).ZData = output.POI_locations{j}(3,i)*grid.resolution;
            field_poi_dots_handles(i).MarkerEdgeColor = apf_color;
            field_poi_dots_handles(i).MarkerFaceColor = apf_color;


                
        
        end
        % ---------------------------------------------------------------

    end

    
    %% TURN ON / OFF VISUALIZATIONS
    % ---------------------------------------------------------------

    function setAPFArrows(checkboxValue)
            % Check if the checkbox is checked
        if checkboxValue == 1
            show_arrows = 1;
        else
            show_arrows = 0;
        end    
    
    end

    function setObstaclesGrid(checkboxValue)
        % Check if the checkbox is checked
        if checkboxValue == 1
            % display grid
            HObstacles = grid.showGridVol3D(grid.grid,'floor',floor,'height',true);
        else
            for i = 1:1:length(HObstacles.handles)
                delete(HObstacles.handles(i))
            end
        end    
    
    end



    %% CONTROL CALLBACKS
    % ---------------------------------------------------------------

    function sliderCallback(source,event)
        % This function will be called whenever the slider's value is changed.
        % You can add the code here that will be executed whenever the slider is moved.
        % For example, this might involve redrawing the robot with different joint positions based on the slider's current value.
        sliderValue = source.Value  % Get the current value of the slider
    
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

    % Callback function for the "Play" button
    function playButtonCallback(~, ~)
        % Get the current slider value
        current_slider_value = 1;

        % Define the animation parameters
        % Total animation time in seconds
        animation_speed = 100;  % Frames per second
        
        % Iterate through slider values to animate
        for i = 1:100
            % Calculate the new slider value
            new_slider_value = current_slider_value + i * 1;

            % Ensure the slider value does not exceed the maximum
            new_slider_value = max(min(new_slider_value, 100),1);

            % Update the slider
            s.Value = new_slider_value;

            % Call the slider callback function to update the visualization
            sliderCallback(s, []);
            
            % Pause to control the animation speed
            pause(1 / animation_speed);
        end
    end
 

end

