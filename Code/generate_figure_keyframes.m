function [f] = generate_figure_keyframes(grid, Tbase, control_points, output)
    % Define the keyframes
    total_frames = size(output.joints_positions, 2);
    keyframes = round(linspace(1, total_frames-1, 6));

    % APF field vector length
    arrow_length = 0.5; % adjust the length to your preference

    %% COLORMAP for APF vectors
    cMap = interp1(0:1,[0 1 0; 1 0 0],linspace(0,1,256));
    cMap = cMap.^(1/2.4); % linearize

    %% Create figure for subplots
    f = figure('Position', [100, 100, 500, 300]);
    set(f, 'WindowStyle', 'normal');

    % Loop through each keyframe
    for kf = 1:length(keyframes)
        % Set the axes limits to ensure consistency across subplots
        xlim([0 20]);   
        ylim([0 20]);   
        zlim([0 10]);
%         axis equal;  

        % Create subplot for each keyframe
        subplot(2, 3, kf);
        % Remove axis rulers
        set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
        box on; % Turn the box on
%         set(gca, 'LineWidth', 1); % Set the box line width to 1 (adjust this value as needed)
        hold on;
%         axis equal;
        
        % Current keyframe index
        j = keyframes(kf);
        
        % Display grid
        grid.showGridVol3D(grid.grid, 'floor', false, 'height', true);
        
        % Display control points
        for i = 1:size(control_points,1)
            scatter3(control_points(i,1)*grid.resolution, control_points(i,2)*grid.resolution, control_points(i,3)*grid.resolution, 'r');
        end
        
        % Draw EE trajectory up to the current keyframe
        plot3(output.EE_positions(1,1:j)*grid.resolution, output.EE_positions(2,1:j)*grid.resolution, output.EE_positions(3,1:j)*grid.resolution, 'red');
        
        % Calculate transforms for the current keyframe
        [transforms] = GeometricPandaMATLAB(output.joints_positions(:,j), Tbase);
        
        % Draw robot configuration for the current keyframe
        for i = 1:length(transforms)
            if i > 1
                prevPos = transforms(1:3, 4, i-1);
                currPos = transforms(1:3, 4, i);
                line([prevPos(1), currPos(1)]*grid.resolution, [prevPos(2), currPos(2)]*grid.resolution, [prevPos(3), currPos(3)]*grid.resolution, 'Color', 'k', 'LineWidth', 2);
            end
        scatter3(transforms(1, 4, i)*grid.resolution, transforms(2, 4, i)*grid.resolution, transforms(3, 4, i)*grid.resolution, 'o',  'MarkerFaceColor', [38/255, 151/255, 224/255], 'MarkerEdgeColor', [38/255, 151/255, 224/255]);
        end


    % Draw APF field for the current keyframe
    for i = 1:(size(output.POI_values{j},2))       
        % Calculate color for APF vector
        apf_magnitude = norm(output.POI_values{j}(:,i));
        apf_magnitude_remapped = round(min(apf_magnitude,1)*255 + 1);
        apf_color = cMap(apf_magnitude_remapped,:);
        
        quiver3(output.POI_locations{j}(1,i)*grid.resolution, output.POI_locations{j}(2,i)*grid.resolution, output.POI_locations{j}(3,i)*grid.resolution, output.POI_values{j}(1,i)*arrow_length, output.POI_values{j}(2,i)*arrow_length, output.POI_values{j}(3,i)*arrow_length, 'LineWidth', 2, 'MaxHeadSize', 1, 'Color', apf_color);
        scatter3(output.POI_locations{j}(1,i)*grid.resolution, output.POI_locations{j}(2,i)*grid.resolution, output.POI_locations{j}(3,i)*grid.resolution, 'filled', 'MarkerFaceColor', apf_color, 'MarkerEdgeColor', apf_color);
    end
    
    % Set the subplot title to indicate the keyframe
%     title(['Keyframe ' num2str(kf)]);
    
    xlabel('');
    ylabel('');
    zlabel('');


    view([-180 45.0]);
    
    hold off;
    end
end