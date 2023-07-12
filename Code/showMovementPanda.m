function [f] = showMovementPanda(grid, grid_matrix, joints_positions_APF, EE_positions_APF, Tbase)

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

% display grid
grid.showGridVol3D(grid_matrix,'floor',false,'height',false)
hold on
axis equal

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


    % function to changed plot time
    function upPlot(sliderValue)

        disp((lines_handles))
    
        % get figure time from slider
        j = round(sliderValue * size(joints_positions_APF,2) / 100)

        % calculate transforms from joint positions
        [transforms] = GeometricPandaMATLAB(joints_positions_APF(:,j), Tbase);
    
        for k = 1:length(transforms)
    %         % Plot the transformation frame with thicker lines
    %         plotTransforms(transforms(1:3, 4, i)'*scale_view, rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);
        
            % Plot the joint point
            points_handles(k).XData = transforms(1, 4, k)*scale_view
            points_handles(k).YData = transforms(2, 4, k)*scale_view
            points_handles(k).ZData = transforms(3, 4, k)*scale_view

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



    end


% callback function is nested so that is can read and write to the existing variables !!!
function sliderCallback(source,event)
    % This function will be called whenever the slider's value is changed.
    % You can add the code here that will be executed whenever the slider is moved.
    % For example, this might involve redrawing the robot with different joint positions based on the slider's current value.
    sliderValue = source.Value;  % Get the current value of the slider

    upPlot(sliderValue)

end

end

