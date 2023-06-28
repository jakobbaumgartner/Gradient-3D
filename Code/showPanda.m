function showPanda(transforms,scale_view)

    % Function: showPanda(transforms)
    %
    % Description:
    % This function plots a visualization of the Panda robot arm with the given set of transformations. It displays the transformation frames, joint points, and connects the joint points with lines.
    %
    % Inputs:
    % - transforms: A 4x4xN array representing the transformation matrices of the Panda robot arm. N is the number of joint positions. Each transformation matrix contains the rotation and translation information for a specific joint position.
    % - scale_view: A scalar value to scale the distances in the displayed plots. Use a value greater than 1 to enlarge the plot or a value less than 1 to shrink the plot.
    %
    % Outputs:
    % None.

    hold on;

    for i = 1:length(transforms)
        % Plot the transformation frame with thicker lines
        plotTransforms(transforms(1:3, 4, i)'*scale_view, rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);

        % Plot the joint point
        plot3(transforms(1, 4, i)*scale_view, transforms(2, 4, i)*scale_view, transforms(3, 4, i)*scale_view, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [38/255, 151/255, 224/255], 'MarkerEdgeColor', [38/255, 151/255, 224/255]);

        % Connect the joint points with thicker lines
        if i > 1
            % Get the previous and current joint positions
            prevPos = transforms(1:3, 4, i-1);
            currPos = transforms(1:3, 4, i);

            % Plot a line between the joint positions with thicker lines
            line([prevPos(1), currPos(1)]*scale_view, [prevPos(2), currPos(2)]*scale_view, [prevPos(3), currPos(3)]*scale_view, 'Color', 'k', 'LineWidth', 2);
        end
    end

    hold off;

end
