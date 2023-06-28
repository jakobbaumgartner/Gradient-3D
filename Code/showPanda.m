function showPanda(transforms)

    hold on;

    for i = 1:length(transforms)
        % Plot the transformation frame with thicker lines
        plotTransforms(transforms(1:3, 4, i)', rotm2quat(transforms(1:3, 1:3, i)), 'FrameSize', 0.2);

        % Plot the joint point
        plot3(transforms(1, 4, i), transforms(2, 4, i), transforms(3, 4, i), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [38/255, 151/255, 224/255], 'MarkerEdgeColor', [38/255, 151/255, 224/255]);

        % Connect the joint points with thicker lines
        if i > 1
            % Get the previous and current joint positions
            prevPos = transforms(1:3, 4, i-1);
            currPos = transforms(1:3, 4, i);

            % Plot a line between the joint positions with thicker lines
            line([prevPos(1), currPos(1)], [prevPos(2), currPos(2)], [prevPos(3), currPos(3)], 'Color', 'k', 'LineWidth', 2);
        end
    end

    hold off;

end
