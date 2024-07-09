function showSimulatedPanda(grid,jointAngles)
    close all
    figure
    robot = importrobot("C:\Users\Jakob\Documents\Coding\Gradient-3D\Code\Panda\panda.urdf");
    config = homeConfiguration(robot);
    
    % Convert orientation to a transformation matrix
    orientation = eul2tform([0, 0, 0]); % Assuming roll and pitch are zero
    
    % Define the new position for the base
    newPosition = [1, 1, 0]; % z is assumed to be wheel height
    
    % Create a new fixed transform for the base joint
    baseTransform = trvec2tform(newPosition) * orientation;
    
    % Get the first link (panda_link1)
    firstLink = robot.getBody('panda_link0');
    
    % Set the new transform for the first link's joint
    firstLink.Joint.setFixedTransform(baseTransform);
    
    % Update the robot model by adding the modified first link back
    robot.replaceBody('panda_link0', firstLink);
    
    % Show obstacles
    HObstacles = grid.showGridVol3D(grid.grid,'floor',true,'height',false, 'scaleFactor', 0.1);
    hold on

    axis equal

    viewOption = 6
     switch viewOption
        case 1
            % Default Isometric View
            view(3)
        case 2
            % Top View
            view(0, 90)
        case 3
            % Side View (from positive Y-axis)
            view(90, 0)
        case 4
            % Front View (from positive X-axis)
            view(0, 0)
        case 5
            % Custom Angled View
            view([-37.5, 30])
        case 6
                % axis equal
            view([-180.9 45.0])
        otherwise
            warning('Invalid view option. Defaulting to isometric view.')
            view(3)
    end
    
    % Initialize robot visualization handle
    robotViz = [];
    
    for j = 1:1:51
        % Set the joint angles
        for i = 1:7
            config(i).JointPosition = jointAngles(i,j);
        end
        
        % Delete previous robot visualization, if it exists
        if ~isempty(robotViz)
            try
                delete(findall(gca, 'Type', 'patch', '-and', '-not', 'Tag', 'obstacle'));
            catch
                % If deletion fails, clear the robotViz handle
                robotViz = [];
            end
        end
        
        % Show new robot configuration
        robotViz = show(robot,config,'visuals','on','collision','off','PreservePlot',false);
        drawnow
        pause(0.1)
    end
    hold off
end