function showRobot(Abase, A01, A12, A23, A34, A45, A56, A67, space_resolution, robotColor)

    % This function shows a robot arm using the given transformation matrices for each joint.
    % The robot arm is drawn in the given color.
    %
    % Inputs:
    % - Abase: 4x4 transformation matrix for the base of the robot arm
    % - A01: 4x4 transformation matrix for joint 1 of the robot arm
    % - A12: 4x4 transformation matrix for joint 2 of the robot arm
    % - A23: 4x4 transformation matrix for joint 3 of the robot arm
    % - A34: 4x4 transformation matrix for joint 4 of the robot arm
    % - A45: 4x4 transformation matrix for joint 5 of the robot arm
    % - A56: 4x4 transformation matrix for joint 6 of the robot arm
    % - A67: 4x4 transformation matrix for joint 7 of the robot arm
    % - robotColor: color of the robot arm (RGB triplet)
    %
    % Outputs:
    % - none
    %
    % Example usage:
    % Abase = eye(4);
    % A01 = transl(0,0,0.2) * trotz(pi/2);
    % A12 = transl(0.2,0,0) * trotz(pi/2);
    % A23 = transl(0,0.2,0) * trotz(pi/2);
    % A34 = transl(0,0.2,0);
    % A45 = trotz(pi/2);
    % A56 = transl(0,0.1,0.3) * trotz(-pi/2);
    % A67 = transl(0,-0.1,0.1) * trotz(pi);
    % robotColor = [0.1 0.7 0.9];
    % showRobot(Abase, A01, A12, A23, A34, A45, A56, A67, robotColor);

    % Set joint and line colors
    jointColor = robotColor;
    lineColor = robotColor;
    lineWidth = 2;
    
    % Draw base
    TT = Abase;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, 0, 'filled', 'MarkerFaceColor', robotColor); % plot base position
    hold on
    plotTransforms(TT(1:3,4)'*space_resolution,rotm2quat(TT(1:3,1:3)),'FrameSize',0.1) % plot base coordinate frame
    
    % Draw robot arm
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor');
    x = [TT(1,4) TT(1,4)]*space_resolution;
    y = [TT(2,4) TT(2,4)]*space_resolution;
    z = [0 TT(3,4)]*space_resolution;
    line(x,y,z,'Color',robotColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01*A12;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01*A12*A23;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01*A12*A23*A34;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01*A12*A23*A34*A45;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01*A12*A23*A34*A45*A56;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    TTo = TT;

    TT = Abase*A01*A12*A23*A34*A45*A56*A67;
    scatter3(TT(1,4)*space_resolution, TT(2,4)*space_resolution, TT(3,4)*space_resolution, 'filled', 'MarkerFaceColor', jointColor);
    x = [TTo(1,4) TT(1,4)]*space_resolution;
    y = [TTo(2,4) TT(2,4)]*space_resolution;
    z = [TTo(3,4) TT(3,4)]*space_resolution;
    line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
    axis equal

    plotTransforms(TT(1:3,4)'*space_resolution,rotm2quat(TT(1:3,1:3)),'FrameSize',0.1)


end