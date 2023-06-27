function showPanda(Abase, A01, A12, A23, A34, A45, A56, A67, space_resolution, robotColor)

    % This function shows a Franka Emika Panda robot arm. The inputs are angles of the joints and the base transformation matrix.
    
    % Set joint and line colors
    jointColor = robotColor;
    lineColor = robotColor;
    lineWidth = 2;

    figure()
    hold on;
    
    % Draw base
    TT = Abase;
    
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

%     hold off;


end