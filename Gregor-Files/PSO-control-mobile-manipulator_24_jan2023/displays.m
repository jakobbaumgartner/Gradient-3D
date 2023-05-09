classdef displays

    % DISPLAYS class

    properties

        % imported classes
        robot = {}
        displayScale = [-2 2 -2 2 0 2];

       
    end

    methods

        function obj = displays (obj)

               obj.robot = robotPmb2Panda()

        end


        function display2Dpoints(obj, baseTrajectory, baseStatus, eeTrajectory, params)

            figure()
            axis(obj.displayScale(1:4))

            % Pee - T end-effector
            % Pbase - x,y,fi base

            % check which of datasets are given
            dataBaseTraj = size(baseStatus,1) > 0;
            dataEETraj = size(eeTrajectory,1) > 0;
            dataParam = size(params,1) > 0;


            % plot all pairs connecting lines
            if dataEETraj && dataBaseTraj
    
                for i = 1:1:size(eeTrajectory,3)
                
                    if (baseStatus(i))
    
                        B = baseTrajectory(i,:);
                        T = eeTrajectory(:,:,i);
    
    
                        x = [B(1) T(1,4)];
                        y = [B(2) T(2,4)];
                        line(x,y,'Color','#a3d6ff','LineStyle','--', 'linewidth', 1);
                        hold on
                    end

                end
            end


            % plot all EE points on trajectory
            if dataEETraj
                for i=1:1:size(eeTrajectory,3)
    
                    T = eeTrajectory(:,:,i);
                    
                    scatter(T(1,4), T(2,4), 'filled', 'MarkerFaceColor', '#0072BD');
    
                    % scalling factor for orientation arrow 
                    arrSc = 1/ norm(T(:,3));
                
    
                % if EE orientation is horitonzal, calculate base position
                    
                    % draw EE orientation arrow
                    quiver(T(1,4),T(2,4),T(1,3) * arrSc, T(2,3) * arrSc,0, 'linewidth',1.5,'color','#0072BD');
    
                end
            end


            % plot all base points on trajectory
            if dataBaseTraj
                for i=1:1:size(baseTrajectory,1)
    
                    B = baseTrajectory(i,:);
                    
                    scatter(B(1), B(2), 'filled', 'MarkerFaceColor', '#0072BD', 'Marker', 's')
                    
    
                    if baseStatus(i)
                     
                        fi = B(3);
        
                        x = B(1) + 0.25 * cos(fi);
                        y = B(2) + 0.25 * sin(fi);
        
                        xy2 = [x y];
        
                    else
        
                        xy2 = [B(1) B(2)];
        
                    end
                end
                
                % draw EE orientation arrow
                quiver(B(1),B(2),xy2(1)-B(1),xy2(2)-B(2),0, 'linewidth',1.5,'color','#0072BD')

            end


            % plot all calculated points
            if dataParam 
            for i = 1:1:size(params,1)

                    % plot lines
                    B = params(i,8:10);
                    T = obj.robot.directKinematics(params(i,:));
                    x = [B(1) T(1,4)];
                    y = [B(2) T(2,4)];
                    line(x,y,'Color','#77AC30','LineStyle','--', 'linewidth', 1)
                    hold on

                    % plot EE orientation
                    fi = atan2(T(2,3), T(1,3));

                    x = T(1,4) + 0.4 * cos(fi);
                    y = T(2,4) + 0.4 * sin(fi);

                    % draw EE orientation arrow
                    quiver(T(1,4),T(2,4),x-(T(1,4)),y-T(2,4),0, 'linewidth',1.5,'color','#77AC30');

                    % plot EE
                    scatter(T(1,4), T(2,4), 'filled', 'MarkerFaceColor', '#77AC30');

                    % plot base
                    scatter(B(1), B(2), 'filled', 'MarkerFaceColor', '#77AC30',  'Marker', 's');

                    % plot base orientation
                    fi = B(3);
    
                    x = B(1) + 0.25 * cos(fi);
                    y = B(2) + 0.25 * sin(fi);
    
                    xy2 = [x y];
    

                    % draw base orientation arrow
                    quiver(B(1),B(2), xy2(1)-B(1),xy2(2)-B(2), 'linewidth',1.5,'color','#77AC30')


            end

%             % plot legend
%             legend('', 'suggested', '', '', '', '', '', 'PSO calculated')
            
            end
        end



     function display3Dpoints(obj, baseTrajectory, baseStatus, eeTrajectory, params)


            % Pee - T end-effector
            % Pbase - x,y,fi base

            % check which of datasets are given
            dataBaseTraj = size(baseStatus,1) > 0;
            dataEETraj = size(eeTrajectory,1) > 0;
            dataParam = size(params,1) > 0;

            figure()
            axis(obj.displayScale)



            % plot all pairs connecting lines 
            if dataEETraj && dataBaseTraj   

                for i = 1:1:size(eeTrajectory,3)
                
    
                        B = baseTrajectory(i,:);
                        T = eeTrajectory(:,:,i);
    
    
                        x = [B(1) T(1,4)];
                        y = [B(2) T(2,4)];
                        z = [0 T(3,4)];
                        plot3(x,y,z,'Color','#a3d6ff','LineStyle','--', 'linewidth', 1);
                        hold on
                        axis(obj.displayScale)

    
                end

            end

                
            % plot all EE points on trajectory
            if dataEETraj   

                for i=1:1:size(eeTrajectory,3)

                    T = eeTrajectory(:,:,i);
                    
                    scatter3(T(1,4), T(2,4), T(3,4), 'filled', 'MarkerFaceColor', '#0072BD');
                    hold on
                    axis(obj.displayScale)

                    
                    % scalling factor for orientation arrow 
                    arrSc = 0.3 / norm(T(:,3));

                        
                    % draw EE orientation arrow
                    quiver3(T(1,4),T(2,4), T(3,4), T(1,3) * arrSc, T(2,3) * arrSc, T(3,3) * arrSc, 'linewidth',1.5,'color','#0072BD');
                    hold on
                end

            end
            

            % plot all base points on trajectory
            if dataBaseTraj   

                for i=1:1:size(baseTrajectory,1)
    
                    B = baseTrajectory(i,:);
                    
                    scatter3(B(1), B(2), 0, 'filled', 'MarkerFaceColor', '#0072BD', 'Marker', 's')
                    hold on
                    axis(obj.displayScale)

    
                if baseStatus(i)
                 
                    fi = B(3);
    
                    x = B(1) + 0.25 * cos(fi);
                    y = B(2) + 0.25 * sin(fi);
    
                    xy2 = [x y];
    
                else
    
                    xy2 = [B(1) B(2)];
    
                end
                    
                    % draw base orientation arrow
                    quiver3(B(1),B(2), 0, xy2(1)-B(1),xy2(2)-B(2), 0, 'linewidth',1.5,'color','#0072BD')
                    hold on
                end
            end


            % plot all calculated points
            if dataParam

                for i = 1:1:size(params,1)
    
                        % plot lines
                        B = params(i,8:end);
                        [~,~,~,~,~,~,~,~,T] = obj.robot.GeometricRobot(params(i,1:7), [params(i,8:9) 0]);

%                         % normalize T matrix
%                         % (for correct arrow drawing)
                        [u s vt] = svd(T(1:3,1:3));
                        T(1:3,1:3) = u * vt';
                        
                        
                        % scalling factor for orientation arrow 
                        arrSc = 0.3 / norm(T(:,3));

                        x = [B(1) T(1,4)];
                        y = [B(2) T(2,4)];
                        z = [0 T(3,4)];
                        plot3(x,y,z,'Color','#77AC30','LineStyle','--', 'linewidth', 1)
                        hold on
                        axis(obj.displayScale)

    
                        % draw EE orientation arrow
                        quiver3(T(1,4),T(2,4), T(3,4), T(1,3) * arrSc, T(2,3) * arrSc, T(3,3) * arrSc, 'linewidth',1.5,'color','#77AC30');
    
                        % plot EE
                        scatter3(T(1,4), T(2,4), T(3,4), 'filled', 'MarkerFaceColor', '#77AC30');
    
                        % plot base
                        scatter3(B(1), B(2), 0, 'filled', 'MarkerFaceColor', '#77AC30',  'Marker', 's');
    
%                         % plot base orientation
%                         fi = B(3);
%         
%                         x = B(1) + 0.25 * cos(fi);
%                         y = B(2) + 0.25 * sin(fi);
%         
%                         xy2 = [x y];
%         
%     
%                         % draw base orientation arrow
%                         quiver3(B(1),B(2), 0, xy2(1)-B(1),xy2(2)-B(2), 0, 'linewidth',1.5,'color','#77AC30')
    
                        % display skeleton
                        [Abase, A01, A12, A23, A34, A45, A56, A67, T] = obj.robot.GeometricRobot(params(i,1:7), [params(i,8:9) 0]);
                        
    
                        jointColor = '#000000';
                        lineColor = '#949494';
                        lineWidth = 2;
                        
    
                      
    
    
                end
            end
            

     end

     


        function displaySkeleton(obj, baseTrajectory, baseStatus, eeTrajectory, params)

            display3Dpoints(obj, baseTrajectory, baseStatus, eeTrajectory, params)

            dataParam = size(params,1) > 0;

            % plot all calculated points
            if dataParam

                for i = 1:1:size(params,1)
    

                        % display skeleton
                        [Abase, A01, A12, A23, A34, A45, A56, A67, T] = obj.robot.kinematicTransformations(params);
                           
                        Abase = eye(4)
                        jointColor = '#000000';
                        lineColor = '#949494';
                        lineWidth = 2;
                        
                        TT = Abase;
                        scatter3(TT(1,4), TT(2,4), 0, 'filled', 'MarkerFaceColor', '#D95319');
                        hold on
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor');
                        x = [TT(1,4) TT(1,4)];
                        y = [TT(2,4) TT(2,4)];
                        z = [0 TT(3,4)];
                        line(x,y,z,'Color','#D95319', 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor);
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01*A12;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', 'red');
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01*A12*A23;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor);
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01*A12*A23*A34;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor);
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01*A12*A23*A34*A45;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor);
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01*A12*A23*A34*A45*A56;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor);
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        TTo = TT;
                        TT = Abase*A01*A12*A23*A34*A45*A56*A67;
                        scatter3(TT(1,4), TT(2,4), TT(3,4), 'filled', 'MarkerFaceColor', jointColor);
                        x = [TTo(1,4) TT(1,4)];
                        y = [TTo(2,4) TT(2,4)];
                        z = [TTo(3,4) TT(3,4)];
                        line(x,y,z,'Color',lineColor, 'linewidth', lineWidth)
                        axis equal
    
    
                end
            end
            

        end
    
    end
end