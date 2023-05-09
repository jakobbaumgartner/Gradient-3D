%%%%%%%%%%%%%%%%%%%%%%%%% Draw figures fot L-shaped obstacle in article
 clear all, close all
 
 flagUseEstar=0;  % ali imam E star ali A star za definiranje potencialnega polja
 
 flagStoreImages=0; % do we store images to eps
 vMAX=1; vMIN=0; wMAX=2;aMAX=4; alphaMAX=1; Ts=0.1; 
 DIM=10;
 space=[0,DIM,0,DIM]; obstacles=[2 4 2 4; 6 8 6 8; 2 4 6 8; 6 8 2 4]; resolution=2;  % MultiRob

 
 
 obstacles=[]

 
 
 % start and goal definition
 nRob=1;
 id=1; Rob(id).start=[1-.1;3.5-.1; pi*(0)];  goal=[8.7;8.25]; 
 Rob(id).goal=[floor(floor(goal(1))*resolution)/resolution+1/resolution/2;floor(floor(goal(2))*resolution)/resolution+1/resolution/2];
 RobotID=id;

 
 %if(~exist('astarG','var')), astarG = AStarClassicMaria(); end; 
 if flagUseEstar
      %nearobst=4.1; % ta za %resolution=4;
      %nearobst=2.1; % ta za %resolution=2;
      nearobst=1.0;
    if(~exist('astar','var')), astarG = EStarClassicMaria(); end  % Estar
 else
    if(~exist('astar','var')), astarG = AStarClassicMaria(); end  % Astar
 end
 


astarG.showMode = 0;  % 3-draws, 1-draws free and occupied cells,   0 -do not draw
astarG.environment(space,resolution, obstacles ); % sets map to only static obstacles




if 0 % naknadno lahko dodam ovire
    %%%=============== add L-shaped obstacle ==================
    obst=[5.1, 5.1 ;
          5.1, 4.7 ; 
          5.1, 5.6; 
          4.6, 5.6; 
          4.1, 5.6;
          3.9 ,5.6;
          5.4 4.25]; 
    obst=obst+[ones(size(obst,1),1),zeros(size(obst,1),1)]*0.5;  

    newObstacleReplan=astarG.addObstaclesToMap(obst);
end


goal=[[5.2; 6.6]];  



 if flagUseEstar
    %Estar % vozi lahko v poljubnih smereh
    astarG.findEstarFromGoal( astarG.mapToGrid(Rob(RobotID).start(1:2)) , astarG.mapToGrid(goal) , nearobst ); 
 else
    %Astar 
    astarG.findDijkstraFromGoal( astarG.mapToGrid(Rob(RobotID).start(1:2)) , astarG.mapToGrid(goal)  ); 
 end



CMG=astarG.getCostMapFromClosedList(); % returns Cost To Here for search from goal (so costs to goal)
CMGmax=max(CMG(CMG~=max(max(CMG))))+1; % seccond highest value +1 (the highest value is obstacle)


%       z=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(x(i),y(j),FF,FFmax,space,resolution);
%       z=[f,gx,gy];  %Interpoliran cost in interpoliran gradient 



if 1 
  fig=4; flagDraw=0; % flagDraw=2 brez kontur, =0 s konturami, =1 gradienti
  fnDrawIterpolatedBilinearPotentialField2(space,resolution,CMG,CMGmax, fig,flagDraw), hold on; % draws interpolated potential on desired figure
  xlabel('   (c)  $$x$$[m]','interpreter','latex','FontSize',12),ylabel('$$y$$[m]','interpreter','latex','FontSize',12)
  zlabel('$$P$$[m]','interpreter','latex','FontSize',12)
  %set(gca,'view',[10 55])
  set(gca,'view',[40 60])
end



% odprem novo okno 
fig=3; astarG.redrawMap(fig,CMG); % draw map on figure, add cost map CMG or put [] for none
h=1;g_=[];


if 0
    axis equal
    g_=ginput(1);
    while ~isempty(g_) 
        g_start=g_;
        QQb=GetPathByfollowigInterpolatedGradientBilinear(g_start,CMG,space,resolution,0.01);
        figure(fig),plot(QQb(:,1),QQb(:,2))

        xlabel('$$x$$[m]','interpreter','latex','FontSize',14),ylabel('$$y$$[m]','interpreter','latex','FontSize',14)
        g_=ginput(1);
    end
end


if 1
        g_(h,:)=[2.2 ,1.5 ];h=h+1;  
        g_(h,:)=[4 ,4.5 ];h=h+1;  
        g_(h,:)=[8.1 ,7.6 ];h=h+1;  
        g_(h,:)=[6.3 ,5.8 ];h=h+1;  
        g_(h,:)=[5.9 ,6.1 ];h=h+1;  
        g_(h,:)=[5.1 ,2.6 ];h=h+1;  
        g_(h,:)=[7.2 ,1.2 ];h=h+1;  
        g_(h,:)=[5.5 ,9];h=h+1;  
        g_(h,:)=[5 ,9];h=h+1;  

%         g_(h,:)=[6.2 ,5.8 ];h=h+1;  % ne deluje ok za tega
%         g_(h,:)=[6.1 ,5.9 ];h=h+1;  % ne deluje ok za tega
%        
%         g_(h,:)=[5.9 ,3.8 ];h=h+1;  % ne deluje ok za tega
% 
%         g_(h,:)=[3.9 ,5.8 ];h=h+1;  % ne deluje ok za tega
%         g_(h,:)=[6.1 ,4.1 ];h=h+1;  % ne deluje ok za tega
%         g_(h,:)=[5.9 ,6.1 ];h=h+1;  % ne deluje ok za tega
%         g_(h,:)=[4.1 ,6.1 ];h=h+1;  % ne deluje ok za tega
 
      for i=1:size(g_,1)
            g_start=g_(i,:);
            QQb=GetPathByfollowigInterpolatedGradientBilinear(g_start,CMG,space,resolution,0.01);
            figure(fig),plot(QQb(:,1),QQb(:,2))

            xlabel('$$x$$[m]','interpreter','latex','FontSize',14),ylabel('$$y$$[m]','interpreter','latex','FontSize',14)
      end
end




