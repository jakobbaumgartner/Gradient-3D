%%%%%%%%%%%%%%%%%%%%%%%%% Draw figures fot L-shaped obstacle in article
% clear all, close all
 
 flagUseEstar=1;  % ali imam E star ali A star za definiranje potencialnega polja
 
 flagStoreImages=0; % do we store images to eps
 vMAX=1; vMIN=0; wMAX=2;aMAX=4; alphaMAX=1; Ts=0.1; 
 
 %DIM=10; 
 %space=[0,DIM,0,DIM]; obstacles=[2 4 2 4; 6 8 6 8; 2 4 6 8; 6 8 2 4]; resolution=2;  % MultiRob
 
 DIM=10;
 space=[0,DIM,0,DIM];  resolution=2;  % MultiRob
% obstacles=[2 4 2 4; 6 8 6 8; 2 4 6 8; 6 8 2 4];
%obstacles=[2 4 2 4;  2 4 6 8;6 8 2 4];
% obstacles=[2 4 2 4; 6 8 6 8; 2 4 6 8; 6 8 2 4];

 
 obstacles=[];

 
 
 % start and goal definition
 nRob=1;
 id=1; Rob(id).start=[1-.1;3.5-.1; pi*(0)];  %goalBase=[6.4+.5+2; 9.5+.5*3] +[-1 ;2]*0  % goal=[7; 9]
 
 %goalBase=[8;8.5] % kot za roko

% goalBase=[10;10] % kot za roko


 Rob(id).goal=[floor(floor(goalBase(1))*resolution)/resolution+1/resolution/2;floor(floor(goalBase(2))*resolution)/resolution+1/resolution/2];
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



%

if 0 % naknadno lahko dodam ovire
    %%%=============== add L-shaped obstacle ==================
    
%     obst=[5.1, 5.1 ;
%           5.1, 4.7 ; 
%           5.1, 5.6; 
%           4.6, 5.6; 
%           4.1, 5.6;
%           3.9 ,5.6;
%           5.4 4.25]; 
%     obst=obst+[ones(size(obst,1),1),zeros(size(obst,1),1)]*0.5;  
%    newObstacleReplan=astarG.addObstaclesToMap(obst);
%    fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none


% astarG.addObstaclesToMap([1.6,3.1]);
% fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none


  % prepišem v M_ in obrnem
    M_=zeros(size(astarG.map,2),size(astarG.map,1));
    for i=1:size(astarG.map,1)
        for j=1:size(astarG.map,2)
           M_(size(astarG.map,1)-j+1,i)=astarG.map(i,j);
        end
    end
   figure(1),imagesc(M_)

 figure(1)
 [x,y,button]=ginput(1)

 while 1
     figure(1)
     [x,y,button]=ginput(1)
      px = round( x+0.5*0)
      py = round( y+0.5*0)
            if button==1 
                 M_(py,px)=1;
            else
                 M_(py,px)=0;
            end
     imagesc(M_)
 end 


    % zamenjam nazaj x/y vrstica/stolpec
    for i=1:size(M_,1)
        for j=1:size(M_,2)
             astarG.map(j,i)=M_(i,j);
        end
    end
  astarG.map= fliplr(astarG.map);
 
 close(10) 
 fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none

 
 load MapEnvironment % astarMap=astarG.map; save MapEnvironment astarMap
   
end


load MapEnvironment  
astarG.map=astarMap;

%close(10) 
fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none




%goal=[[5.2; 6.6]];  



 if flagUseEstar
    %Estar % vozi lahko v poljubnih smereh
    astarG.findEstarFromGoal( astarG.mapToGrid(Rob(RobotID).start(1:2)) , astarG.mapToGrid(goalBase) , nearobst ); 
 else
    %Astar 
    astarG.findDijkstraFromGoal( astarG.mapToGrid(Rob(RobotID).start(1:2)) , astarG.mapToGrid(goalBase)  ); 
 end



CMG=astarG.getCostMapFromClosedList(); % returns Cost To Here for search from goal (so costs to goal)
CMGmax=max(CMG(CMG~=max(max(CMG))))+1; % seccond highest value +1 (the highest value is obstacle)

if 1 
  fig=11; flagDraw=0; % flagDraw=2 brez kontur, =0 s konturami, =1 gradienti
  fnDrawIterpolatedBilinearPotentialField2(space,resolution,CMG,CMGmax, fig,flagDraw), hold on; % draws interpolated potential on desired figure
  xlabel('$$x$$[m]','interpreter','latex','FontSize',12),ylabel('$$y$$[m]','interpreter','latex','FontSize',12)
  zlabel('$$P$$[m]','interpreter','latex','FontSize',12)
  %set(gca,'view',[10 55])
  set(gca,'view',[40 60])
  
 if 0 
  fig=14; flagDraw=1; % flagDraw=2 brez kontur, =0 s konturami, =1 gradienti
  
  fnDrawIterpolatedBilinearPotentialField2(space,resolution,CMG,CMGmax, fig,flagDraw), hold on; % draws interpolated potential on desired figure
  xlabel('$$x$$[m]','interpreter','latex','FontSize',14),ylabel('$$y$$[m]','interpreter','latex','FontSize',14)
  zlabel('$$P$$[m]','interpreter','latex','FontSize',14)
  set(gca,'view',[-0.3375   90.0000])
  astarG.redrawMap(fig,[])
    axis equal

  %axis([0 10 0 10])
  axis([5 10 0 5])
  
  %  print -depsc  gradinetNonInterpolatedBig;   
  %  print -depsc  gradinetInterpolatedBig;   

  axis([6.5 7 3 3.5])
  %  print -depsc  gradinetNonInterpolatedSmall;   
  %  print -depsc  gradinetInterpolatedSmall;   

 end
  
  
end



% odprem novo okno 
%fig=3; astarG.redrawMap(fig,CMG); % draw map on figure, add cost map CMG or put [] for none
fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none
h=1;g_=[];


return


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
 
  end
  for i=1:size(g_,1)
        g_start=g_(i,:);
        QQb=GetPathByfollowigInterpolatedGradientBilinear(g_start,CMG,space,resolution,0.01);
        figure(fig),plot(QQb(:,1),QQb(:,2))

        xlabel('$$x$$[m]','interpreter','latex','FontSize',14),ylabel('$$y$$[m]','interpreter','latex','FontSize',14)

 
  end
end




