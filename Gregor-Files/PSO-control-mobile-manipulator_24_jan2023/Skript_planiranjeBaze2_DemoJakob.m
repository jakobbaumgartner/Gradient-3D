%%%% Demo primer za definiranje diskretnega potencialnega polja in
%%%% interpoliranega potencila ter baze

%  clear all, close all
 
 flagUseEstar=1;  % ali imam E star ali A star za definiranje potencialnega polja
 
 flagStoreImages=0; % do we store images to eps
 vMAX=1; vMIN=0; wMAX=2;aMAX=4; alphaMAX=1; Ts=0.1; 
 
 
 DIM=5;
 space=[0,DIM,0,DIM]; % dimenzije okolja xmin, xmax, ymin, ymax
 resolution=10;  %  velikost celice = 1/resolucija   (koliko celic na enoto 1m)
 obstacles=[];

 
 
 % start and goal definition
 id=1; Rob(id).start=[1-.1;3.5-.1; pi*(0)];  %goalBase=[6.4+.5+2; 9.5+.5*3] +[-1 ;2]*0  % goal=[7; 9]
 
Rob(id).start=[1.5;4; pi*(0)]; 


 
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

fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none


  goalBase=[4.5;2.5] % kot za roko
% goalBase=[10;10] % kot za roko

% Rob(id).goal=[floor(floor(goalBase(1))*resolution)/resolution+1/resolution/2;floor(floor(goalBase(2))*resolution)/resolution+1/resolution/2];
 Rob(id).goal=astarG.mapToGrid(goalBase);

astarG.mapToGrid(goalBase)


 RobotID=id;


if 1 % naknadno lahko dodam ovire

    
 disp('Modificiraj zemljevid:levi klik=prosto (modro), levi klik=ovira (rumeno). ctrl+C za koncanje')

  % prepišem v M_ in obrnem
    M_=zeros(size(astarG.map,2),size(astarG.map,1));
    for i=1:size(astarG.map,1)
        for j=1:size(astarG.map,2)
           M_(size(astarG.map,1)-j+1,i)=astarG.map(i,j);
        end
    end
   figure(1),imagesc(M_)
  title('Modificiraj zemljevid:5x klikni leva ali desna tipka')


%  imax=5
%  while imax>0
%      figure(1)
%      [x,y,button]=ginput(1)
%       px = round( x+0.5*0)
%       py = round( y+0.5*0)
%             if button==1 
%                  M_(py,px)=1;
%             else
%                  M_(py,px)=0;
%             end
%      imagesc(M_)
%      imax=imax-1;
%  end 

     M_ = rot90(grid_occupancy(:,:,2));
     M_(1,:) = 1;
     M_(end,:) = 1;
     M_(:,1) = 1;
     M_(:,end) = 1;


    % zamenjam nazaj x/y vrstica/stolpec
    for i=1:size(M_,1)
        for j=1:size(M_,2)
             astarG.map(j,i)=M_(i,j);
        end
    end
  astarG.map= fliplr(astarG.map);
 
 close(10) 
 fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none

 
% load MapEnvironment % astarMap=astarG.map; save MapEnvironment astarMap
 % load MapEnvironment % astarMap=astarG.map; save MapEnvironmentJakob astarMap



else
    disp('nalozim demo zemljevid celic')

     % nalozim demo zemljevid celic
    load MapEnvironment  
    astarG.map=astarMap;

    %close(10) 
    fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none

end







 % dolocim diskretno potencialno polje z Dijkstro

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


%return


 %==== primer klica za izracun interpolacije potenciala in gradienta za
 %podano tocko
  tocka=[2,2]; % testna tocka
  cc=tmp_GetInterpolatedBiLinearCostToGoalAndGrad_Test3(tocka(1),tocka(2),CMG,CMGmax,space,resolution);
  smer=-cc(2:3); %cc=[potencial, gradX, gradY]
 %====


% primer izrisov poti (kliknes na zaceteke in ti izrise pot gradienta pokaze pot v smeri negativnega gradienta polja)
if 1
    figure(fig),title('primeri poti, klikaj na graf, ENTER za konec')
    plot(Rob(id).goal(1),Rob(id).goal(2),'rx')
    
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







