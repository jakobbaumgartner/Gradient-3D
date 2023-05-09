classdef AStarClassicMaria < handle
properties
    map = []; % Map: 0 - free space, 1 - obstacles
    open = []; closed = []; start = []; goal = []; act = []; path = [];
    %HIDE
    hFig = [];
    hMap;
    hPath;
    showMode = 0;
    showTs = 0.1;
    
    allNodes=0;
    
    distMode = 'Euclidean'; % 'Euclidean', 'Manhattan', 'eight'
   
    
    lam=[];B=[];dB=[];ddB=[]; % pripravim koeificiente BB polinomov
    
    
    % environment
    x_min = 0;x_max = 10;y_min = 0;y_max = 10; resolution=100; free; obst;
     
    pathCost=nan;
    
    
    %SHOW
end
    
methods
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     function findReplanDijstraFromOpenList(obj, start, goal, OListInit) % start=[x; y], goal=[x; y]
    % function path = findReplan(obj, start, goal, OListInit) % start=[x; y], goal=[x; y]
        
        obj.start =start;   obj.goal = goal; obj.path = [];
                   
        obj.closed = []; % Empty closed list
        obj.open = [];
        obj.allNodes=0;
        heurToGoal=0; % not important for dijkstra, compute heuristic to start since you begin search from goal
        
        for i=1:size(OListInit,1)  % ?? how to initi OpenList, only one best border node or all ring?
              % Initial open list, node structure
              node = struct('id', 1,...% obj.allNodes+1, ... % id ??? all from init OL have the same index since parent (src) is unknown
                          'pose',OListInit(i,1:2)',...
                          'srcId', 0, ... % parent id
                          'srcPose',nan,...
                          'cth', OListInit(i,3), ...  % cost to here 
                          'ctg', Inf); % Not needed for Dijkstra. Cost to goal which would is start (reverse exploring from goal to start) (heuristic)
            
            obj.allNodes=obj.allNodes+1;
            obj.open =[obj.open, node];
            obj.updateDraw(obj.open(end));
        end
        
        
        [~,i] = sortrows([obj.open.cth].', [1]);   % Dijkstra - sort according to costToHere
        obj.open = obj.open(i);

        
        % % Initial open list, node structure
%         obj.open = struct('id', 1, ... % starting node id==1
%                           'pose',goal,...
%                           'srcId', 0, ... % parent id id==0
%                           'srcPose',nan,...
%                           'cth', 0, ...  % cost to here
%                           'ctg', obj.heuristic(goal)); % not in use for Dijkstra. Cost to start since we start plannig from goal (heuristic)
%         obj.allNodes=1;
%        obj.updateDraw(obj.open(end));
        
                        
%          if obj.isPointInFreeSpace(obj.start)==0 || obj.isPointInFreeSpace(obj.goal)==0
%              path = []; 
%              disp('Path not feasible, goal or start not in free space!');
%               return; % Path not feasible!
%          end
        

      jjj=0;
        
  while true % Search loop
  
            jjj=jjj+1;
  
  
            if isempty(obj.open), 
                
                disp('empty open')
                obj.get_path();
                break; 
            
            end % Checked all free cells or no path found :(
            
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.

              obj.pathCost=nan;
              

              
              % extend neigbours
             % dx= (obj.x_max-obj.x_min)/obj.resolution; dy= (obj.y_max-obj.y_min)/obj.resolution;  % increment
               dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution (koliko razdelkov na enoto)
          %   nxx=obj.act.pose(1)+[dx;0;-dx;0]; nyy=obj.act.pose(2)+[0;dy;0;-dy];   % 4 neigbour coordinates
              nxx=obj.act.pose(1)+[dx;dx;0;-dx;-dx;-dx;0;dx]; nyy=obj.act.pose(2)+[0;dy;dy;dy;0;-dy;-dy;-dy];   % 8 neigbour coordinates
             
              for j=1:length(nxx)  % add neighbours
                   if obj.addNodeToOpenListG(nxx(j), nyy(j),heurToGoal)  
                        obj.updateDraw(obj.open(end));  % draw the last node (new added node) 
                        % Sort open list % % GK?? tukaj vplivas na lastnost
                        % iskanja e.g. za greedy:  .3*cth+.7*ctg
                        % to poka�i na predavanjih .3*cth+.7*ctg  ali
                        % .7*cth+.3*ctg
                  %      [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*1; obj.open.ctg].', [1,2]);   
                       % [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*0].', [1]);  
                       [~,i] = sortrows([obj.open.cth].', [1]);   % Dijkstra - sort according to costToHere
                       obj.open = obj.open(i);
                   end
              end
     end
      %  path = obj.path;
    end

    
    
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    function path = findDijkstraFromGoal(obj, start, goal) % start=[x; y], goal=[x; y]
        
        obj.start =start;   obj.goal = goal; obj.path = [];
                   
        heurToGoal=0; % compute heuristic to start since you begin search from goal


        obj.closed = []; % Empty closed list
        obj.open = [];

        
        % % Initial open list, node structure
        obj.open = struct('id', 1, ... % starting node id==1
                          'pose',goal,...
                          'srcId', 0, ... % parent id id==0
                          'srcPose',nan,...
                          'cth', 0, ...  % cost to here
                          'ctg', obj.heuristic(start,heurToGoal) ); % cost to goal (heuristic)
        obj.allNodes=1;
        obj.updateDraw(obj.open(end));
        
                        
         if obj.isPointInFreeSpace(obj.start)==0 || obj.isPointInFreeSpace(obj.goal)==0
             path = []; 
             disp('Path not feasible, goal or start not in free space!');
              return; % Path not feasible!
         end
        

      jjj=0;
        
      while true % Search loop
  
            jjj=jjj+1;
          %  if(jjj>1000), disp('1000 iterations reached'); break; end
  
  
            if isempty(obj.open), 
                
                disp('empty open')
                obj.get_path();
                break; 
            
            end % No path found :(
            
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.

 % GK: vzratno poisci pot           
                
              % chech that sufficiently close to goal ???
              obj.pathCost=nan;
              
           %  px = round( (obj.goal(1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1)+1);
           %  py = round( (obj.goal(2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1)+1);

              
         %     dist=sqrt (sum( (obj.act.pose(1:2) - obj.goal(1:2)).^2 ));
         %     dist=sqrt (sum( (obj.act.pose(1:2) - obj.start(1:2)).^2 ));

           %  dist=sqrt (sum( ([px;py] - obj.act.pose).^2 ));
%               if abs(dist)<0.001   % ??? GK to popravi
%                    obj.pathCost=obj.act.cth+obj.act.ctg;
%                    p = obj.act.id; obj.path = [p]; ids = [obj.closed.id];
%                 while (p~=1) % Follow src nodes to the start
%                     p = obj.closed(ids==p).srcId;
%                     obj.path = [p, obj.path];
%                 end
%                 break;
% 
%               end

              
              % extend neigbours
             % dx= (obj.x_max-obj.x_min)/obj.resolution; dy= (obj.y_max-obj.y_min)/obj.resolution;  % increment
               dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution (koliko razdelkov na enoto)
          %   nxx=obj.act.pose(1)+[dx;0;-dx;0]; nyy=obj.act.pose(2)+[0;dy;0;-dy];   % 4 neigbour coordinates
              nxx=obj.act.pose(1)+[dx;dx;0;-dx;-dx;-dx;0;dx]; nyy=obj.act.pose(2)+[0;dy;dy;dy;0;-dy;-dy;-dy];   % 8 neigbour coordinates
             
              for j=1:length(nxx)  % add neighbours
                 
                 % check if diagonal transition is possible
                 go=1; 
                 if     (j==2 && (obj.isPointInFreeSpace([nxx(1);nyy(1)])==0 && obj.isPointInFreeSpace([nxx(3);nyy(3)])==0 ) ) 
                    go=0; 
                 elseif (j==4 && (obj.isPointInFreeSpace([nxx(3);nyy(3)])==0 && obj.isPointInFreeSpace([nxx(5);nyy(5)])==0 ) ) 
                    go=0; 
                 elseif (j==6 && (obj.isPointInFreeSpace([nxx(5);nyy(5)])==0 && obj.isPointInFreeSpace([nxx(7);nyy(7)])==0 ) ) 
                    go=0; 
                 elseif (j==8 && (obj.isPointInFreeSpace([nxx(7);nyy(7)])==0 && obj.isPointInFreeSpace([nxx(1);nyy(1)])==0) ) 
                    go=0; 
                 end
                    
                   if go && obj.addNodeToOpenListG(nxx(j), nyy(j),heurToGoal) 
                        obj.updateDraw(obj.open(end));  % draw the last node (new added node) 
                        % Sort open list % % GK?? tukaj vpliva� na lastnost
                        % iskanja e.g. za greedy:  .3*cth+.7*ctg
                        % to poka�i na predavanjih .3*cth+.7*ctg  ali
                        % .7*cth+.3*ctg
                  %      [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*1; obj.open.ctg].', [1,2]);   
                       % [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*0].', [1]);  
                        [~,i] = sortrows([obj.open.cth].', [1]);   % Dijkstra - sort according to costToHere
                        obj.open = obj.open(i);
                   end
                   
              end
              
        end
        path = obj.path;
    end

%==========================================================================    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    function path = findAstarFromStart(obj, start, goal) % start=[x; y], goal=[x; y]
        
        obj.start =start;   obj.goal = goal; obj.path = [];
        heurToGoal=1; % search is from start  so compute heuristic to goal         
        

        obj.closed = []; % Empty closed list
        obj.open = [];

        
        % % Initial open list, node structure
        obj.open = struct('id', 1, ... % starting node id==1
                          'pose',start,...
                          'srcId', 0, ... % parent id id==0
                          'srcPose',nan,...
                          'cth', 0, ...  % cost to here
                          'ctg', obj.heuristic(start,heurToGoal) ); % cost to goal (heuristic)
        obj.allNodes=1;
        obj.updateDraw(obj.open(end));
        
                        
%          if obj.isPointInFreeSpace(obj.start)==0 || obj.isPointInFreeSpace(obj.goal)==0
%              path = []; 
%              disp('Path not feasible, goal or start not in free space!');
%               return; % Path not feasible!
%          end
         if obj.isPointInFreeSpace(obj.start)==0 
             path = []; 
             disp('Path not feasible,  start not in free space!');
              return; % Path not feasible!
         end
      

      jjj=0;
        
      while true % Search loop
  
            jjj=jjj+1;
          %  if(jjj>1000), disp('1000 iterations reached'); break; end
  
  
            if isempty(obj.open), 
                
                disp('empty open')
                obj.get_path();
                break; 
            
            end % No path found :(
            
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.

 % GK: vzratno poisci pot           
                
              % chech that sufficiently close to goal ???
              obj.pathCost=nan;
              
           %  px = round( (obj.goal(1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1)+1);
           %  py = round( (obj.goal(2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1)+1);

              
         %     dist=sqrt (sum( (obj.act.pose(1:2) - obj.goal(1:2)).^2 ));
         %     dist=sqrt (sum( (obj.act.pose(1:2) - obj.start(1:2)).^2 ));

           %  dist=sqrt (sum( ([px;py] - obj.act.pose).^2 ));
%               if abs(dist)<0.001   % ??? GK to popravi
%                    obj.pathCost=obj.act.cth+obj.act.ctg;
%                    p = obj.act.id; obj.path = [p]; ids = [obj.closed.id];
%                 while (p~=1) % Follow src nodes to the start
%                     p = obj.closed(ids==p).srcId;
%                     obj.path = [p, obj.path];
%                 end
%                 break;
% 
%               end

              
              % extend neigbours
             % dx= (obj.x_max-obj.x_min)/obj.resolution; dy= (obj.y_max-obj.y_min)/obj.resolution;  % increment
               dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution (koliko razdelkov na enoto)
          %   nxx=obj.act.pose(1)+[dx;0;-dx;0]; nyy=obj.act.pose(2)+[0;dy;0;-dy];   % 4 neigbour coordinates
              nxx=obj.act.pose(1)+[dx;dx;0;-dx;-dx;-dx;0;dx]; nyy=obj.act.pose(2)+[0;dy;dy;dy;0;-dy;-dy;-dy];   % 8 neigbour coordinates
             
              for j=1:length(nxx)  % add neighbours
                   if obj.addNodeToOpenListG(nxx(j), nyy(j),heurToGoal) 
                        obj.updateDraw(obj.open(end));  % draw the last node (new added node) 
                        % Sort open list % % GK?? tukaj vpliva? na lastnost
                        % iskanja e.g. za greedy:  .3*cth+.7*ctg
                        % to poka?i na predavanjih .3*cth+.7*ctg  ali
                        % .7*cth+.3*ctg
                        [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*1; obj.open.ctg].', [1,2]);   
                       % [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*0].', [1]);  
                %        [~,i] = sortrows([obj.open.cth].', [1]);   % Dijkstra - sort according to costToHere
                        obj.open = obj.open(i);
                   end
              end
              
        end
        path = obj.path;
    end

    
%==========================================================================        
%==========================================================================    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    function path = findAstarFromStartOriginal(obj, start, goal) % start=[x; y], goal=[x; y]
        
        obj.start =start;   obj.goal = goal; obj.path = [];
        heurToGoal=1; % search is from start  so compute heuristic to goal         
        

        obj.closed = []; % Empty closed list
        obj.open = [];

        
        % % Initial open list, node structure
        obj.open = struct('id', 1, ... % starting node id==1
                          'pose',start,...
                          'srcId', 0, ... % parent id id==0
                          'srcPose',nan,...
                          'cth', 0, ...  % cost to here
                          'ctg', obj.heuristic(start,heurToGoal) ); % cost to goal (heuristic)
        obj.allNodes=1;
        obj.updateDraw(obj.open(end));
        
                        
         if obj.isPointInFreeSpace(obj.start)==0 || obj.isPointInFreeSpace(obj.goal)==0
             path = []; 
             disp('Path not feasible, goal or start not in free space!');
              return; % Path not feasible!
         end
         if obj.isPointInFreeSpace(obj.start)==0 
             path = []; 
             disp('Path not feasible,  start not in free space!');
              return; % Path not feasible!
         end
      

      jjj=0;
        
      while true % Search loop
  
            jjj=jjj+1;
          %  if(jjj>1000), disp('1000 iterations reached'); break; end
  
  
            if isempty(obj.open), 
                
                disp('empty open')
                obj.get_path();
                break; 
            
            end % No path found :(
            
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.

 % GK: vzratno poisci pot           
                
              % chech that sufficiently close to goal ???
              obj.pathCost=nan;
              
           % px = round( (obj.goal(1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1)+1);
          %  py = round( (obj.goal(2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1)+1);

              
            dist=sqrt (sum( (obj.act.pose(1:2) - obj.goal(1:2)).^2 ));
            %dist=sqrt (sum( (obj.act.pose(1:2) - obj.start(1:2)).^2 ));

         %   dist=sqrt (sum( ([px;py] - obj.act.pose).^2 ));
              if abs(dist)<0.001   % ??? GK to popravi
                   obj.pathCost=obj.act.cth+obj.act.ctg;
                   p = obj.act.id; obj.path = [p]; ids = [obj.closed.id];
                while (p~=1) % Follow src nodes to the start
                    p = obj.closed(ids==p).srcId;
                    obj.path = [p, obj.path];
                end
                break;

              end

              
              % extend neigbours
             % dx= (obj.x_max-obj.x_min)/obj.resolution; dy= (obj.y_max-obj.y_min)/obj.resolution;  % increment
               dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution (koliko razdelkov na enoto)
          %   nxx=obj.act.pose(1)+[dx;0;-dx;0]; nyy=obj.act.pose(2)+[0;dy;0;-dy];   % 4 neigbour coordinates
              nxx=obj.act.pose(1)+[dx;dx;0;-dx;-dx;-dx;0;dx]; nyy=obj.act.pose(2)+[0;dy;dy;dy;0;-dy;-dy;-dy];   % 8 neigbour coordinates
             
              for j=1:length(nxx)  % add neighbours
                   if obj.addNodeToOpenListG(nxx(j), nyy(j),heurToGoal) 
                        obj.updateDraw(obj.open(end));  % draw the last node (new added node) 
                        % Sort open list % % GK?? tukaj vpliva� na lastnost
                        % iskanja e.g. za greedy:  .3*cth+.7*ctg
                        % to poka�i na predavanjih .3*cth+.7*ctg  ali
                        % .7*cth+.3*ctg
                        [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*1; obj.open.ctg].', [1,2]);   
                       % [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*0].', [1]);  
                %        [~,i] = sortrows([obj.open.cth].', [1]);   % Dijkstra - sort according to costToHere
                        obj.open = obj.open(i);
                   end
              end
              
        end
        path = obj.path;
    end

    
%==========================================================================    
 
      function get_path(obj)

        pp=[obj.closed.pose];
        if ~isempty(pp)
            z=abs(pp(1,:)-obj.start(1))<0.0001 & abs(pp(2,:)-obj.start(2))<0.0001;
            if isempty(find(z, 1)), return , end;
            s= find(z, 1);
            a=obj.closed(s);
            obj.pathCost=a.cth;

            p = a.id; obj.path = [p]; ids = [obj.closed.id];
            while (p~=1) % Follow src nodes to the goal
                p = obj.closed(ids==p).srcId;
                obj.path = [p, obj.path];
            end
             
        end                
    end  
    
 %=========================================================================   
    
    function added=addNodeToOpenListG(obj,xF, yF,heurToGoal)
    
        
        if obj.isPointInFreeSpace([xF;yF])==0  
            added=0; 
            return;
        end; % node not possible

        xS=obj.act.pose(1);    yS=obj.act.pose(2); % starting pose (parent node)        
        deltaX = xF-xS; deltaY = yF-yS;
        pose=[xF;yF];
        cth= obj.act.cth + (sqrt(deltaX^2+deltaY^2));
        ctg= obj.heuristic(pose,heurToGoal);
     
        
        pp=[obj.closed.pose];  % check if it is in the closed list
        z=pp(1,:)==xF & pp(2,:)==yF; 
        s= find(z, 1);
        if ~isempty(find(s, 1))
            if cth<obj.closed(s).cth &&0  % has better cost than node in the closed list
                obj.closed(s).cth = cth;
                obj.closed(s).srcId = obj.act.id;
                obj.closed(s).srcPose = obj.act.pose;
                
                obj.open = [obj.open, obj.closed(s)]; % move back to open list
                obj.closed = [obj.closed(1:s-1), obj.closed(s+1:end)]; % remove from closed list
                added=1; % was updated 
                return;
            else
               added=0; 
               return; 
            end
        end;  %is the pose already in the closed list?
        

        
        
        pp=[obj.open.pose]; s=[];
        if ~isempty(pp)
            z= pp(1,:)==xF & pp(2,:)==yF;
            s= find(z, 1);
        end   
        
        if isempty(find(s, 1)) % Add new node to the open list
               % add new node to open list
              node = struct('id', obj.allNodes+1, ... % id
                          'pose',pose,...
                          'srcId', obj.act.id, ... % parent id
                          'srcPose',obj.act.pose,...
                          'cth', cth, ...  % cost to here 
                          'ctg', ctg); % cost to goal (heuristic)
              obj.allNodes=obj.allNodes+1;
              obj.open = [obj.open, node];
              added=1; % node was added           
         else % Update the node in the open list if it has better score
            if cth<obj.open(s).cth
                obj.open(s).cth = cth;
                obj.open(s).srcId = obj.act.id;
                obj.open(s).srcPose = obj.act.pose;
                added=1; % was updated 
                return;
            end
            added=0;  
        end   
        
    end
 
 %=========================================================================   
    
    function h = heuristic(obj, a,heurToGoal)
           % h = sum(abs(a-obj.goal)); % Manhattan   
           if(heurToGoal)
            h = sqrt(sum((a(1:2)-obj.goal(1:2)).^2)); % Euclidean
           else
            h = sqrt(sum((a(1:2)-obj.start(1:2)).^2)); % Euclidean
           end
    end   
   %=========================================================================   
    

    
    function updateDraw(obj,node)
        if obj.showMode==0, return; end

        figure(10),hold on
         
       if obj.showMode~=30  
                if(node.id~=1)                    
                    
                    r= [node.srcPose(1:2,1),node.pose(1:2,1)];
                   
                    plot(node.pose(1), node.pose(2),'rx', r(1,:),r(2,:),'c'); hold on %path sections  and nodes
                else
                    plot(node.pose(1), node.pose(2),'rx'); hold on
                end
       end     
             if  obj.showMode==30   
               text(node.pose(1)-0.2, node.pose(2),num2str( node.cth,2 ), 'FontSize',7) 
             elseif obj.showMode==3   
               text(node.pose(1), node.pose(2),num2str( node.cth,3 ), 'FontSize',8) 

             elseif obj.showMode==4
               text(node.pose(1), node.pose(2),num2str( node.ctg,3 ), 'FontSize',8)  
             elseif obj.showMode==5
               text(node.pose(1), node.pose(2),num2str( node.cth+node.ctg,3 ), 'FontSize',8)  
             end
    end

    
    
  %=========================================================================   

    
    
   
     function environment(obj,limits,res, Rect)
            obj.x_min = limits(1);obj.x_max = limits(2);obj.y_min = limits(3); obj.y_max = limits(4);           
            obj.resolution = res; % res=100 -> 100 razdelkov na enoto
            y_delta = obj.y_max - obj.y_min;
            x_delta = obj.x_max - obj.x_min;
            obj.free = 0; obj.obst = 1;
            obj.map = ones(ceil(y_delta*res),ceil(x_delta*res))*obj.free;
            obj.map(1,:)=obj.obst; obj.map(end,:)=obj.obst; obj.map(:,1)=obj.obst; obj.map(:,end)=obj.obst;
            

            
            for i=1:size(Rect,1)
                 obj.map( ceil((Rect(i,1)-obj.x_min)*res+1):ceil((Rect(i,2)-obj.x_min)*res),...
                          ceil((Rect(i,3)-obj.y_min)*res+1):ceil((Rect(i,4)-obj.y_min)*res) ) = obj.obst; 
                 

                % draw environment and obstacles  
%                rectangle('Position',[Rect(i,1), Rect(i,3), Rect(i,2)-Rect(i,1), Rect(i,4)-Rect(i,3)],'EdgeColor','k');
            end
            
  if obj.showMode~=0 % izrisem mapo za test
            figure(10),hold on
  
            Nx=size(obj.map,1);
            Ny=size(obj.map,2);
        
        for g=1:Nx
            for h=1:Ny 
                
                dd= 1/obj.resolution/2; %increment
                gx=   1/obj.resolution*g-dd + obj.x_min;
                gy=   1/obj.resolution*h-dd + obj.y_min;
                
                
                nx=gx+[-dd,dd,dd,-dd, -dd]; ny=gy+[-dd,-dd,dd,dd,-dd];   % cell vertices

                if(obj.isPointInFreeSpace([gx;gy]) )
                    line(nx,ny,'Color',[0.9 0.9 0.9]); %
                else
                    % patch(nx,ny,[0.7 0.7 0.7]); %promijenila iz zelene u sivo prepreke
                     patch(nx,ny,[0.7 0.7 0.7],'EdgeColor','None');
                end
                 
        
            end
        end

            for i=1:size(Rect,1)
  
                % draw environment and obstacles  
                rectangle('Position',[Rect(i,1), Rect(i,3), Rect(i,2)-Rect(i,1), Rect(i,4)-Rect(i,3)],'EdgeColor','k');
            end
 end      
            
            
     end

 %=========================================================================   
   
     function ok = isPointInFreeSpace(obj, point)
            if point(1)>obj.x_max || point(1)<obj.x_min || point(2)>obj.y_max || point(2)<obj.y_min
                ok = 0;
                return;
            end
            px =  (point(1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1)+1;
            py =  (point(2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1)+1;
            
            if obj.map(round(px),round(py))==obj.free 
                ok = 1;
            else
                ok = 0;
            end
                   
     end

 %=========================================================================   
   
     function rr=drawFinalPath(obj,color,fig,cross)
        % cross - do I need connections and crosses to be drawn 
        p=obj.path; rr=[];
        ids=[ obj.closed.id];
        figure(fig);hold on
        
        
        for i=1:length(p)
            ind = (ids==p(i));
            node=obj.closed(ind);
            
            if(p(i)>1)
                r= [node.srcPose(1:2,1),node.pose(1:2,1)];
%                 if color==1
%                   plot(r(1,:),r(2,:),'r--','LineWidth',2 ); 
%                 else
%                   plot(r(1,:),r(2,:),'b--','LineWidth',2 );
%                 end
                 plot(r(1,:),r(2,:),color,'LineWidth',2 ); 

            end
            
            % draw cells of the path           
            xx=node.pose(1,1); yy=node.pose(2,1);            
       if(cross)     
            dd= 1/obj.resolution/2; %increment
            nx=xx+[-dd,dd,dd,-dd, -dd]; ny=yy+[-dd,-dd,dd,dd,-dd];   % cell vertices
            line(nx,ny);
       end      
            
            
            rr=[rr;xx,yy]; % store points of the optimal path
            
        end
     end
 %=========================================================================   
     
     
       function rr=getFinalPath(obj)
        % cross - do I need connections and crosses to be drawn 
        p=obj.path; rr=[];
        ids=[ obj.closed.id];
        
        
        for i=1:length(p)
            ind = (ids==p(i));
            node=obj.closed(ind);
            
            if(p(i)>1)
                r= [node.srcPose(1:2,1),node.pose(1:2,1)];

            end
            
            % draw cells of the path           
            xx=node.pose(1,1); yy=node.pose(2,1);            
            
            
            rr=[rr;xx,yy]; % store points of the optimal path
            
        end
     end   
     
 %=========================================================================   
  
    function rr=mapToGrid(obj,point)  % just maps to the center of the grid
            px = round( (point(1,1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1*0)+0.5*1);
            py = round( (point(2,1)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1*0)+0.5*1);
            
            rr=[0;0];
            rr(1)=   1/obj.resolution*px - 1/obj.resolution/2 + obj.x_min;
            rr(2)=   1/obj.resolution*py - 1/obj.resolution/2 + obj.y_min;
    end 
     
 %==========================================================================    
    function newObstacle=addObstaclesToMap(obj,obst)  % adds obstacle points as ocsuppied cells, returns newObstacle==1 if new cell is blocekd
        newObstacle=0;   
        for i=1:size(obst,1)
           if( obj.isPointInFreeSpace(obst(i,:)) ) 
             px = round( (obst(i,1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1*0)+0.5*1);
             py = round( (obst(i,2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1*0)+0.5*1);
             obj.map(px,py)=obj.obst;
             newObstacle=1;
           end
        end
    end 
 %==========================================================================    
 
     function redrawMap(obj,fig,CMG)  % adds obstacle points as occuppied cells, returns newObstacle==1 if new cell is blocekd
        figure(fig);hold on
        Nx=size(obj.map,1);
        Ny=size(obj.map,2);
        
        drawCosts=~isempty(CMG);
        
        for g=1:Nx
            for h=1:Ny 
                dd= 1/obj.resolution/2; %increment
                gx=   1/obj.resolution*g-dd + obj.x_min;
                gy=   1/obj.resolution*h-dd + obj.y_min;
                nx=gx+[-dd,dd,dd,-dd, -dd]; ny=gy+[-dd,-dd,dd,dd,-dd];   % cell vertices
                if(obj.isPointInFreeSpace([gx;gy]) )
                     line(nx,ny,'Color',[0.9 0.9 0.9]) 
                     if drawCosts
                        %text(gx,gy,num2str( CMG(g,h),3 ), 'FontSize',7)
                        text(gx-0.2,gy,num2str( CMG(g,h),3 ), 'FontSize',6)
                        
                     end
                else
                   % text(gx,gy,num2str( sample ), 'FontSize',6)
                    patch(nx,ny,[0.7 0.7 0.7],'EdgeColor','None'); 
                end
            end
        end
     end
 %==========================================================================    

 
     
 function drawClosedListAndConnections(obj,fig)  % just draws the result on figure
    
        figure(fig) 
        %draw grid first
        Nx=size(obj.map,1);Ny=size(obj.map,2);
        
        for g=1:Nx
            for h=1:Ny 
                dd= 1/obj.resolution/2; %increment
                gx=   1/obj.resolution*g-dd + obj.x_min;
                gy=   1/obj.resolution*h-dd + obj.y_min;
                                
                nx=gx+[-dd,dd,dd,-dd, -dd]; ny=gy+[-dd,-dd,dd,dd,-dd];   % cell vertices
                if(obj.isPointInFreeSpace([gx;gy]) )
                     line(nx,ny,'Color',[0.9 0.9 0.9 0.9]) 
                else
                    patch(nx,ny,[0.7 0.7 0.7],'EdgeColor','None'); %za clanak zakomentirala % GK odkomentiral
                end
            end
        end
     
       hold on 
           for i=1:length(obj.closed)
                node=obj.closed(i);
      
                if(node.id~=1)
                    r= [node.srcPose(1:2,1),node.pose(1:2,1)];
                    plot(node.pose(1), node.pose(2),'rx', r(1,:),r(2,:),'c'); hold on %path sections  and nodes
                else
                    plot(node.pose(1), node.pose(2),'rx'); hold on
                end
                
               text(node.pose(1), node.pose(2),num2str( node.cth,3 ), 'FontSize',8) 
           end

 end 
  
%============================================================
function CM=getCostMapFromClosedList(obj)  
              CM=ones(size(obj.map))*50000;
              for i=1:length(obj.closed)
                node=obj.closed(i);
                xxc=node.pose(1);
                yyc=node.pose(2);
                % dobim pointer na matriko okolja in CML napolnim s cenami  
                px = round( (xxc-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1))+0.5);
                py = round( (yyc-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2))+0.5);
                CM(px,py)= obj.closed(i).cth;
              end
end
%============================================================
 
 
 
   
end

end