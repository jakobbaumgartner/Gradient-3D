function QQ=GetPathByfollowigInterpolatedGradientBilinear(qs,CMG,space,resolution,step)
% returns path obtained from initial pose by following the negative
% gradient direction.
% input:
%       qs             ... start position [x;y]
%       CMG           ... discrete cost map (cost-to-goal)
%       space         ... map dimensions [xmin xmax ymin ymax]
%       resolution    ... numbers of cells in 1 unit (cel with 0.5m side -> resolution is 2)
%       step          ... step length used in integration of teh path (e.g. 0.01)
%
% output:
%       QQ            ... obtained path

QQ=[];

  q=qs;
  CMGmax=max(CMG(CMG~=max(max(CMG))))+.1*1; % second max potential value (max potential of nonoccupied cells)


  %cc=tmp_GetInterpolatedBiLinearCostToGoal2_Test(q(1),q(2),CMG,CMGmax,space,resolution);
  cc=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(q(1),q(2),CMG,CMGmax,space,resolution);

  
  
  QQ=[QQ;[q,cc(1)]];

  x_cen=(space(1)+space(2))/2;   y_cen=(space(3)+space(4))/2; % center of environment

  for i=1:3000
    if (cc(2)~=0 || cc(3)~=0 )
        grad=-cc(2:3)./norm(cc(2:3));
    else
        grad=[x_cen-q(1) , y_cen-q(2)]; % if not defined (e.g. on the space border), set it to show inside
    end
    
    q=q+grad*step; 
 %   cc=tmp_GetInterpolatedBiLinearCostToGoal2_Test(q(1),q(2),CMG,CMGmax,space,resolution);
    cc=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(q(1),q(2),CMG,CMGmax,space,resolution);
    
    QQ=[QQ;[q,cc(1)]];
  end
  
  
% figure(fig),hold on,plot3(QQ(:,1),QQ(:,2),QQ(:,3),'b')
% figure(fig),plot(QQ(:,1),QQ(:,2))
end