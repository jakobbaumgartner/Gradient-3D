clear all, close all
classical = Classical_optimizer();

%trajec = trajectorys();
%points = trajec.trajectory1(0);

% robot initial values
q0 = [0 ...
    pi/4 ...
    0 ...
    -pi/3 ...
    0 ...
    1.8675 ...
    0 ...
    0 ... 
    0 ...
    0];

%goalPose = points(:,:,21);

goalPose =[1.0000         0         0    0.4000
         0   -0.7071   -0.7071    1.0000
         0    0.7071   -0.7071    1.4000
         0         0         0    1.0000];
     
     
% goalPose =[1.0000    0         0    0.4000   % orintacija prijemala v z osi
%            0         1         0    1.0000
%            0         0         1    1.4000
%            0         0         0    1.0000];

goalPose =[0         0         1    0.4000   % orintacija v x osi
           0         1         0    1.0000
          -1         0         0    1.4000
           0         0         0    1.0000];
     
q0(8:10)=[0 .3+.1  pi/2*.3]; % popravi bazo da je blizje referenci


classical.dt=0.1;  % sample time


scenarij=2;   % 1=premica, 2=srednja pot, 3=dolga pot,  4=kratka pot

if(scenarij==1)
    q0(8:10)=[3.5 1.2  pi/3]; % popravi bazo da je blizje referenci
   % goalPose(1:3,4)=[3.3;1.5;1];  % popravi roko da je blizje referenci

    %===== referenca za roko
   t=0:classical.dt:21;   start=[3.5 1.5]; goal=[7 9]+[0 1]*1; % ok za mode 1 ali 13 in zamik xar=+0.2
   w=2*2*pi/3;
   
    xar= start(1) + (goal(1)-start(1))/t(end)*t +.2*3;  % zamikam x koordinato *2, *3,...
   yar= start(2) + (goal(2)-start(2))/t(end)*t ;  
   zar=1+0.1*sin(0.25*w*t) +.1; % referenca za roko
   dxar=(goal(1)-start(1))/t(end)*ones(size(t)); 
   dyar=(goal(2)-start(2))/t(end)*ones(size(t)); 
   dzar=0.1*0.25*w*cos(0.25*w*t);  % odvodi reference za roko


   armRef= [t;xar;yar;zar;dxar;dyar;dzar];
    %========

    %===== potencialno polje za bazo
    %goalBase=[6.4+.5+2; 9.5+.5*3] +[-1 ;2]*0  
    goalBase=[xar(end); yar(end)];
    Skript_planiranjeBaze; % dolocim potencialno polje za bazo
    %=====
elseif(scenarij==-1)  % spirala
    q0(8:10)=[3.5 1.2  pi/3]; % popravi bazo da je blizje referenci
   % goalPose(1:3,4)=[3.3;1.5;1];  % popravi roko da je blizje referenci

    %===== referenca za roko
   t=0:classical.dt:21;   start=[3.5 1.5]; goal=[7 9]+[0 1]*1; % ok za mode 1 ali 13 in zamik xar=+0.2
   w=2*2*pi/3;
   
   xar= start(1) + (goal(1)-start(1))/t(end)*t +.2*1+0.2*cos(0.25*w*t);  % zamikam x koordinato *2, *3,...
   yar= start(2) + (goal(2)-start(2))/t(end)*t     + 0.2*sin(0.3*w*t)*0;  
   zar=1+0.2*sin(0.25*w*t) +.1; % referenca za roko
   dxar=(goal(1)-start(1))/t(end)*ones(size(t)) - 0.2*0.25*w*sin(0.25*w*t); 
   dyar=(goal(2)-start(2))/t(end)*ones(size(t)) +0.2*0.25*w*cos(0.25*w*t)*0; 
   dzar=0.2*0.25*w*cos(0.25*w*t);  % odvodi reference za roko

   armRef= [t;xar;yar;zar;dxar;dyar;dzar];
    %========

    %===== potencialno polje za bazo
    %goalBase=[6.4+.5+2; 9.5+.5*3] +[-1 ;2]*0  
    goalBase=[xar(end); yar(end)];
    Skript_planiranjeBaze; % dolocim potencialno polje za bazo
    %=====

elseif scenarij==2
   q0(8:10)=[3.7-.4 1.8+.1  pi/3]; % popravi bazo da je blizje referenci
  % goalPose(1:3,4)=[3.3;1.5;1];  % popravi roko da je blizje referenci

   %===== referenca za roko
      w=2*2*pi/3;
      a=[0.1065   -1.6405    9.1195  -15.2290]; % y=f(x) polinom 3. reda
      t=0:classical.dt:16.5;
      
      xar=4+t*4/15;
      yar=polyval(a,xar); % y=f(x)
      zar=1+0.1*sin(0.25*w*t) +.1; % referenca za roko
      dxar=4/15*ones(size(t)); 
      dyar=(3*a(1)*xar.^2+2*a(2)*xar+a(3)*ones(size(t)))*4/15; 
      dzar=0.1*0.25*w*cos(0.25*w*t);  % odvodi reference za roko

      armRef= [t;xar;yar;zar;dxar;dyar;dzar];
    %========

    %===== potencialno polje za bazo
    goalBase=[8;8.5] 
    %goalBase=[xar(end); yar(end)];
    Skript_planiranjeBaze2; % dolocim potencialno polje za bazo
    %=====


elseif scenarij==3
   q0(8:10)=[3.7-.4 1.8+.1  pi/3]; % popravi bazo da je blizje referenci
   % polinom 5.reda
   load poly5Ref
  % ax=[ -0.0000    0.0000   -0.0001    0.0011   -0.0137    0.0935   -0.3469    0.5411    0.5752 3.8034];
  % ay=[-0.0000    0.0000   -0.0000    0.0001    0.0001   -0.0082    0.0598   -0.0599    0.1596    2.1558];
   u=0:0.05*classical.dt/0.1:20;
   t=2*u;
 
   
     %===== referenca za roko
      w=2*2*pi/3;
      
      xar=polyval(ax,u);
      yar=polyval(ay,u); 
      zar=1+0.1*sin(0.25*w*t) +.1; % referenca za roko
      dxar=0.5*(ax(1)*5*u.^4+ax(2)*4*u.^3+ax(3)*3*u.^2+ax(4)*2*u.^1+ax(5)*ones(size(t))); 
      dyar=0.5*(ay(1)*5*u.^4+ay(2)*4*u.^3+ay(3)*3*u.^2+ay(4)*2*u.^1+ay(5)*ones(size(t)));     
      dzar=0.1*0.25*w*cos(0.25*w*t);  % odvodi reference za roko
      dzar=0.1*0.25*w*cos(0.25*w*t);  % odvodi reference za roko

      armRef= [t;xar;yar;zar;dxar;dyar;dzar];
    %========


   
%    
% if 0 % za dolocitev polinoma
%       a=[0.1065   -1.6405    9.1195  -15.2290]; % y=f(x) polinom 3. reda
%       t=0:classical.dt:10;
%       
% 
%       figure(10),plot(g(:,1),g(:,2),'.',xar,yar,'r')
%    %   figure(10),plot(xar,yar,'b')
%    
%   g=[]; 
%    while 1
%       % figure(10)
%       g_=ginput(1);
%       g=[g;g_];
%       figure(10),plot(g_(1),g_(2),'.')
%    end   
%    
%       u=linspace(0, 20, length(g));
%       ax=polyfit(u,g(:,1),5);
%       ay=polyfit(u,g(:,2),5);
%       %save poly5Ref ax ay
%       %save poly5RefScen4 ax ay
%       xx=polyval(ax,u);
%       yy=polyval(ay,u);
%     %  figure(10),plot(g(:,1),g(:,2),'.',xx,yy,'r')
%       figure(1),plot(g(:,1),g(:,2),'.',xx,yy,'r')
% 
%   end
 
   

    %===== potencialno polje za bazo
   % goalBase=[8;8.5] 
    goalBase=[xar(end); yar(end)];
    Skript_planiranjeBaze2; % dolocim potencialno polje za bazo
    %=====
   
    
elseif scenarij==4
   q0(8:10)=[3.7-.4 1.8+.1  pi/3]; % popravi bazo da je blizje referenci
   % polinom 5.reda
   load poly5RefScen4
   u=0:1/30*classical.dt/0.1:10;
   t=3*u;
 
   
     %===== referenca za roko
      w=2*2*pi/3;
      
      xar=polyval(ax,u);
      yar=polyval(ay,u); 
      zar=1+0.1*sin(0.25*w*t) +.1; % referenca za roko
      dxar=1/3*(ax(1)*5*u.^4+ax(2)*4*u.^3+ax(3)*3*u.^2+ax(4)*2*u.^1+ax(5)*ones(size(t))); 
      dyar=1/3*(ay(1)*5*u.^4+ay(2)*4*u.^3+ay(3)*3*u.^2+ay(4)*2*u.^1+ay(5)*ones(size(t)));     
      dzar=0.1*0.25*w*cos(0.25*w*t);  % odvodi reference za roko
      dzar=0.1*0.25*w*cos(0.25*w*t);  % odvodi reference za roko

      armRef= [t;xar;yar;zar;dxar;dyar;dzar];
    %========

    %===== potencialno polje za bazo
   % goalBase=[8;8.5] 
    goalBase=[xar(end); yar(end)];
    Skript_planiranjeBaze2; % dolocim potencialno polje za bazo
    %=====
    
    
    
   
end


%return

figure(10),set(gca,'view',[-0.6792   90.0000]);  %get(gca,'view')
%set(gcf,'Position',[664   188   904   685]); %get(gcf,'Position')



 % flagSecundar
 % 0-le primarna, 
 % 1=primarna +sekundarna za v in w , 
 % 12=kot 1 le da še manipulab za sklepe, 
 % 13= dodana se primarna in teciarna naloga (1. naloga = hitrost EE, 2. naloga =hitrost baze,3. naloga je optimizacija manipulabilnosti)

 % flagVLimit=1; =1 ima omejitve hitrosti in pospeska, =0 brez omejitev 
 % flagOnlyPositionEE=0; %=1 z orientacijo EE, =0 brez orientacije EE

 
flagSecundar=13;flagVLimit=1; flagOnlyPositionEE=1; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
%flagSecundar=1;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2 goalA=[7 10]



if scenarij==1 || scenarij==-1
   % flagSecundar=13;flagVLimit=1; flagOnlyPositionEE=1; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
    flagSecundar=13;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
  %  flagSecundar=1;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]

elseif scenarij==2
    flagSecundar=13;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]

elseif scenarij==3
 %   flagSecundar=13;flagVLimit=1; flagOnlyPositionEE=1; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
  %  flagSecundar=13;flagVLimit=0; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
    flagSecundar=13;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
elseif scenarij==4
    flagSecundar=0;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]
   % flagSecundar=1;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]

end


classical.optimizerTrajectoryPotencialBase(q0, goalPose,flagSecundar,flagVLimit, CMG,CMGmax,space,resolution,flagOnlyPositionEE,armRef); %dodal animacijo in par sprememb


if 0

  risiRezultate1  % za clanek izrise rezultate

  

   %  print -depsc  ideaLayoutPotentialField;   

   fs=14;  xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)

   fig=12; astarG.redrawMap(fig,CMG); % draw map on figure, add cost map CMG or put [] for none
   % axis equal , axis([0 9 1 10]); print -depsc  ideaLayoutDiscretePotential
end


if 0  % slike za clanek
    
   figure(11),
   %print -depsc  trackingShort_potential 
   %print -depsc  trackingMid_potential 
   %print -depsc  trackingLong_potential 
  
    
   risiRezultate1  % za clanek izrise rezultate
   
   figure(10), 
   set(gca,'view',[-27.1594   29.4140]) 
   %print -depsc  trackingShort_side    
   %print -depsc  trackingMid_side    
   %print -depsc  trackingLong_side    
   
   figure(10), 
   set(gca,'view',[0   90]) 
   %print -depsc  trackingShort_top    
   %print -depsc  trackingMid_top    
   %print -depsc  trackingLong_top    
 
   %print -depsc  trackingMidNoise_top    


   
   figure(1), 
   %print -depsc  trackingShort_vw 
   %print -depsc  trackingMid_vw 
   %print -depsc  trackingLong_vw 

   %print -depsc  trackingMidNoise_vw 

   
   figure(2), 
   %print -depsc  trackingShort_jointVel 
   %print -depsc  trackingMid_jointVel 
   %print -depsc  trackingLong_jointVel 
   
   %print -depsc  trackingMidNoise_jointVel 

    figure(5), 
   %print -depsc  trackingShort_err 
   %print -depsc  trackingMid_err 
   %print -depsc  trackingLong_err 
   
   %print -depsc  trackingMidNoise_err 
    
 
   figure(8), 
   %print -depsc  trackingShort_eerel 
   %print -depsc  trackingMid_eerel 
   %print -depsc  trackingLong_eerel 

   %%print -depsc  trackingMidNoise_eerel 

   
   
end
    








