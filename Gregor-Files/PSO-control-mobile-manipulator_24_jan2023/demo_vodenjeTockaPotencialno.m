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


goalPose =[1.0000         0         0    0.4000
         0   -0.7071   -0.7071    2.000
         0    0.7071   -0.7071    1.4000
         0         0         0    1.0000];



q0(8:10)=[.5 .04 pi/2*0]; % popravi bazo da je blizje referenci


classical.dt=0.1;


scenarij=1;  % 1=srednje dalec, 2=blizu, 3=dalec

q0(8:10)=[3.7-.4 1.8+.1  pi/3]; % popravi bazo da je blizje referenci

if scenarij==1
  % goalPose(1:3,4)=[3.3;1.5;1];  % popravi roko da je blizje referenci

   %===== referenca za roko
      w=2*2*pi/3;
    %  a=[0.1065   -1.6405    9.1195  -15.2290]; % y=f(x) polinom 3. reda
    %  t=0:classical.dt:16.5;
      
      

      xar=8;
      yar=8.5; % y=f(x)
      zar=1; % referenca za roko
      
      armRef= [xar;yar;zar];
     
    %========
    goalBase=[8;8.5] 

elseif scenarij==2
    %===== potencialno polje za bazo
    
      xar=4;    
      yar=5; % y=f(x)
      zar=1; % referenca za roko
      
      armRef= [xar;yar;zar];
     
    %========

    %===== potencialno polje za bazo
    goalBase=[xar;yar] 

elseif scenarij==3
    %===== potencialno polje za bazo
    
      xar=1.5;    
      yar=7; % y=f(x)
      zar=1; % referenca za roko
      
      armRef= [xar;yar;zar];
     
    %========

    %===== potencialno polje za bazo
    goalBase=[xar;yar] 
   
    
end

    Skript_planiranjeBaze2; % dolocim potencialno polje za bazo
    %=====




figure(10),set(gca,'view',[-0.6792   90.0000]);  %get(gca,'view')


%flagOmejitve=1;
%classical.optimizerGrega(q0, goalPose,flagOmejitve); %dodal animacijo in par sprememb



 % flagSecundar
 % 0-le primarna, 
 % 1=primarna +sekundarna za v in w , 
 % 12=kot 1 le da še manipulab za sklepe, 
 % 13= dodana se primarna in teciarna naloga (1. naloga = hitrost EE, 2. naloga =hitrost baze,3. naloga je optimizacija manipulabilnosti)

 % flagVLimit=1; =1 ima omejitve hitrosti in pospeska, =0 brez omejitev 
 % flagOnlyPositionEE=0; %=1 z orientacijo EE, =0 brez orientacije EE
 
flagSecundar=1;flagVLimit=1; flagOnlyPositionEE=0; % ok za secundar 1 ali 13 in zamik xar=+0.2do0.6 goalA=[7 10]

classical.optimizerTockaPotencialBase(q0, goalPose,flagSecundar,flagVLimit, CMG,CMGmax,space,resolution,flagOnlyPositionEE,armRef); %dodal animacijo in par sprememb


if 0  % slike za clanek
    
   figure(11),
   %print -depsc  controlPoseShort_potential 
   %print -depsc  controlPoseMid_potential 
   %print -depsc  controlPoseLong_potential 
  
    
   risiRezultate1  % za clanek izrise rezultate
   
   figure(10), 
   set(gca,'view',[-27.1594   29.4140]) 
   %print -depsc  controlPoseShort_side    
   %print -depsc  controlPoseMid_side    
   %print -depsc  controlPoseLong_side    
   
   figure(10), 
   set(gca,'view',[0   90]) 
   %print -depsc  controlPoseShort_top    
   %print -depsc  controlPoseMid_top    
   %print -depsc  controlPoseLong_top    
 
   % fixepsbbox('controlPoseMid_top.eps',10,25)
    fixepsbbox('controlPoseMid_top.eps',10,0)

   
   figure(1), 
   %print -depsc  controlPoseShort_vw 
   %xlim([0 13]),print -depsc  controlPoseMid_vw 
   %xlim([0 22]),print -depsc  controlPoseLong_vw 

   figure(2), 
   %print -depsc  controlPoseShort_jointVel 
   %xlim([0 13]),print -depsc  controlPoseMid_jointVel 
   %xlim([0 22]),print -depsc  controlPoseLong_jointVel 

    figure(5), 
   %print -depsc  controlPoseShort_err 
   %xlim([0 13]),print -depsc  controlPoseMid_err 
   %xlim([0 22]),print -depsc  controlPoseLong_err 
   
   figure(8), 
   %print -depsc  controlPoseShort_eerel 
   %xlim([0 13]),print -depsc  controlPoseMid_eerel  
   %xlim([0 22]),print -depsc  controlPoseLong_eerel  
   
        
end


