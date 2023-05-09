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

 % flagSecundar:
 % 0-le primarna, 
 % 1=primarna +sekundarna za v in w , 
 % 12=kot 1 le da še manipulab za sklepe, 
 % 13= dodana se primarna in teciarna naloga (1. naloga = hitrost EE, 2. naloga =hitrost baze,3. naloga je optimizacija manipulabilnosti)
flagSecundar=13; % ta ima se drugo nalogo za bazo ampak je premalo redundance in ne uspe bolje kot opcija 1
flagVLimit=0; 

classical.dt=0.1;

classical.optimizerTrajectory(q0, goalPose,flagSecundar,flagVLimit); %dodal animacijo in par sprememb

% % se opcija brez zahtev za orientacijo EE
% classical.optimizerTrajectoryOnlyPosition(q0, goalPose,flagSecundar,flagVLimit); %dodal animacijo in par sprememb
% 





