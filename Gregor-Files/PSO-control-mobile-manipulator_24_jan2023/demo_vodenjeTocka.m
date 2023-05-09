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


classical.dt=0.01;

flagOmejitve=1;
classical.optimizerGrega(q0, goalPose,flagOmejitve); %dodal animacijo in par sprememb

