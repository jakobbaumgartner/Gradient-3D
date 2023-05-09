% demo
close all

robot = robotPmb2Panda()
optimizer = PSO_optimizer()
displayss = displays()
trajec = trajectorys()


% T1 = [-0.8547    0.3096    0.4168    1.6963 ;
%       0.5191    0.5256    0.6740    1.8917 ;
%      -0.0103    0.7924   -0.6099    1.6007];
% 
% T2 = [  0.5262   -0.8504    0.0002  1.4202;
%         0.8504    0.5262    0.0003  1.4452;
%        -0.0004    0.0000    1.0000  0.0978;
%        0 0 0 1];
% 
% T3 = [0.3109   -0.6594    0.6845  1.5327
%    -0.2729   -0.7518   -0.6003  0.5221
%     0.9104   -0.0002   -0.4137  1.5036
%     0 0 0 1];

points = trajec.trajectory1(0);
pointss = points(:,:,5:6);
% T = points(:,:,15)

if 1



[path_parameters, convergence_success, convergence_times, convergence_runs] = optimizer.PSO_optimization_diff_path(pointss);

[baseTrajectory, baseStatus] = robot.baseApproxPositions(pointss);
 
% [param, history_cost, history_distance, tim] = optimizer.PSO_optimization(T, baseTrajectory, []);


%     param = params(I,:)
%     close all
%     displayss.display2Dpoints(baseTrajectory,baseStatus,pointss,path_parameters)
    displayss.display3Dpoints(baseTrajectory,baseStatus,pointss,path_parameters)
    figure()
    plot(path_parameters(:,1:7))
%     display(robot.directKinematics(param)) 



% plot(history_distance)
% figure()
% plot(history_cost)

end

% classical
if 0
gazebo = true;

classical = Classical_optimizer(gazebo)


% robot initial values
q0 = [0 ...
    pi/4 ...
    0 ...
    -pi/3 ...
    0 ...
    1.8675 ...
    0 ...
    0.5 ... 
    0.04 ...
    0];


% classical.simulation.resetSimulation()
classical.optimizer(q0, goalPose)


goalPose =[1.0000         0         0    0.4000
          0   -0.7071   -0.7071    2.000
          0    0.7071   -0.7071    1.4000
          0         0         0    1.0000];


classical.optimizer(q0, goalPose)

end
