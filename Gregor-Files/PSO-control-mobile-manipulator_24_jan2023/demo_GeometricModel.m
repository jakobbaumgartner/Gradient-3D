% simulation = GazeboSimulator('192.168.11.131')
robot = robotPmb2Panda()

%%

joints_val =  [pi/2 pi/3 pi/3 -pi/3 pi/4 pi/2 pi/2]
joints = zeros(1,7) + joints_val .* [0 0 0 0 0 0 0]

% 
% simulation.resetSimulation()
% simulation.jointControl(1,joints(1))
% simulation.jointControl(2,joints(2))
% simulation.jointControl(3,joints(3))
% simulation.jointControl(4,joints(4))
% simulation.jointControl(5,joints(5))
% simulation.jointControl(6,joints(6))
% simulation.jointControl(7,joints(7))
% 
% pause(1)


[Abase, A01, A12, A23, A34, A45, A56, A67, T] = robot.GeometricRobot(joints, [0 0 0])

% simulation.publishRobotPosition()
% A01
% A01g = simulation.getTransformation('panda_link0', 'panda_link1')
% A12
% A12g = simulation.getTransformation('panda_link1', 'panda_link2')
% A23
% A23g = simulation.getTransformation('panda_link2', 'panda_link3')
% A34
% A34g = simulation.getTransformation('panda_link3', 'panda_link4')
% A45
% A45g = simulation.getTransformation('panda_link4', 'panda_link5')
% A56
% A56g = simulation.getTransformation('panda_link5', 'panda_link6')
% A67
% A67g = simulation.getTransformation('panda_link6', 'panda_link7')

% A01err = A01-A01g;
% A01err = sum(A01err(:))
% A12err = A12-A12g;
% A12err = sum(A12err(:))
% A23err = A23-A23g;
% A23err = sum(A23err(:))
% A34err = A34-A34g;
% A34err = sum(A34err(:))
% A45err = A45-A45g;
% A45err = sum(A45err(:))
% A56err = A56-A56g;
% A56err = sum(A56err(:))
% A67err = A67-A67g;
% A67err = sum(A67err(:))

% Aflange = eye(4);
% Aflange(3,4) = 0.107;

Tr = A01 *A12 *A23 *A34 *A45 *A56 * A67
% Tg = A01g*A12g*A23g*A34g*A45g*A56g*A67g*Aflange

% Tr-Tg

showRobot(Abase, A01, A12, A23, A34, A45, A56, A67)



