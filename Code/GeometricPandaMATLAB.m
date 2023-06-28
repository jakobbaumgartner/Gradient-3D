function [robot_transforms] = GeometricPandaMATLAB(joints, Tbase)


% this is a classic (Spong?) DH convention parameters table, on official Franka
% Emika site there is a different (Craig) convention (that is a pain in the
% arse)
% | a | alpha | d | theta-0 (ignored) |
dhparams = [0 -pi/2 0.333 0 ;
            0 pi/2 0 0 ;
            0.0825 pi/2 0.316 0 ;
            -0.0825 -pi/2 0 0 ;
            0 pi/2 0.384 0 ;
            0.088 pi/2 0 0 ;
            0 0 0.107 0];

% transformations tree
robot = rigidBodyTree;

% joint 1
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;
addBody(robot,body1,'base')

% joint 2
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2,dhparams(2,:),'dh');
body2.Joint = jnt2;
addBody(robot,body2,'body1')

% joint 3
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
setFixedTransform(jnt3,dhparams(3,:),'dh');
body3.Joint = jnt3;
addBody(robot,body3,'body2')

% joint 4
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
setFixedTransform(jnt4,dhparams(4,:),'dh');
body4.Joint = jnt4;
addBody(robot,body4,'body3')

% joint 5
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
setFixedTransform(jnt5,dhparams(5,:),'dh');
body5.Joint = jnt5;
addBody(robot,body5,'body4')

% joint 6
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
setFixedTransform(jnt6,dhparams(6,:),'dh');
body6.Joint = jnt6;
addBody(robot,body6,'body5')

% joint 7
body7 = rigidBody('body7');
jnt7 = rigidBodyJoint('jnt7','revolute');
setFixedTransform(jnt7,dhparams(7,:),'dh');
body7.Joint = jnt7;
addBody(robot,body7,'body6')

% set configuration
config = homeConfiguration(robot);

% set joint positions
for i = 1:1:7
    config(i).JointPosition = joints(i) + dhparams(i,4);
end

% create a list of transforms (and move robot base)
robot_transforms = [];
for i = 1:1:size(dhparams,1)
    robot_transforms(:,:,i) = Tbase*getTransform(robot, config, ['body'+string(i)]);
end


end