function [Je, Jgea, Jgep] = jacobianAnalitic(angles)

% The function "jacobianAnalitic" calculates the Jacobian matrix of a robotic system 
% 
% Input:
% 
% angles: a column vector with x,y,phi of Platform and joint variables of the Panda Arm 
%
%
% Output:
% 
% Je: a 6x9 matrix representing the Jacobian matrix of the end-effector with respect to the joint variables, where N is the number of joints in the robotic system.
% Jgea: a 6x7 matrix representing the Jacobian matrix of the end-effector with respect to the joints (q1 ... q7) velocities changes of the robotic arm.
% Jgep: a 6x2 matrix representing the Jacobian matrix of the end-effector with respect to the linear and angular velocities of the differential drive platform.

% decompose input
q = angles(4:10);
phi = angles(3);

% NOT ACTUALLY SURE IF THIS ONE IS ANALITIC OR GEOMETRIC - CHECK DIFFERENT
% ANGLE ERROR INPUTS


%% Calculate the Panda arm Jacobian


% Calculate trigonometric functions of joint angles
c1=cos(q(1));
s1=sin(q(1));
c2=cos(q(2));
s2=sin(q(2));
c3=cos(q(3));
s3=sin(q(3));
c4=cos(q(4));
s4=sin(q(4));
c5=cos(q(5));
s5=sin(q(5));
c6=cos(q(6));
s6=sin(q(6));
c7=cos(q(7));
s7=sin(q(7));

% Define Panda arm parameters
a3=0.082500;
a4=-0.082500;
a6=0.088000;

d1=0.333000;
d3=0.316000;
d5=0.384000;
d7=0.107000;

% Calculate the position Jacobian of the Panda arm
Jp=zeros(3,7);
Jp(1,1)=d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - a3*c1*s3 - d3*s1*s2 + a6*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - a6*c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - a4*c4*(c1*s3 + c2*c3*s1) - a3*c2*c3*s1 - a4*s1*s2*s4;
Jp(1,2)=d5*(c1*c2*c4 + c1*c3*s2*s4) - d7*(c6*(c1*c2*c4 + c1*c3*s2*s4) - s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5)) + a6*c6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5) + d3*c1*c2 + a6*s6*(c1*c2*c4 + c1*c3*s2*s4) - a3*c1*c3*s2 + a4*c1*c2*s4 - a4*c1*c3*c4*s2;
Jp(1,3)=d7*(s6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3)) + a6*c6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) + d5*s4*(c3*s1 + c1*c2*s3) - a3*c3*s1 - a4*c4*(c3*s1 + c1*c2*s3) - a3*c1*c2*s3 + a6*s4*s6*(c3*s1 + c1*c2*s3);
Jp(1,4)=d5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - d7*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) + a4*s4*(s1*s3 - c1*c2*c3) + a6*s6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + a4*c1*c4*s2 + a6*c5*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2);
Jp(1,5)=a6*c6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + d7*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3));
Jp(1,6)=d7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + a6*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + a6*s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
Jp(1,7)=0;
Jp(2,1)=d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + d3*c1*s2 - a3*s1*s3 + a6*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - a6*c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - a4*c4*(s1*s3 - c1*c2*c3) + a3*c1*c2*c3 + a4*c1*s2*s4;
Jp(2,2)=d5*(c2*c4*s1 + c3*s1*s2*s4) - d7*(c6*(c2*c4*s1 + c3*s1*s2*s4) - s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) + s1*s2*s3*s5)) + a6*c6*(c5*(c2*s1*s4 - c3*c4*s1*s2) + s1*s2*s3*s5) + d3*c2*s1 + a6*s6*(c2*c4*s1 + c3*s1*s2*s4) - a3*c3*s1*s2 + a4*c2*s1*s4 - a4*c3*c4*s1*s2;
Jp(2,3)=a3*c1*c3 - a6*c6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) - d5*s4*(c1*c3 - c2*s1*s3) - d7*(s6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3)) + a4*c4*(c1*c3 - c2*s1*s3) - a3*c2*s1*s3 - a6*s4*s6*(c1*c3 - c2*s1*s3);
Jp(2,4)=d7*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) - d5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - a4*s4*(c1*s3 + c2*c3*s1) - a6*s6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + a4*c4*s1*s2 - a6*c5*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
Jp(2,5)=- a6*c6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - d7*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
Jp(2,6)=- d7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - a6*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - a6*s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
Jp(2,7)=0;
Jp(3,1)=0;
Jp(3,2)=- d5*(c4*s2 - c2*c3*s4) - d3*s2 - d7*(s6*(c5*(s2*s4 + c2*c3*c4) - c2*s3*s5) - c6*(c4*s2 - c2*c3*s4)) - a6*s6*(c4*s2 - c2*c3*s4) - a3*c2*c3 - a4*s2*s4 - a6*c6*(c5*(s2*s4 + c2*c3*c4) - c2*s3*s5) - a4*c2*c3*c4;
Jp(3,3)=d7*(s6*(c3*s2*s5 + c4*c5*s2*s3) + c6*s2*s3*s4) + a3*s2*s3 + a6*c6*(c3*s2*s5 + c4*c5*s2*s3) + a4*c4*s2*s3 - d5*s2*s3*s4 - a6*s2*s3*s4*s6;
Jp(3,4)=d7*(c6*(c2*s4 - c3*c4*s2) + c5*s6*(c2*c4 + c3*s2*s4)) - d5*(c2*s4 - c3*c4*s2) - a6*s6*(c2*s4 - c3*c4*s2) + a4*c2*c4 + a4*c3*s2*s4 + a6*c5*c6*(c2*c4 + c3*s2*s4);
Jp(3,5)=- a6*c6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - d7*s6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3);
Jp(3,6)=d7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + s6*(c2*c4 + c3*s2*s4)) - a6*s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + a6*c6*(c2*c4 + c3*s2*s4);
Jp(3,7)=0;

% Calculate the rotation Jacobian of the Panda arm
Jr=zeros(3,7);
Jr(1,1)=0;
Jr(1,2)=-s1;
Jr(1,3)=c1*s2;
Jr(1,4)=c3*s1 + c1*c2*s3;
Jr(1,5)=s4*(s1*s3 - c1*c2*c3) + c1*c4*s2;
Jr(1,6)=c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4);
Jr(1,7)=- c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
Jr(2,1)=0;
Jr(2,2)=c1;
Jr(2,3)=s1*s2;
Jr(2,4)=c2*s1*s3 - c1*c3;
Jr(2,5)=c4*s1*s2 - s4*(c1*s3 + c2*c3*s1);
Jr(2,6)=s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3);
Jr(2,7)=c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
Jr(3,1)=1;
Jr(3,2)=0;
Jr(3,3)=c2;
Jr(3,4)=-s2*s3;
Jr(3,5)=c2*c4 + c3*s2*s4;
Jr(3,6)=s5*(c2*s4 - c3*c4*s2) - c5*s2*s3;
Jr(3,7)=s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4);

% Combine arm Jacobians
Jae=[Jp;Jr];

% Calculate EE position with respect to Arm Base
Xarm=[...
    d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + d3*c1*s2 - a3*s1*s3 + a6*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - a6*c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - a4*c4*(s1*s3 - c1*c2*c3) + a3*c1*c2*c3 + a4*c1*s2*s4;...
    d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + a3*c1*s3 + d3*s1*s2 - a6*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + a6*c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) + a4*c4*(c1*s3 + c2*c3*s1) + a3*c2*c3*s1 + a4*s1*s2*s4;...
    d1 + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4) + d3*c2 + a6*s6*(c2*c4 + c3*s2*s4) - a3*c3*s2 + a4*c2*s4 + a6*c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - a4*c3*c4*s2;...
  ];

%% Global - Arm Transformation

% Calculate rotation of arm base with respect to global coordinate system
Rga = [cos(phi) -sin(phi) 0 ;
       sin(phi)  cos(phi) 0 ;
       0         0        1 ];

% Expand it to 6x6
Rga66 = [cos(phi) -sin(phi) 0 0 0 0 ;
         sin(phi)  cos(phi) 0 0 0 0 ;
         0         0        1 0 0 0 ;
         0 0 0 cos(phi) -sin(phi) 0 ; 
         0 0 0 sin(phi)  cos(phi) 0 ; 
         0 0 0 0         0        1];

%% Arm + Transformation

% Correct Arm Jacobian to Global coordinate system
Jgea = Rga66 * Jae; 

%% Platfrom Jacobian as it effects EE (differential drive)

% platform - arm base translations
lx = 0;
ly = 0;
lz = 0.823;

Jgep = [cos(phi) -(lx*sin(phi)+ly*cos(phi))-(Xarm(1)*sin(phi)+Xarm(2)*cos(phi)) ; 
        sin(phi) lx*cos(phi)-ly*sin(phi)+Xarm(1)*cos(phi)-Xarm(2)*sin(phi) ; 
        0 0 ; 
        0 0 ;
        0 0 ;
        0 1 ];

%% Combined Jacobian

% Combine platform and Arm Jacobians
Je = [Jgep Jgea];



end