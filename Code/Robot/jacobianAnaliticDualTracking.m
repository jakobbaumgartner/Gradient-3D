function [Jdt] = jacobianAnaliticDualTracking(angles)
% The function "jacobianAnaliticDualTracking" calculates the Jacobian matrix of a dual trajectory tracking robotic system 
% 
% Input:
% 
% angles: a column vector with x,y,phi of Platform and joint variables of the Panda Arm 
%
%
% Output:
% 
% Jdt: an 8x9 matrix representing the Jacobian matrix of the dual tracking robotic system,
% where the first six rows correspond to the Jacobian of the end-effector with 
% respect to the joint variables and the last two rows correspond to the Jacobian of 
% the platform with respect to its own velocity commands.


% Call the function "jacobianAnalitic" to obtain the Jacobian matrix "Je" of the end-effector with respect to the joint variables.
[Je] = jacobianAnalitic(angles);

% Define the Jacobian matrix "Jp" for the platform with respect to its own velocity commands.
Jp = [1 0 0 0 0 0 0 0 0 ;
      0 1 0 0 0 0 0 0 0 ];

% Concatenate "Je" and "Jp" along the row axis to form the Jacobian matrix "Jdt".
Jdt = [Je; Jp];

end
