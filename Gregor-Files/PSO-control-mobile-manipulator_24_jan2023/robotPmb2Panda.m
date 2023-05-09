classdef robotPmb2Panda

    properties
        
        % prefered horizontal distance EE <-> base
        preferedDistance = 0.5;

        % wheels seperation distance
        L = 0.4044; 
        
    end

    methods

    function [Abase, A01, A12, A23, A34, A45, A56, A67, T] = GeometricRobot_old(obj, qArm, vwBase)


        % calculates direct geometric model of panda arm and tiago base
        % --------------------------------------------------------------
        % returns transformation matrix of EE, based on joint values 
        % and joint transformation matrixes 
    
        % Link DH: https://www.youtube.com/watch?v=rA9tm0gTln8&t=64s


        fi1 = qArm(1);
        fi2 = qArm(2);
        fi3 = qArm(3);
        fi4 = qArm(4);
        fi5 = qArm(5);
        fi6 = qArm(6);
        fi7 = qArm(7);
        x = vwBase(1);
        y = vwBase(2);
        phi = vwBase(3);
    
       
        % base position + rotation

        Abase = [cos(phi), -sin(phi), 0, x ;
                 sin(phi),  cos(phi), 0, y ;
                 0,         0,        1, 0.833 ;
                 0,         0,        0, 1 ];

%         Abase = [1, 0, 0, x ;
%                  0, 1, 0, y ;
%                  0, 0, 1, 0.833 ;
%                  0, 0, 0, 1 ];
        
       
        % joint 1
        a1 = 0;
        d1 = 0.333;
        alpha1 = -pi/2;
    
        A01 = obj.getTransformationA(a1, d1, alpha1, fi1);
        
        % joint 2
        a2 = 0;
        d2 = 0;
        alpha2 = pi/2;
    
        A12 = obj.getTransformationA(a2, d2, alpha2, fi2);
        
        % joint 3
        a3 = 0;
        d3 = 0.316;
        alpha3 = pi/2;
    
        A23 = obj.getTransformationA(a3, d3, alpha3, fi3);
    
        % joint 4
        a4 = 0.0825;
        d4 = 0;
        alpha4 = -pi/2;
        fi40 = 0; %pi;
    
        A34 = obj.getTransformationA(a4, d4, alpha4, fi40 + fi4);
    
        % joint 5
        a5 = -0.0825;
        d5 = 0.384;
        alpha5 = pi/2;
        fi50 = 0; %pi;
    
        A45 = obj.getTransformationA(a5, d5, alpha5, fi50 + fi5);
    
        % joint 6
        a6 = 0.0880;
        d6 = 0;
        alpha6 = pi/2;
        fi60 = 0; %pi;
    
        A56 = obj.getTransformationA(a6, d6, alpha6, fi60 + fi6);
    
        % joint 7
        a7 = 0;
        d7 = 0.1070;
        alpha7 = 0;
    
        A67 = obj.getTransformationA(a7, d7, alpha7, fi7);
    
    
        % assemble transformation matrix
    
        T = Abase * A01 * A12 * A23 * A34 * A45 * A56 * A67;
     
    end

    function [Abase, A01, A12, A23, A34, A45, A56, A67, T] = GeometricRobot(obj, qArm, qBase)


        % calculates direct geometric model of panda arm and tiago base
        % --------------------------------------------------------------
        % returns transformation matrix of EE, based on joint values 
        % and joint transformation matrixes 
    
        % Link DH: https://www.youtube.com/watch?v=rA9tm0gTln8&t=64s


        fi1 = qArm(1);
        fi2 = qArm(2);
        fi3 = qArm(3);
        fi4 = qArm(4);
        fi5 = qArm(5);
        fi6 = qArm(6);
        fi7 = qArm(7);
        x = qBase(1);
        y = qBase(2);
        phi = qBase(3);
    
       
        % base position + rotation

        Abase = [cos(phi), -sin(phi), 0, x ;
                 sin(phi),  cos(phi), 0, y ;
                 0,         0,        1, 0.833 ;
                 0,         0,        0, 1 ];
 
       
        % joint 1
        a1 = 0;
        d1 = 0.333;
        alpha1 = -pi/2;
    
        A01 = obj.getTransformationA(a1, d1, alpha1, fi1);
        
        % joint 2
        a2 = 0;
        d2 = 0;
        alpha2 = pi/2;
    
        A12 = obj.getTransformationA(a2, d2, alpha2, fi2);
        
        % joint 3
        a3 = 0.0825;
        d3 = 0.316;
        alpha3 = pi/2;
    
        A23 = obj.getTransformationA(a3, d3, alpha3, fi3);

        % joint 4
        a4 = -0.0825;
        d4 = 0;
        alpha4 = -pi/2;
    
        A34 = obj.getTransformationA(a4, d4, alpha4, fi4);
    
        % joint 5
        a5 = 0;
        d5 = 0.384;
        alpha5 = pi/2;
    
        A45 = obj.getTransformationA(a5, d5, alpha5, fi5);
    
        % joint 6
        a6 = 0.088;
        d6 = 0;
        alpha6 = pi/2;
    
        A56 = obj.getTransformationA(a6, d6, alpha6, fi6);
    
        % joint 7
        a7 = 0;
        d7 = 0.107;
        alpha7 = 0;
    
        A67 = obj.getTransformationA(a7, d7, alpha7, fi7);

    
        % assemble transformation matrix
    
        T = Abase * A01 * A12 * A23 * A34 * A45 * A56 * A67;
%           T = A01 * A12 * A23 * A34 * A45 * A56 * A67; 

     
    end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function [ A01, A12, A23, A34, A45, A56, A67, T] = GeometricArm(obj, qArm)


        % calculates direct geometric model of panda arm and tiago base
        % --------------------------------------------------------------
        % returns transformation matrix of EE, based on joint values 
        % and joint transformation matrixes 
    
        % Link DH: https://www.youtube.com/watch?v=rA9tm0gTln8&t=64s


        fi1 = qArm(1);
        fi2 = qArm(2);
        fi3 = qArm(3);
        fi4 = qArm(4);
        fi5 = qArm(5);
        fi6 = qArm(6);
        fi7 = qArm(7);
    
       
        % base position + rotation

%         Abase = [cos(phi), -sin(phi), 0, x ;
%                  sin(phi),  cos(phi), 0, y ;
%                  0,         0,        1, 0.833 ;
%                  0,         0,        0, 1 ];

%         Abase = [1, 0, 0, x ;
%                  0, 1, 0, y ;
%                  0, 0, 1, 0.833 ;
%                  0, 0, 0, 1 ];
        
       
        % joint 1
        a1 = 0;
        d1 = 0.333;
        alpha1 = -pi/2;
    
        A01 = obj.getTransformationA(a1, d1, alpha1, fi1);
        
        % joint 2
        a2 = 0;
        d2 = 0;
        alpha2 = pi/2;
    
        A12 = obj.getTransformationA(a2, d2, alpha2, fi2);
        
        % joint 3
        a3 = 0;
        d3 = 0.316;
        alpha3 = pi/2;
    
        A23 = obj.getTransformationA(a3, d3, alpha3, fi3);
    
        % joint 4
        a4 = 0.0825;
        d4 = 0;
        alpha4 = -pi/2;
        fi40 = 0; %pi;
    
        A34 = obj.getTransformationA(a4, d4, alpha4, fi40 + fi4);
    
        % joint 5
        a5 = -0.0825;
        d5 = 0.384;
        alpha5 = pi/2;
        fi50 = 0; %pi;
    
        A45 = obj.getTransformationA(a5, d5, alpha5, fi50 + fi5);
    
        % joint 6
        a6 = 0.0880;
        d6 = 0;
        alpha6 = pi/2;
        fi60 = 0; %pi;
    
        A56 = obj.getTransformationA(a6, d6, alpha6, fi60 + fi6);
    
        % joint 7
        a7 = 0;
        d7 = 0.1070;
        alpha7 = 0;
    
        A67 = obj.getTransformationA(a7, d7, alpha7, fi7);
    
    
        % assemble transformation matrix
    
        T =  A01 * A12 * A23 * A34 * A45 * A56 * A67;
     
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function J = v2_jacobi_panda_pmb2_wheels (obj, q, fi_base)

        % calculates direct geometric model of panda arm with added base, base
        % considered as rotational + linear joints
        % --------------------------------------------------------------
        % returns jacobian matrix 
    
        
        % link: https://www.rosroboticslearning.com/jacobian
    
        fi1 = q(1);
        fi2 = q(2);
        fi3 = q(3);
        fi4 = q(4);
        fi5 = q(5);
        fi6 = q(6);
        fi7 = q(7);
            
        % baza wheels
    
        r = 0.0985; % wheel radius
        L = 0.4044; % wheel seperation
        z_base = 0.83; % 0.2976;
    
        Jbase = zeros(6,2); % kinematika za bazo dXe=Jbase*[vDesni; vLevi]
    
        Jbase (1,:)= [r/2 * cos(fi_base) r/2 * cos(fi_base)]; % vx
        Jbase (2,:)= [r/2 * sin(fi_base) r/2 * sin(fi_base)]; % vy
        
        Jbase(6,:) = [r/L -r/L];   %wz 
    
        
        Awheels = eye(4);   % ??
        
%          Awheels = [cos(fi_base), -sin(fi_base), 0,  qb(1)*1 ;
%                     sin(fi_base),  cos(fi_base), 0,  qb(2)*1 ;
%                     0,         0,                1,  z_base*1 ;
%                     0,         0,                0,  1 ];
%          Awheels = [cos(fi_base), -sin(fi_base), 0,  0 ;
%                     sin(fi_base),  cos(fi_base), 0,  0 ;
%                     0,         0,                1,  z_base*1 ;
%                     0,         0,                0,  1 ];
       
        
        
    
        % joint 1
        a1 = 0;
        d1 = 0.333;
        alpha1 = -pi/2;
    
        A01 = obj.getTransformationA(a1, d1, alpha1, fi1);
        
        % joint 2
        a2 = 0;
        d2 = 0;
        alpha2 = pi/2;
    
        A12 = obj.getTransformationA(a2, d2, alpha2, fi2);
        
        % joint 3
        a3 = 0.0825;
        d3 = 0.316;
        alpha3 = pi/2;
    
        A23 = obj.getTransformationA(a3, d3, alpha3, fi3);
    
        % joint 4
        a4 = -0.0825;
        d4 = 0; 
        alpha4 = -pi/2;
    
        A34 = obj.getTransformationA(a4, d4, alpha4, fi4);
    
        % joint 5
        a5 = 0;
        d5 = 0.384;
        alpha5 = pi/2;
    
        A45 = obj.getTransformationA(a5, d5, alpha5, fi5);
    
        % joint 6
        a6 = 0.0880;
        d6 = 0;
        alpha6 = pi/2;
    
        A56 = obj.getTransformationA(a6, d6, alpha6, fi6);
    
        % joint 7
        a7 = 0;
        d7 = 0.107;
        alpha7 = 0;
    
        A67 = obj.getTransformationA(a7, d7, alpha7, fi7);
    
    
    
        % transform matrix
        T7 = A01 * A12 * A23 * A34 * A45 * A56 * A67;
        
        p7 = T7(1:3,4);
    
        z0 = [0 0 1]';
    
    
        % initialize Jacobi matrix
        J = zeros(6,9);
    
        % first column & second - wheels
    
        p = [0 0 0]';
    
        z = z0;
    
        p_panda = p7;
        p_panda(3) = p_panda(3) + z_base;
    
        jp = Jbase(1:3,:) + Awheels(1:3,1:3)*v2m(p_panda)'*Jbase(4:6,:);
        jo = Jbase(4:6,:);
    
        J(1:3,1:2) = jp;
        J(4:6,1:2) = jo;
    
        
        % third column
    
        z =Awheels(1:3,1:3)*z0;
    
        T = Awheels;
    
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,3) = jp;
        J(4:6,3) = jo;
    
    
        % forth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*z0;
    
        T = Awheels*A01;
    
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,4) = jp;
        J(4:6,4) = jo;
    
    
        % fifth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*z0;
    
        T = Awheels*A01*A12;
    
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,5) = jp;
        J(4:6,5) = jo;
    
        
        % sixth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*z0;
    
        T = Awheels*A01*A12*A23;
    
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,6) = jp;
        J(4:6,6) = jo;
    
    
        % seventh column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*z0;
    
        T = Awheels*A01*A12*A23*A34;
    
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,7) = jp;
        J(4:6,7) = jo;
    
    
        % eighth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*A45(1:3,1:3)*z0;
    
        T = Awheels*A01*A12*A23*A34*A45;
    
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,8) = jp;
        J(4:6,8) = jo;
    
    
        % nineth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*A45(1:3,1:3)*A56(1:3,1:3)*z0;
    
        T = Awheels*A01*A12*A23*A34*A45*A56;
    
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,9) = jp;
        J(4:6,9) = jo; 
    
 
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [A] = getTransformationA(obj,a,d,alpha,fi)

        % getTransformationA - get one step of direct kinematic model
        % --------------------------------------------------------------
        % creates transformation between two Coordinate systems, 
        % based on DH parameters
        
        A = [ cos(fi) -sin(fi)*cos(alpha) sin(fi)*sin(alpha) a*cos(fi) ;
              sin(fi) cos(fi)*cos(alpha) -cos(fi)*sin(alpha) a*sin(fi) ;
              0       sin(alpha)          cos(alpha)         d ;
              0       0                   0                  1];
        
                    
    end


    function [basePositions, calcStatus] = baseApproxPositions(obj, eeTrajectory)

            % calculates approx base position, behind EE goal
            % input: EE Tmatrix
            % output: basePosition = [x,y,fi], status = 1/0  (position
            % calculated or not (not possible from given matrix))

            % ----------------------------------------------------------

            basePositions = [];

            for i = 1:1:size(eeTrajectory,3)
                
                Tgoal = eeTrajectory(:,:,i);

                basePosition = [];
    
                % baseApproxPosition calculate approx position of base
                
                positionEE = Tgoal(1:3,4);
    
                zx = Tgoal(1,3);
                zy = Tgoal(2,3);
                zz = Tgoal(3,3);
    
                % if EE orientation is mostly vertical, return err
                status = 0 + (abs(zz) < 0.9) * 1;
    
                % if EE orientation is horitonzal, calculate base position
                if status
                 
                    fi = atan2(zy, zx);
    
                    xB = positionEE(1) - obj.preferedDistance * cos(fi);
                    yB = positionEE(2) - obj.preferedDistance * sin(fi);
    
                    basePosition = [xB yB fi];

                    calcStatus(i) = 1;
                    basePositions(i,:) = basePosition;
    
                else
    
                    display('Approx base position not calculated. Input error.')

                    calcStatus(i) = 0;
    
                end
            end

    end

    function vw_vel = convertWheelsToVW(obj, wheelsVelocity)

        % CONVERT WHEELS WL AND WR TO V, W
        %
        % ----------------------------------
        %
        % INPUT: 
        % 
        % wheelsVelocity : [ wL, wR ]
        %
        % OUTPUT: 
        %
        % vw_vel: [ v, w ]
        %
        % ----------------------------------

        % convert wheels speeds to ang + trans
        
        vw_vel = [(wheelsVelocity(1) + wheelsVelocity(2)) / 2 ;
                 -(wheelsVelocity(2) - wheelsVelocity(1)) / obj.L]';


    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function J = v2_jacobi_panda_pmb2_vw (obj, qa, qb)

        % modifikacije GK: 27.11.2022  ?? 
        %
        % calculates direct geometric model of panda arm with added base, base
        % considered as rotational + linear joints
        % vhoda za bazo sta v in w
        % --------------------------------------------------------------
        % returns jacobian matrix 
    
        
        % link: https://www.rosroboticslearning.com/jacobian
    
        fi1 = qa(1);
        fi2 = qa(2);
        fi3 = qa(3);
        fi4 = qa(4);
        fi5 = qa(5);
        fi6 = qa(6);
        fi7 = qa(7);
        
        fi_base=qb(3);
        % baza wheels
    
        r = 0.0985; % wheel radius
        L = 0.4044; % wheel seperation
        z_base = 0.833; % 0.2976;
    
        Jbase = zeros(6,2); % kinematika za bazo dXe=Jbase*[v; w]
    
        Jbase (1,:)= [cos(fi_base) 0]; % vx
        Jbase (2,:)= [sin(fi_base) 0]; % vy
        Jbase(6,:) = [0 1];   %wz 
    
        
        Awheels = eye(4);   % ??
%         Awheels = [cos(fi_base), -sin(fi_base), 0,  qb(1)*1 ;
%                    sin(fi_base),  cos(fi_base), 0,  qb(2)*1 ;
%                    0,         0,                1,  z_base*1 ;
%                    0,         0,                0,  1 ];
        
    
        % joint 1
        a1 = 0;
        d1 = 0.333;
        alpha1 = -pi/2;
        A01 = obj.getTransformationA(a1, d1, alpha1, fi1);
        
        % joint 2
        a2 = 0;
        d2 = 0;
        alpha2 = pi/2;
        A12 = obj.getTransformationA(a2, d2, alpha2, fi2);
        
        % joint 3
        a3 = 0.0825;
        d3 = 0.316;
        alpha3 = pi/2;
        A23 = obj.getTransformationA(a3, d3, alpha3, fi3);
    
        % joint 4
        a4 = -0.0825;
        d4 = 0;
        alpha4 = -pi/2;
        fi40 = 0; %pi;
        A34 = obj.getTransformationA(a4, d4, alpha4, fi40 + fi4);
    
        % joint 5
        a5 = 0;
        d5 = 0.384;
        alpha5 = pi/2;
        fi50 = 0; %pi;
        A45 = obj.getTransformationA(a5, d5, alpha5, fi50 + fi5);
    
        % joint 6
        a6 = 0.0880;
        d6 = 0;
        alpha6 = pi/2;
        fi60 = 0; %pi;
        A56 = obj.getTransformationA(a6, d6, alpha6, fi60 + fi6);
    
        % joint 7
        a7 = 0;
        d7 = 0.1070;
        alpha7 = 0;
        A67 = obj.getTransformationA(a7, d7, alpha7, fi7);
    
    
    
        % transform matrix 
        T7 = A01 * A12 * A23 * A34 * A45 * A56 * A67;
        p7 = T7(1:3,4); % pozicija EE relativno na bazo
    
        z0 = [0 0 1]'; % ??
    
    
        % initialize Jacobi matrix
        J = zeros(6,9);
    
        % first column & second - base (inputs v in w)
        p = [0 0 0]';
        z = z0;
    
        p_panda = p7;
        p_panda(3) = p_panda(3) + z_base; % z-koordinata EE z upostevanjem baze
    
        jp = Jbase(1:3,:) + Awheels(1:3,1:3)*v2m(p_panda)'*Jbase(4:6,:);
        jo = Jbase(4:6,:);
    
        J(1:3,1:2) = jp;
        J(4:6,1:2) = jo;
    
        
        % third column
        z =Awheels(1:3,1:3)*z0;
        T = Awheels;
        p = T(1:3,4);
    
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,3) = jp;
        J(4:6,3) = jo;
    
    
        % forth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*z0;
        T = Awheels*A01;
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,4) = jp;
        J(4:6,4) = jo;
    
    
        % fifth column
    
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*z0;
        T = Awheels*A01*A12;
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,5) = jp;
        J(4:6,5) = jo;
    
        
        % sixth column
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*z0;
        T = Awheels*A01*A12*A23;
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,6) = jp;
        J(4:6,6) = jo;
    
    
        % seventh column
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*z0;
        T = Awheels*A01*A12*A23*A34;
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,7) = jp;
        J(4:6,7) = jo;
    
    
        % eighth column
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*A45(1:3,1:3)*z0;
        T = Awheels*A01*A12*A23*A34*A45;
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,8) = jp;
        J(4:6,8) = jo;
    
    
        % nineth column
        z =Awheels(1:3,1:3)*A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*A45(1:3,1:3)*A56(1:3,1:3)*z0;
        T = Awheels*A01*A12*A23*A34*A45*A56;
        p = T(1:3,4);
        jp = cross(z,(p7-p));
        jo = z;
    
        J(1:3,9) = jp;
        J(4:6,9) = jo; 
    
 
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    end

end
