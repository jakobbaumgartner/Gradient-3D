function [T, Abase, A01, A12, A23, A34, A45, A56, A67] = GeometricRobot(robot_angles)


        % calculates direct geometric model of panda arm and tiago base
        % --------------------------------------------------------------
        % returns transformation matrix of EE, based on joint values 
        % and joint transformation matrixes 
    
        % Link DH: https://www.youtube.com/watch?v=rA9tm0gTln8&t=64s

        x = robot_angles(1);
        y = robot_angles(2);
        phi = robot_angles(3);
        fi1 = robot_angles(4);
        fi2 = robot_angles(5);
        fi3 = robot_angles(6);
        fi4 = robot_angles(7);
        fi5 = robot_angles(8);
        fi6 = robot_angles(9);
        fi7 = robot_angles(10);

       
        % base position + rotation

        Abase = [cos(phi), -sin(phi), 0, x ;
                 sin(phi),  cos(phi), 0, y ;
                 0,         0,        1, 0.823 ;
                 0,         0,        0, 1 ];
 
       
        % joint 1
        a1 = 0;
        d1 = 0.333;
        alpha1 = -pi/2;
    
        A01 = getTransformationA(a1, d1, alpha1, fi1);
        
        % joint 2
        a2 = 0;
        d2 = 0;
        alpha2 = pi/2;
    
        A12 = getTransformationA(a2, d2, alpha2, fi2);
        
        % joint 3
        a3 = 0.0825;
        d3 = 0.316;
        alpha3 = pi/2;
    
        A23 = getTransformationA(a3, d3, alpha3, fi3);

        % joint 4
        a4 = -0.0825;
        d4 = 0;
        alpha4 = -pi/2;
    
        A34 = getTransformationA(a4, d4, alpha4, fi4);
    
        % joint 5
        a5 = 0;
        d5 = 0.384;
        alpha5 = pi/2;
    
        A45 = getTransformationA(a5, d5, alpha5, fi5);
    
        % joint 6
        a6 = 0.088;
        d6 = 0;
        alpha6 = pi/2;
    
        A56 = getTransformationA(a6, d6, alpha6, fi6);
    
        % joint 7
        a7 = 0;
        d7 = 0.107;
        alpha7 = 0;
    
        A67 = getTransformationA(a7, d7, alpha7, fi7);

    
        % assemble transformation matrix
    
        T = Abase * A01 * A12 * A23 * A34 * A45 * A56 * A67;
%           T = A01 * A12 * A23 * A34 * A45 * A56 * A67; 

     
    end