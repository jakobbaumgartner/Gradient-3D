function J = jacobianAnalitic(fi_base, q)


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
    z_base = 0.2976; % 0.83;

    Jbase = zeros(6,2);

%     Jbase (1,:)= [r/2 r/2];


    Jbase (1,:)= [r/2 * cos(fi_base) r/2 * cos(fi_base)];
    Jbase (2,:)= [r/2 * sin(fi_base) r/2 * sin(fi_base)];
    
    Jbase(6,:) = [r/L -r/L];    

    
    Awheels = eye(4);
    

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