classdef Classical_optimizer

    properties


        % ros-ip
        rosip = '192.168.11.130';

        % imported classes
        robot = {};

        % gazebo
        simulation = [];
        useGazebo = 0;

        % time-step
        dt = 0.1; 

        % za animacijo
        mainFig=10;
        hMainFig=[];
        hRob=[];hArm=[];hEnd=[]

    end

    methods


        function obj = Classical_optimizer(obj)

            obj.robot = robotPmb2Panda();
            
            % if using Gazebo simulator
            if obj.useGazebo
                obj.simulation = GazeboSimulator(obj.rosip)
            end

            % za animacijo
             obj.hMainFig=figure(obj.mainFig); clf; 
             hold on;axis equal ,grid
           %  axis([-1  1 -1 1 0 1]);
             xlabel('x (m)','FontSize', 8);ylabel('y (m)','FontSize', 8);
            %get(gca,'view')
             set(gca,'view',[-41.4380   34.7032]);  
             %get(gcf,'Position')
             set(gcf,'Position',[48.2000  316.2000  560.0000  420.0000]);
             obj.hArm=plot3(nan,nan,nan,'b','LineWidth',2) ;     % handle na roko 
             obj.hRob=plot3(nan,nan,nan,'r','LineWidth',2) ;     % handle na platformo 
             obj.hEnd=plot3(nan,nan,nan,'g','LineWidth',2) ;     % handle na prijemalo 

            
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function optimizer(obj, qBegin, poseGoal)

        % ----------------------------------------------------
        %
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % ----------------------------------------------------
            
            
        % logs
        history_qBase = [];
        history_posEE = [];
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % ----------------------------------------------------
            

            RecBase = [];
            RecEnd = [];

        
            % logs
            history_qBase = [];

            % decompose goal matrix
            gR = poseGoal(1:3,1:3);
            gP = poseGoal(1:3,4);
            
            % loop run / stop constant
            runLoop = 1;
            figure(obj.mainFig); 
            plot3(gP(1),gP(2),gP(3),'mx') % narise referencno tocko

            
            % loop run / stop constant
            runLoop = 1;
            stevc=1;
            
            qArm = qBegin(1:7);
            qBase = qBegin(8:10); % [x y phi]

            % optimization loop
            while runLoop               

                stevc=stevc+1;
                % calculate distances from goal Pose
                % ----------------------------------

                % get transformation matrix

                if obj.useGazebo

                    % read states from gazebo
                    % ----------------------------------
    
                    % publish robot locations from gazebo to tf 
                    obj.simulation.publishRobotPosition();
    
                    % read arm joints
                    qArm = obj.simulation.jointsAllRead();
    
                    % read base orientation
                    angleBase = obj.simulation.getBaseAngle();  

                    % read EE position
                    rT = obj.simulation.getPositionEE();

                else
                    % calculate
                    [~, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);
                    
                    % get base angle
                    angleBase = qBase(3);
                end
                
                % decompose current transformation matrix
                cR = rT(1:3,1:3);
                cP = rT(1:3,4);

                % rotation 

                dR = gR*cR'; 
                dR = dR / norm(dR);
                eq = 2*log(quaternion(rotm2quat(dR)));
                [~, qB, qC, qD] = eq.parts;                
                eR = [qB, qC, qD];

                % translation
                eP = gP-cP;

                % compose velocity vector

                dist = [2*eP ; 4 * eR'];


                % run inverse kinematics
                % ----------------------------------
                

                J = obj.robot.v2_jacobi_panda_pmb2_wheels(qArm, angleBase);
                % J(:,1:2) = zeros(6,2);

                pinv_J = pinv(J);
    
                q_vel = pinv_J * dist;

                if obj.useGazebo

                    % control Gazebo simulation
                    % ----------------------------------

                    % numerically "integrate" arm angles
                    qNew = qArm + 0.2 * q_vel(3:9); % !!! add integ

                    % convert [wL wR] to [v w]
                    vw_vel = obj.robot.convertWheelsToVW(q_vel(1:2));

                    % send commands to Gazebo
                    obj.simulation.driveControl(vw_vel, 0);
                    obj.simulation.jointsAllControl(qNew);
        
                    else
    
                    % calculate new pose (numerical integration)
                    % ----------------------------------
    
                    % wheels velocity to translat and angular velocity 
                    base_wv = [ 0.5 * (q_vel(1) + q_vel(2)) / 2 ;
                                 2 * (q_vel(1) - q_vel(2)) / obj.robot.L]; % !!! is this efficient or am I copying the whole robot object every loop ? 
                    
                    % arm joints num integration
                    qArm = qArm + q_vel(3:9)' * obj.dt;
    
                    % base num integration
                    qBase = qBase + [base_wv(1) * cos(qBase(3));
                                     base_wv(1) * sin(qBase(3));
                                     base_wv(2) ]' * obj.dt;
    
                    history_qBase = [history_qBase qBase'];
                    RecBase = [RecBase; qBase];
                    RecEnd  = [RecEnd; cP'];
    
                    % break condition
                       % !!!
    
                  %  if(mod(stevc,5)==1)
                        obj.animateRobot([qArm,qBase]') ; 
                        pause(obj.dt);   
                        drawnow();  
                  %  end
                   
                   if(stevc>400)
                       runLoop=0;
                   end

                end

              fs=14;     
              plot(RecBase(:,1),RecBase(:,2),'r:')
              plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'g:')
              xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
              ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
              zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)
                 
   
            end
          
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function optimizerGrega(obj, qBegin, poseGoal,flagOmejitve)

        % ----------------------------------------------------
        %
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % ----------------------------------------------------
            % logs
            RecBase = [];
            RecEnd = [];
            RecT=[];
            RecU=[];
            
            % decompose goal matrix
            gR = poseGoal(1:3,1:3);
            gP = poseGoal(1:3,4);
            
            figure(obj.mainFig); 
            plot3(gP(1),gP(2),gP(3),'mx') % narise referencno tocko

            
            % loop run / stop constant
            runLoop = 1;
            
            qArm = qBegin(1:7);
            qBase = qBegin(8:10); % [x y phi]
            t=0;
            stevc=0;
            % optimization loop
            while runLoop               
                t=t+obj.dt; stevc=stevc+1;
                % calculate distances from goal Pose
                % ----------------------------------

                % get transformation matrix
                [~, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);
                
                % decompose current transformation matrix
                cR = rT(1:3,1:3);
                cP = rT(1:3,4);

                % rotation error
                dR = gR*cR';
                
                dR = dR / norm(dR);
                eq = 2*log(quaternion(rotm2quat(dR)));
                [~, qB, qC, qD] = eq.parts;                
                eR = [qB, qC, qD];

                % translation
                eP = gP-cP;

                % controller : compose velocity vector
                velocityEE = 1*[eP*2 ; eR'*4];  % p-regulator na poziciji EE-ja
             %   velocityEE = 0.7*[eP*1 ; eR'*2];  % p-regulator na poziciji EE-ja


                % run inverse kinematics
                % ----------------------------------
              if 0   % original le predelana na vhoda v in w     
                J = obj.robot.v2_jacobi_panda_pmb2_vw (qArm, qBase);
              else   % po clanku in J ocenim numericno  
                opcijaJ=2;  % opcije izracuna 1=original 2=preko R matrike AMS Knjiga, 3= preko kvaterninona AMS Knjiga, 4=preko kvatenoponov ?? ne dela za orientacijo           
                J_=obj.jacobianNumeric([qBase,qArm], opcijaJ);
                Sb=[cos(qBase(3)) 0;
                    sin(qBase(3)) 0;
                    0             1];
                S=[Sb         zeros(3,7);
                   zeros(7,2) eye(7)];
                J=J_*S;
              end
              
                pinv_J = pinv(J);
               % pinv_J = J'*(J*J' + 0.01*eye(6) )^(-1);
                
                q_vel = pinv_J * velocityEE;


                % calculate new pose (numerical integration)
                % ----------------------------------

                % translation and angular velocity 
                
              if flagOmejitve  % omejitve hitrosti
                vMax=[0.5, 2, 2,2,2,2,2,2,2];  
                for i=1:9
                    if(abs(q_vel(i))>vMax(i))
                        q_vel(i)=vMax(i)*sign(q_vel(i));
                    end
                end
              end          
           
                base_wv = [ q_vel(1) ;
                            q_vel(2) ]; 

                % arm joints num integration
                qArm = qArm + q_vel(3:9)' * obj.dt;

                % base num integration
                qBase = qBase + [base_wv(1) * cos(qBase(3));
                                 base_wv(1) * sin(qBase(3));
                                 base_wv(2) ]' * obj.dt;

                RecBase = [RecBase; qBase];
                RecEnd  = [RecEnd; cP'];
                RecT  = [RecT; t];
                RecU  = [RecU; q_vel'];

             %if(stevc>10)
                obj.animateRobot([qArm,qBase]') ; 
                pause(obj.dt);   
                drawnow();
                stevc=0;
           %  end
               %  rT
               

               if(t>6), runLoop=0; end
            end
            
      fs=14;     
      plot(RecBase(:,1),RecBase(:,2),'r:')
      plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'g:')
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      figure(1),plot(RecT,RecU(:,1),RecT,RecU(:,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[rad/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
      figure(2),plot(RecT,RecU(:,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)
      
   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
function animateRobot(obj, qBegin)
% ----------------------------------------------------
% animation
% qBegin - [q1 ... q7 x y phi]
% ----------------------------------------------------
    
    qa = qBegin(1:7);  
    qb = qBegin(8:10); % [x y phi]
    
    [Abase, A01, A12, A23, A34, A45, A56, A67, rT] = obj.robot.GeometricRobot(qa, qb);

    p=Abase(1:3,4); % baza roke (le dvignjena nad robota)
    
    f=pi*2/3;s=0.25; %dolzina puscice za smer platforme
    R=[p'
       qb(1),qb(2), 0; 
       qb(1)+s*cos(qb(3)),qb(2)+s*sin(qb(3)),0;
       qb(1)+s/2*cos(qb(3)+f),qb(2)+s/2*sin(qb(3)+f),0;
       qb(1)+s/2*cos(qb(3)-f),qb(2)+s/2*sin(qb(3)-f),0;
       qb(1)+s*cos(qb(3)),qb(2)+s*sin(qb(3)),0];

    P=[p'];
    
    A=Abase*A01;  % 1. sklep roke
    p=A(1:3,4); P=[P;p'];

    A=A*A12;  % 2. sklep roke
    p=A(1:3,4); P=[P;p'];
    
    A=A*A23;  % 3. sklep roke
    p=A(1:3,4); P=[P;p'];

    A=A*A34;  % 4. sklep roke
    p=A(1:3,4); P=[P;p'];

    A=A*A45;  % 5. sklep roke
    p=A(1:3,4); P=[P;p'];

    A=A*A56;  % 6. sklep roke
    p=A(1:3,4); P=[P;p'];
    
    A=A*A67;  % 7. sklep roke
    p=A(1:3,4); P=[P;p'];
    
    % dodam eno demo EE prijamalo
    eg=[0 0 0  ; % tocke za demo prijemalo v yz ravnini tocke 
        0 0 2  ;  
       0 -1 2  ;
       0 -1 4  ;
       0 -1 2  ;
       0 0 2   ;
       0 1 2 ;
       0 1 4 ;]'*0.02;  
   
   % eg=[0 0 0]'  ; % tocke za demo prijemalo v yz ravnini tocke 
 
   
   
    egh=[eg; ones(1,size(eg,2))]; % homogene koordinate, dodam enice v zadnji stolpec
    E=(A*egh)'; %tocke prijemala za izris
    
    set(obj.hRob,'XData',R(:,1),'YData',R(:,2),'ZData',R(:,3))   % izris robota
    set(obj.hArm,'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3))   % izris roke
    set(obj.hEnd,'XData',E(:,1),'YData',E(:,2),'ZData',E(:,3))   % izris roke
  %  drawnow;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

function controlBase(obj, qBegin)
    % ----------------------------------------------------
    % animation
    % qBegin - [q1 ... q7 x y phi]
    % ----------------------------------------------------
        qa = qBegin(1:7);  
        qb = qBegin(8:10); % [x y phi]
        
        obj.animateRobot(qBegin);

        % referenca za bazo
        Ts=0.01; w=2*pi/10;
        t=0:Ts:10; xr=sin(w*t); yr=cos(w*t);
        dxr=w*cos(w*t); dyr=-w*sin(w*t); 
        ddxr=-w^2*sin(w*t); ddyr=-w^2*cos(w*t); 
        v_ref=sqrt(dxr.^2+dyr.^2); 
        w_ref=(dxr.*ddyr-dyr.*ddxr)./(dxr.^2+dyr.^2);

        Q_ref=[xr' yr' atan2(dyr,dxr)']; % reference trajectory

        figure(obj.mainFig); 
        plot(xr,yr,'m.')
       
        q=qb;Q=[]; U=[];
        
        for i=1:length(t)                
           q_ref=Q_ref(i,:); % current reference pose 
               e=[cos(q(3)) sin(q(3)) 0;...
                 -sin(q(3)) cos(q(3)) 0;... 
                  0            0      1]     * (q_ref-q)';    % error vector
           e(3)=wrapToPi(e(3)); % correct angle

            % control 
            e_x=e(1); e_y=e(2); e_phi=e(3);
            zeta=0.9;
            g=85*.1;

            % current reference inputs  
            vref = v_ref(i);
            wref = w_ref(i);

            Kx=2*zeta*sqrt(wref^2+g*vref^2);
            Kphi=Kx;
            Ky=g;        

            v_ = vref*cos(e(3)) + Kx*e_x;
            w_ = wref + Ky*vref*sinc(e_phi/pi)*e_y+Kphi*e_phi;

            Q=[Q;q]; U=[U;[v_, w_]];  
            
            phi=q(1,3);
            dq=[ v_*cos(phi+w_*Ts/2);     % simulate robot kinematics
                 v_*sin(phi+w_*Ts/2);
                 w_];    
            q=q+Ts*dq';  

            obj.animateRobot( [qBegin(1:7), q] );
            pause(Ts);
        end
        
        plot(Q(:,1),Q(:,2),'r')
        
        figure(1),plot(Q(:,1),Q(:,2),xr,yr,'--'), xlabel('x [m]'),ylabel('y [m]')
        figure(2),plot(t,U(:,1),t,U(:,2),'--'),xlabel('t [s]'),ylabel('v [m/s], \omega [rad/s]'),legend('v','\omega'),% axis([0 Tsampling(end) 0 1])

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

function J=jacobianNumeric(obj,q, opcija)
        % ----------------------------------------------------
        % animation get jacobian numerically for given configuration q
        % q - [ x y phi q1 ... q7]
        % J - jacobian for end effector velocities
        % ----------------------------------------------------
        dd=0.005; % cca 5mm
        df=0.01; % ccca 0.5 stopinje
        delta=[dd dd df df df df df df df df];
        dq_zeros=zeros(1,10);

        J=zeros(6,10);
    
     for i=1:10  
        dq=dq_zeros;
        dq(i)=delta(i);
        qq2=q+dq;
        qq1=q-dq;
        
        [~, ~, ~, ~, ~, ~, ~, ~, T1] = obj.robot.GeometricRobot(qq1(4:10), qq1(1:3));
        [~, ~, ~, ~, ~, ~, ~, ~, T2] = obj.robot.GeometricRobot(qq2(4:10), qq2(1:3));
        
        % parcialni odvodi za pozicije
        J(1:3,i)= ( T2(1:3,4)-T1(1:3,4) )/ (qq2(i)-qq1(i));
   
        % parcialni odvodi za kote
        R1=T1(1:3,1:3);
        R2=T2(1:3,1:3);
        dR = R2*R1';  % R2=dR*R1 , od orientacije 1 proti orientaciji 2 (je kot referenca)

        dR = dR / norm(dR);
   if opcija==1     
        eq = 2*log(quaternion(rotm2quat(dR))); % kvaternion pogreÅ¡ka
        [~, qB, qC, qD] = eq.parts; 
        eR = [qB, qC, qD]';  
   elseif opcija==2      
        % 2. opcija za izracun kotov (AMS knjiga slo, str 165)
        OmegaDT=-(dR-eye(3));
        eR = [OmegaDT(2,3); OmegaDT(3,1); OmegaDT(1,2)];
   elseif opcija==3       
        % 3. opcija za izracun kotov (AMS knjiga slo, str 168)
        eq =rotm2quat(dR); % kvaternion pogreÅ¡ka 
        phi=2*acos(eq(1));
        if abs(phi>0)
            eR =phi/sin(phi/2)*[eq(2:4)]';
        else
            eR = [0;0;0];
        end
 elseif opcija==4  % ?? ni ok?
        % 4. opcija za izracun kotov (AMS knjiga slo, str 168)
        quat1Kon=quaternion(rotm2quat(R1).*[1 -1 -1 -1]);
        quat2=quaternion(rotm2quat(R2)); % konjugiran
        eqq= quat2*quat1Kon; 

        [qA, qB, qC, qD] = eqq.parts; 
        eq = [qA,qB, qC, qD];  
        phi=2*acos(eq(1));
        if abs(phi>0)
            eR = -phi/sin(phi/2)*[eq(2:4)]';
        else
            eR = [0;0;0];
        end
   end     
        
        J(4:6,i)= eR / (qq2(i)-qq1(i));
     end      
         
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
 function J=jacobianNumericOnlyPosition(obj,q, opcija)
        % ----------------------------------------------------
        % animation get jacobian numerically for given configuration q
        % q - [ x y phi q1 ... q7]
        % J - jacobian for end effector velocities
        % ----------------------------------------------------
        dd=0.005; % cca 5mm
        df=0.01; % ccca 0.5 stopinje
        delta=[dd dd df df df df df df df df];
        dq_zeros=zeros(1,10);

        J=zeros(3,10);
    
     for i=1:10  
        dq=dq_zeros;
        dq(i)=delta(i);
        qq2=q+dq;
        qq1=q-dq;
        
        [~, ~, ~, ~, ~, ~, ~, ~, T1] = obj.robot.GeometricRobot(qq1(4:10), qq1(1:3));
        [~, ~, ~, ~, ~, ~, ~, ~, T2] = obj.robot.GeometricRobot(qq2(4:10), qq2(1:3));
        
        % parcialni odvodi za pozicije
        J(1:3,i)= ( T2(1:3,4)-T1(1:3,4) )/ (qq2(i)-qq1(i));
     end      
         
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function J=jacobianNumericOnlyArm(obj,q, opcija)
        % ----------------------------------------------------
        % animation get jacobian numerically for given configuration q
        % q - [q1 ... q7]
        % J - jacobian for end effector velocities
        % ----------------------------------------------------
        dd=0.005; % cca 5mm
        df=0.01; % ccca 0.5 stopinje
        delta=[df df df df df df df]; % ne delam perturbacij za bazo
        dq_zeros=zeros(1,7);

        J=zeros(6,7);
    
     for i=1:7  
        dq=dq_zeros;
        dq(i)=delta(i);
        qq2=q+dq;
        qq1=q-dq;
        
        [ ~, ~, ~, ~, ~, ~, ~, T1] = obj.robot.GeometricArm(qq1);
        [ ~, ~, ~, ~, ~, ~, ~, T2] = obj.robot.GeometricArm(qq2);

        % parcialni odvodi za pozicije
        J(1:3,i)= ( T2(1:3,4)-T1(1:3,4) )/ (qq2(i)-qq1(i));
   
        % parcialni odvodi za kote
        R1=T1(1:3,1:3);
        R2=T2(1:3,1:3);
        dR = R2*R1';  % R2=dR*R1 , od orientacije 1 proti orientaciji 2 (je kot referenca)

        dR = dR / norm(dR);
   if opcija==1     
        eq = 2*log(quaternion(rotm2quat(dR))); % kvaternion pogreÅ¡ka
        [~, qB, qC, qD] = eq.parts; 
        eR = [qB, qC, qD]';  
   elseif opcija==2      
        % 2. opcija za izracun kotov (AMS knjiga slo, str 165)
        OmegaDT=-(dR-eye(3));
        eR = [OmegaDT(2,3); OmegaDT(3,1); OmegaDT(1,2)];
   elseif opcija==3       
        % 3. opcija za izracun kotov (AMS knjiga slo, str 168)
        eq =rotm2quat(dR); % kvaternion pogreÅ¡ka 
        phi=2*acos(eq(1));
        if abs(phi>0)
            eR =phi/sin(phi/2)*[eq(2:4)]';
        else
            eR = [0;0;0];
        end
 elseif opcija==4  % ?? ni ok?
        % 4. opcija za izracun kotov (AMS knjiga slo, str 168)
        quat1Kon=quaternion(rotm2quat(R1).*[1 -1 -1 -1]);
        quat2=quaternion(rotm2quat(R2)); % konjugiran
        eqq= quat2*quat1Kon; 

        [qA, qB, qC, qD] = eqq.parts; 
        eq = [qA,qB, qC, qD];  
        phi=2*acos(eq(1));
        if abs(phi>0)
            eR = -phi/sin(phi/2)*[eq(2:4)]';
        else
            eR = [0;0;0];
        end
   end     
        
        J(4:6,i)= eR / (qq2(i)-qq1(i));
     end      
         
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function optimizerTrajectory(obj, qBegin, poseGoal,flagSecundar,flagVLimit)
        % ----------------------------------------------------
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        % ----------------------------------------------------
        
       % definiram referencno trajektorijo za EE
       % ? dodam se sekundarno nalogo za bazo
       % ?? referenca za orientacijo EE-ja je sedaj kar fiksna
       w=2*2*pi/3;
       t=0:obj.dt:8; 
       xbr=0.3*t; ybr=0.5+0.1*sin(.3*w*t); % referenca za bazo
       dxbr=0.3*ones(size(t)); dybr=0.1*.3*w*cos(.3*w*t);
       ddxbr=zeros(size(t)); ddybr=-0.1*.3*w*.3*w*sin(.3*w*t);
       fibr=atan2(dybr,dxbr);
       vb_ref=sqrt(dxbr.^2+dybr.^2); 
       wb_ref=(dxbr.*ddybr-dybr.*ddxbr)./(dxbr.^2+dybr.^2);

       
       xar=0.3*t+.6; yar=.8*ones(size(t)); zar=1+0.05*sin(0.25*w*t); % referenca za roko
       dxar=0.3*ones(size(t)); dyar=zeros(size(t)); dzar=0.05*0.25*w*cos(0.25*w*t);  % odvodi reference za roko
       
       
       figure(obj.mainFig); plot(xbr,ybr,'r--')
       plot3(xar,yar,zar,'g--')
       
            % logs
            RecBase = [];
            RecArm = [];
            RecEnd = [];
            RecT=[];
            RecU=[];
            RecErrEE= [];
            
            % decompose goal matrix
            gR = poseGoal(1:3,1:3);  % orientacija je sedaj fiksna, lahko se definira tudi to
            
            qArm = qBegin(1:7);
            qBase = qBegin(8:10); % [x y phi]
            stevc=0; nShow=ceil(0.1/obj.dt);
           NN=length(t);
           
         for i=1:NN
                gP =[xar(i);yar(i);zar(i)];  % goal reference pose
                stevc=stevc+1;
                % calculate distances from goal Pose
                % ----------------------------------
                % get transformation matrix
                [~, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);
                
                % decompose current transformation matrix
                cR = rT(1:3,1:3);
                cP = rT(1:3,4);

                % rotation error
                dR = gR*cR';
                if(1)  % original ali druga opcija ??
                    dR = dR / norm(dR);
                    eq = 2*log(quaternion(rotm2quat(dR)));
                    [~, qB, qC, qD] = eq.parts;                
                    eR = [qB, qC, qD];
                else
                    OmegaDT=-(dR-eye(3));
                    eR = wrapToPi([OmegaDT(2,3), OmegaDT(3,1), OmegaDT(1,2)]);
                end
                
                
                % translation error
                eP = gP-cP;

                % controller : compose velocity vector for EE
              %  velocityEE = 3*[eP*2 ; eR'*4];  % p-regulator na poziciji EE-ja
              dRef=[dxar(i);dyar(i);dzar(i);0;0;0]; % feedforward za orientacijo je 0, ce je ta konstantna
          %    velocityEE = 1*[eP*2 ; eR'*4].*[1 1 1 1 ] + dRef;  % p-regulator + feedforward na poziciji EE-ja
              velocityEE = 1*[eP*2 ; eR'*4].*[1 1 1 1 1 1]' + dRef;  % p-regulator + feedforward na poziciji EE-ja


                % run inverse kinematics
                % ----------------------------------
                % po nasem clanku, J ocenim numericno  
                opcijaJ=2;  % opcije izracuna 1=original 2=preko R matrike AMS Knjiga, 3= preko kvaterninona AMS Knjiga, 4=preko kvatenoponov ?? ne dela za orientacijo           
                J_=obj.jacobianNumeric([qBase,qArm], opcijaJ);
                Sb=[cos(qBase(3)) 0;
                    sin(qBase(3)) 0;
                    0             1];
                S=[Sb         zeros(3,7);
                   zeros(7,2) eye(7)];
                J=J_*S;
              
                pinv_J = pinv(J);
               % pinv_J = J'*(J*J')^(-1);
 
               
              if flagSecundar==0 % brez sekundarne naloge
                 q_vel = pinv_J * velocityEE;
              elseif flagSecundar==1 || flagSecundar==12 || flagSecundar==13 % enostavna sekundarna le za bazo
                 N=eye(9)-pinv_J*J;
                 
                 q_ref=[xbr(i), ybr(i), fibr(i)]; % current reference pose for base 
                 e=[cos(qBase(3)) sin(qBase(3)) 0;...
                   -sin(qBase(3)) cos(qBase(3)) 0;... 
                    0            0      1]     * (q_ref-qBase)';    % error vector
                 e(3)=wrapToPi(e(3)); % correct angle
  
                % control 
                e_x=e(1); e_y=e(2); e_phi=e(3);
                zeta=0.9;
                g=85*.8;

                % current reference inputs  
                vref = vb_ref(i); wref = wb_ref(i);

                Kx=2*zeta*sqrt(wref^2+g*vref^2);
                Kphi=Kx;
                Ky=g;        

                v_ = vref*cos(e(3))*1 + Kx*e_x;
                w_ = wref*1 + Ky*vref*sinc(e_phi/pi)*e_y+Kphi*e_phi;
                 
                xi=[v_;w_;zeros(7,1)]; % sekundarne pseudo hitrosti;
                q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
                
                               %------------ 
                if flagSecundar==12 % se optimizacija za sklepe manipulabilnost - le za sklepe
                  % se manipulabilnost za sklepe ,numeriÄ?no doloÄ?imo gradient
                  w1=det(J*J');  % manipulabilnost zacetne lege
                  df=0.01; % parturbacija sklepa
                  for s=1:7
                      qArm2=qArm;
                      qArm2(s)=qArm2(s)+df; % parturbacija sklepa
                      J_2=obj.jacobianNumeric([qBase,qArm2], opcijaJ);
                      J2=J_2*S;
                      w2=det(J2*J2');
                      xi(s+2,1)=(w2-w1)/(df); %bolje je cimvec zato gledam smer + gradienta
                  end
                  q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
                end
              %------------  

                
                if flagSecundar==13 % se sekundarna naloga za hitrosti baze v in w (primarna pa je za hitrost ee-ja) in optimizacija maniulabilnost
                  w1=det(J*J');  % manipulabilnost zacetne lege
                  df=0.01; % parturbacija sklepa
                   
                  for s=1:7 % manipulabilnost naredim le za roko
                      qArm2=qArm;
                      qArm2(s)=qArm2(s)+df; % parturbacija sklepa
                      J_2=obj.jacobianNumeric([qBase,qArm2], opcijaJ);
                      J2=J_2*S;
                      w2=det(J2*J2');
                      xi(s+2,1)=(w2-w1)/(df); %bolje je cimvec zato gledam smer + gradienta
                  end
                  
                  v1=velocityEE; % primarni task je zelena hitrost EE
                  J1=J;          % v1=J1*dq;
                  piJ1=pinv_J; 
                  N1=N;

                  v2=[v_;w_];  % sekundarni task je zelena hitrost baze
                  J2=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0]; % v2=J2*dq;
                  J2_=J2*N1;
                  piJ2s=pinv(J2_);
                  N2=eye(9)-piJ2s*J2_;

                  % xi je tercialni task optimizacija manipulabilnosti
                  q_vel = piJ1*v1 + N1*piJ2s*(v2-J2*piJ1*v1)+N1*N2*xi;
               end
              %------------  
                
              else   % sekundarna v smeri negativnega gradienta za kriterij W
                 N=eye(9)-pinv_J*J;
                 q_ref=[xbr(i), ybr(i), fibr(i)];
                 
                 % W=(q_ref-qBase)*(q_ref-qBase)';  
                 % kriterij je vsota kvadraticnih odstopanj za lege za baz0
                 % negativni gradient glede na konfiguracijo q=[qBase,qArm]
                 gradW=[-2*(q_ref(1)-qBase(1)), -2*(q_ref(2)-qBase(2)), -2*wrapToPi(q_ref(3)-qBase(3)), zeros(1,7)];
                                  
                 lambda=1;
                 xi=-lambda*(gradW*S*N)';  % sekundarne pseudo hitrosti;
                % xi=xi.*[1 1 1 1 1 1 1 1 1]';
                q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
              end


                % calculate new pose (numerical integration)
                % ----------------------------------

                % translation and angular velocity 
                
              if flagVLimit  % omejitve hitrosti
                vMax=[0.5, 2, 2,2,2,2,2,2,2]*.7;  
                for j=1:9
                    if(abs(q_vel(j))>vMax(j))
                        q_vel(j)=vMax(j)*sign(q_vel(j));
                    end
                end
              end          
           
                RecBase = [RecBase; qBase];     % najprej shranim Å¡ele na to propagiram
                RecArm = [RecArm; qArm];     % najprej shranim Å¡ele na to propagiram
                RecEnd  = [RecEnd; cP'];
                RecT  = [RecT; t(i)];
                RecU  = [RecU; q_vel'];
                RecErrEE=  [RecErrEE; [eP' , eR]];    % Endeffector pogresek  
                
                
                
                
                base_wv = [ q_vel(1) ;
                            q_vel(2) ]; 

                % arm joints num integration
                qArm = qArm + q_vel(3:9)' * obj.dt;

                % base num integration (trapez integration)
                qBase = qBase + [base_wv(1) * cos(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(1) * sin(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(2) ]' * obj.dt;


             if(stevc>=nShow||i==NN)
                obj.animateRobot([qArm,qBase]') ; 
                pause(obj.dt*.5);   
                drawnow();
                stevc=0;
             end
               
         end
     
   % izris slik       
%   save klasikaGregor RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar 

      
      fs=14;     
      plot(RecBase(:,1),RecBase(:,2),'m:')
      plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b:')
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      figure(1),plot(RecT,RecU(:,1),RecT,RecU(:,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
      
      figure(2),plot(RecT,RecU(:,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)

   %   ee=RecBase(:,1:2)-[xbr',ybr'];
      ee=RecBase(:,1:3)-[xbr',ybr',fibr']; ee(:,3)=wrapToPi(ee(:,3));

      figure(3),plot(RecT,ee)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{bx}$$[m], $$e_{by}$$[m]','interpreter','latex','FontSize',fs)
      legend('$$e_{bx}$$','$$e_{by}$$','interpreter','latex','FontSize',fs)
     % title('base tracking error')
      
      
      eee=sqrt(ee(:,1).^2+ee(:,2).^2);
      figure(4),plot(RecT,eee)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_d$$[m]','interpreter','latex','FontSize',fs)
      title('base tracking error')

      figure(5),plot(RecT,RecEnd(:,1:3)-[xar',yar',zar'])
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{ax}$$[m], $$e_{ay}$$[m], $$e_{az}$$[m]','interpreter','latex','FontSize',fs)
      title('arm tracking error')

  
      
%    %   plot(xbr,ybr,'r',xbr,ybr,'rx',RecBase(:,1),RecBase(:,2),'mo' )
%      figure(6)
%       plot(xbr,ybr,'g--',RecBase(:,1),RecBase(:,2),'b')
%       xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
%       ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
% 
%      figure(7), hold on
%       plot3(xar,yar,zar,'g--')
%       plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b')
%       
%       plot(xbr,ybr,'g--',RecBase(:,1),RecBase(:,2),'b')
%       xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
%       ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
%       zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)
%  
% 
      
      
      
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function optimizerTrajectoryOnlyPosition(obj, qBegin, poseGoal,flagSecundar,flagVLimit)
        % ----------------------------------------------------
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        % ----------------------------------------------------
        
       % definiram referencno trajektorijo za EE
       % ? dodam se sekundarno nalogo za bazo
       % ?? referenca za orientacijo EE-ja je sedaj kar fiksna
       w=2*2*pi/3;
       t=0:obj.dt:8; 
       xbr=0.3*t; ybr=0.5+0.1*sin(.3*w*t); % referenca za bazo
       dxbr=0.3*ones(size(t)); dybr=0.1*.3*w*cos(.3*w*t);
       ddxbr=zeros(size(t)); ddybr=-0.1*.3*w*.3*w*sin(.3*w*t);
       fibr=atan2(dybr,dxbr);
       vb_ref=sqrt(dxbr.^2+dybr.^2); 
       wb_ref=(dxbr.*ddybr-dybr.*ddxbr)./(dxbr.^2+dybr.^2);

       
       xar=0.3*t+.6; yar=.8*ones(size(t)); zar=1+0.05*sin(0.25*w*t); % referenca za roko
       dxar=0.3*ones(size(t)); dyar=zeros(size(t)); dzar=0.05*0.25*w*cos(0.25*w*t);  % odvodi reference za roko
       
       
       figure(obj.mainFig); plot(xbr,ybr,'r--')
       plot3(xar,yar,zar,'g--')
       
            % logs
            RecBase = [];
            RecEnd = [];
            RecT=[];
            RecU=[];
            
            % decompose goal matrix
            gR = poseGoal(1:3,1:3);  % orientacija je sedaj fiksna, lahko se definira tudi to
            
            qArm = qBegin(1:7);
            qBase = qBegin(8:10); % [x y phi]
            stevc=0; nShow=ceil(0.1/obj.dt);
           NN=length(t);
           
         for i=1:NN
                gP =[xar(i);yar(i);zar(i)];  % goal reference pose
                stevc=stevc+1;
                % calculate distances from goal Pose
                % ----------------------------------
                % get transformation matrix
                [~, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);
                
                % decompose current transformation matrix
                cR = rT(1:3,1:3);
                cP = rT(1:3,4);

                % rotation error
                dR = gR*cR';
                if(1)  % original ali druga opcija ??
                    dR = dR / norm(dR);
                    eq = 2*log(quaternion(rotm2quat(dR)));
                    [~, qB, qC, qD] = eq.parts;                
                    eR = [qB, qC, qD];
                else
                    OmegaDT=-(dR-eye(3));
                    eR = wrapToPi([OmegaDT(2,3), OmegaDT(3,1), OmegaDT(1,2)]);
                end
                
                
                % translation error
                eP = gP-cP;

                % controller : compose velocity vector for EE
%                 %  velocityEE = 3*[eP*2 ; eR'*4];  % p-regulator na poziciji EE-ja
%                 dRef=[dxar(i);dyar(i);dzar(i);0;0;0]; % feedforward za orientacijo je 0, ce je ta konstantna
%                 % velocityEE = 1*[eP*2 ; eR'*4].*[1 1 1 1 ] + dRef;  % p-regulator + feedforward na poziciji EE-ja
%                 velocityEE = 1*[eP*2 ; eR'*4].*[1 1 1 1 1 1]' + dRef;  % p-regulator + feedforward na poziciji EE-ja
              % za EE nas zanima le pozicija
                dRef=[dxar(i);dyar(i);dzar(i)]; % feedforward 
                velocityEE = 1*[eP*2].*[1 1 1]' + dRef;  % p-regulator + feedforward na poziciji EE-ja

                
                % run inverse kinematics
                % ----------------------------------
                % po nasem clanku, J ocenim numericno  
                opcijaJ=2;  % opcije izracuna 1=original 2=preko R matrike AMS Knjiga, 3= preko kvaterninona AMS Knjiga, 4=preko kvatenoponov ?? ne dela za orientacijo           
              %  J_=obj.jacobianNumeric([qBase,qArm], opcijaJ);
                J_=obj.jacobianNumericOnlyPosition([qBase,qArm], opcijaJ);

                Sb=[cos(qBase(3)) 0;
                    sin(qBase(3)) 0;
                    0             1];
                S=[Sb         zeros(3,7);
                   zeros(7,2) eye(7)];
                J=J_*S;
              
                pinv_J = pinv(J);
               % pinv_J = J'*(J*J')^(-1);
                
              if flagSecundar==0 % brez sekundarne naloge
                 q_vel = pinv_J * velocityEE;
              elseif flagSecundar==1 || flagSecundar==12 || flagSecundar==13  % enostavna sekundarna le za bazo
                 N=eye(9)-pinv_J*J;
                 
                 q_ref=[xbr(i), ybr(i), fibr(i)]; % current reference pose for base 
                 e=[cos(qBase(3)) sin(qBase(3)) 0;...
                   -sin(qBase(3)) cos(qBase(3)) 0;... 
                    0            0      1]     * (q_ref-qBase)';    % error vector
                 e(3)=wrapToPi(e(3)); % correct angle
  
                % control 
                e_x=e(1); e_y=e(2); e_phi=e(3);
                zeta=0.9;
                g=85*.8;

                % current reference inputs  
                vref = vb_ref(i); wref = wb_ref(i);

                Kx=2*zeta*sqrt(wref^2+g*vref^2);
                Kphi=Kx;
                Ky=g;        

                v_ = vref*cos(e(3))*1 + Kx*e_x;
                w_ = wref*1 + Ky*vref*sinc(e_phi/pi)*e_y+Kphi*e_phi;
                 
                xi=[v_;w_;zeros(7,1)]; % sekundarne pseudo hitrosti;

                q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
               %------------ 
                if flagSecundar==12 % se optimizacija za sklepe manipulabilnost - le za sklepe
                  % se manipulabilnost za sklepe ,numeriÄ?no doloÄ?imo gradient
                  w1=det(J*J');  % manipulabilnost zacetne lege
                  df=0.01; % parturbacija sklepa
                  for s=1:7
                      qArm2=qArm;
                      qArm2(s)=qArm2(s)+df; % parturbacija sklepa
                      J_2=obj.jacobianNumericOnlyPosition([qBase,qArm2], opcijaJ);
                      J2=J_2*S;
                      w2=det(J2*J2');
                      xi(s+2,1)=(w2-w1)/(df); %bolje je cimvec zato gledam smer + gradienta
                  end
                  q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
                end
              %------------  
               if flagSecundar==13 % se sekundarna naloga za hitrosti baze v in w (primarna pa je za hitrost ee-ja) in optimizacija maniulabilnost
                    w1=det(J*J');  % manipulabilnost zacetne lege
                    df=0.01; % parturbacija sklepa
% za bazo manipulabilnost mogoce ni smiselna in tudi ni efekta                      
%                   qBase2=qBase'+Sb*[df;0];
%                   J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
%                   Sb2=[cos(qBase2(3)) 0;
%                     sin(qBase2(3)) 0;
%                     0             1];
%                   S=[Sb2         zeros(3,7);
%                       zeros(7,2) eye(7)];
%                   J2=J_2*S;
%                   w2=det(J2*J2');
%                   xi(1,1)=(w2-w1)/(df); 
% 
%                   qBase2=qBase'+Sb*[0;df];
%                   J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
%                   Sb2=[cos(qBase2(3)) 0;
%                     sin(qBase2(3)) 0;
%                     0             1];
%                   S=[Sb2         zeros(3,7);
%                       zeros(7,2) eye(7)];
%                   J2=J_2*S;
%                   w2=det(J2*J2');
%                   xi(2,1)=(w2-w1)/(df); 

                  
                  for s=1:7 % manipulabilnost naredim le za roko
                      qArm2=qArm;
                      qArm2(s)=qArm2(s)+df; % parturbacija sklepa
                      J_2=obj.jacobianNumericOnlyPosition([qBase,qArm2], opcijaJ);
                      J2=J_2*S;
                      w2=det(J2*J2');
                      xi(s+2,1)=(w2-w1)/(df); %bolje je cimvec zato gledam smer + gradienta
                  end
                  
                  v1=velocityEE; % primarni task je zelena hitrost EE
                  J1=J;          % v1=J1*dq;
                  piJ1=pinv_J; 
                  N1=N;

                  v2=[v_;w_];  % sekundarni task je zelena hitrost baze
                  J2=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0]; % v2=J2*dq;
                  J2_=J2*N1;
                  piJ2s=pinv(J2_);
                  N2=eye(9)-piJ2s*J2_;

                  % xi je tercialni task oz. optimizacija manipulabilnosti
                  q_vel = piJ1*v1 + N1*piJ2s*(v2-J2*piJ1*v1)+N1*N2*xi;
               end
              %------------  
                
                
                
              else   % sekundarna v smeri negativnega gradienta za kriterij W
                 N=eye(9)-pinv_J*J;
                 q_ref=[xbr(i), ybr(i), fibr(i)];
                 
                 % W=(q_ref-qBase)*(q_ref-qBase)';  
                 % kriterij je vsota kvadraticnih odstopanj za lego za bazo
                 % negativni gradient glede na konfiguracijo q=[qBase,qArm]
                 gradW=[-2*(q_ref(1)-qBase(1)), -2*(q_ref(2)-qBase(2)), -2*wrapToPi(q_ref(3)-qBase(3)), zeros(1,7)];
                                  
                 lambda=1;
                 xi=-lambda*(gradW*S*N)';  % sekundarne pseudo hitrosti;
                % xi=xi.*[1 1 1 1 1 1 1 1 1]';
                q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
              end


                % calculate new pose (numerical integration)
                % ----------------------------------

                % translation and angular velocity 
                
              if flagVLimit  % omejitve hitrosti
                vMax=[0.5, 2, 2,2,2,2,2,2,2]*1;  
                for j=1:9
                    if(abs(q_vel(j))>vMax(j))
                        q_vel(j)=vMax(j)*sign(q_vel(j));
                    end
                end
              end          
           
                base_wv = [ q_vel(1) ;
                            q_vel(2) ]; 

                % arm joints num integration
                qArm = qArm + q_vel(3:9)' * obj.dt;

                % base num integration (trapez integration)
                qBase = qBase + [base_wv(1) * cos(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(1) * sin(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(2) ]' * obj.dt;

                RecBase = [RecBase; qBase];
                RecEnd  = [RecEnd; cP'];
                RecT  = [RecT; t(i)];
                RecU  = [RecU; q_vel'];

             if(stevc>=nShow||i==NN)
                obj.animateRobot([qArm,qBase]') ; 
                pause(obj.dt*.5);   
                drawnow();
                stevc=0;
             end
               
          end
            
      fs=14;     
      plot(RecBase(:,1),RecBase(:,2),'m:')
      plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b:')
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      figure(1),plot(RecT,RecU(:,1),RecT,RecU(:,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[rad/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
      figure(2),plot(RecT,RecU(:,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)

      ee=RecBase(:,1:2)-[xbr',ybr'];
      figure(3),plot(RecT,ee)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{bx}$$[m], $$e_{by}$$[m]','interpreter','latex','FontSize',fs)
      legend('$$e_{bx}$$','$$e_{by}$$','interpreter','latex','FontSize',fs)
      title('base tracking error')
      
      eee=sqrt(ee(:,1).^2+ee(:,2).^2);
      figure(4),plot(RecT,eee)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_d$$[m]','interpreter','latex','FontSize',fs)
      title('base tracking error')

      eea=RecEnd(:,1:3)-[xar',yar',zar'];
      figure(5),plot(RecT,eea)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{ax}$$[m], $$e_{ay}$$[m], $$e_{az}$$[m]','interpreter','latex','FontSize',fs)
      title('arm tracking error')

      [gR,gP;0 0 0 1]
      rT
      
      disp('Sum sqare error for base and endeffector tracking'); 
      eeea=eea(:,1).^2+eea(:,2).^2+eea(:,3).^2;
      SSE_base=sum(eee.^2)
      SSE_arm=sum(eeea)

 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function optimizerTrajectorySeparateBase(obj, qBegin, poseGoal,flagSecundar,flagVLimit)
        % ----------------------------------------------------
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        % ----------------------------------------------------
        
       % definiram referencno trajektorijo za EE
       % ? dodam se sekundarno nalogo za bazo
       % ?? referenca za orientacijo EE-ja je sedaj kar fiksna
       w=2*2*pi/3;
       t=0:obj.dt:8; 
       xbr=0.3*t; ybr=0.5+0.1*sin(.3*w*t); % referenca za bazo
       dxbr=0.3*ones(size(t)); dybr=0.1*.3*w*cos(.3*w*t);
       ddxbr=zeros(size(t)); ddybr=-0.1*.3*w*.3*w*sin(.3*w*t);
       fibr=atan2(dybr,dxbr);
       vb_ref=sqrt(dxbr.^2+dybr.^2); 
       wb_ref=(dxbr.*ddybr-dybr.*ddxbr)./(dxbr.^2+dybr.^2);

       
       xar=0.3*t+.6; yar=.8*ones(size(t)); zar=1+0.05*sin(0.25*w*t); % referenca za roko
       dxar=0.3*ones(size(t)); dyar=zeros(size(t)); dzar=0.05*0.25*w*cos(0.25*w*t);  % odvodi reference za roko
       
       
       figure(obj.mainFig); plot(xbr,ybr,'r--')
       plot3(xar,yar,zar,'g--')
       
            % logs
            RecBase = [];
            RecEnd = [];
            RecT=[];
            RecU=[];
            
            % decompose goal matrix
            gR = poseGoal(1:3,1:3);  % orientacija je sedaj fiksna, lahko se definira tudi to
            
            qArm = qBegin(1:7);
            qBase = qBegin(8:10); % [x y phi]
            stevc=0; nShow=ceil(0.1/obj.dt);
           NN=length(t);
           
         for i=1:NN
                gP =[xar(i);yar(i);zar(i)];  % goal reference pose
                stevc=stevc+1;
                % calculate distances from goal Pose
                % ----------------------------------
                % get transformation matrix
                [baseT, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);
            %    [baseT, A1, A2, A3, A4, A5,A6, A7, rT] = obj.robot.GeometricRobot(qArm, qBase);

                % decompose current transformation matrix
                cR = rT(1:3,1:3);
                cP = rT(1:3,4);
                
              %  aT=A1*A2*A3*A4*A5*A6*A7;
                
            aT= baseT^(-1)*rT; % transformacijska le za roko   
           Ra=aT(1:3,1:3); Ta=aT(1:3,4);
   %            Ra=eye(3); Ta=zeros(3,1);
  
   
   
   
%        baseT = [cos(fibr(i)), -sin(fibr(i)), 0, xbr(i) ; % za bazo vzamem kar referencno lego
%              sin(fibr(i)),  cos(fibr(i)), 0, ybr(i) ;
%              0,         0,        1, 0.833 ;
%              0,         0,        0, 1 ];
% 
%    
%             Rb=baseT(1:3,1:3); Tb=baseT(1:3,4);
%             
%              gP=Ra* Rb'*(gP-Tb) + Ta; % referenca relativno na bazo
%              gR=Ra* Rb'*gR;      % rotacije relativno na bazo
%             gP=Ra'*( Rb'*(gP-Tb) - Ta ); % referenca relativno na roko
%             gR=Ra'*( Rb'*gR);      % rotacije relativno na bazo
%             gP=Ra'*( gP - Ta ); % referenca relativno na roko
%             gR=Ra'*( gR);      % rotacije relativno na bazo
      
%              cP=Ra* Rb'*(cP-Tb) + Ta; % lega EE relativno na bazo
%              cR=Ra* Rb'*cR;      % lega EE rotacije relativno na bazo
%             cP=Ra'*( Rb'*(cP-Tb) - Ta ); % lega EE relativno na bazo
%             cR=Ra'*( Rb'*cR);      % lega EE rotacije relativno na bazo
%             cP=Ra'*( cP - Ta ); % lega EE relativno na bazo
%             cR=Ra'*( cR);      % lega EE rotacije relativno na bazo
     
            
            

                % rotation error
                dR = gR*cR' ;
      %  dR = Rb'* dR; % relativno na bazo      
                
                if(1)  % original ali druga opcija ??
                    dR = dR / norm(dR);
                    eq = 2*log(quaternion(rotm2quat(dR)));
                    [~, qB, qC, qD] = eq.parts;                
                    eR = [qB, qC, qD]';
                else
                    OmegaDT=-(dR-eye(3));
                    eR = wrapToPi([OmegaDT(2,3), OmegaDT(3,1), OmegaDT(1,2)]);
                end
                
                
                % translation error
                eP = (gP-cP);

                % controller : compose velocity vector for EE
                %  velocityEE = 3*[eP*2 ; eR*4];  % p-regulator na poziciji EE-ja
                dRef=[dxar(i);dyar(i);dzar(i);0;0;0]; % feedforward za orientacijo je 0, ce je ta konstantna
                % velocityEE = 1*[eP*2 ; eR*4].*[1 1 1 1 ] + dRef;  % p-regulator + feedforward na poziciji EE-ja
%                velocityEE = 1*[eP*2 ; eR*4].*[1 1 1 1 1 1]' + dRef*1;  % p-regulator + feedforward na poziciji EE-ja


                % run inverse kinematics
                % ----------------------------------
                % po nasem clanku, J ocenim numericno  
                opcijaJ=1;  % opcije izracuna 1=original 2=preko R matrike AMS Knjiga, 3= preko kvaterninona AMS Knjiga, 4=preko kvatenoponov ?? ne dela za orientacijo           
                J=obj.jacobianNumericOnlyArm(qArm, opcijaJ);
%                  J_=obj.jacobianNumeric([qBase,qArm], opcijaJ);
%                 Sb=[cos(qBase(3)) 0;
%                     sin(qBase(3)) 0;
%                     0             1];
%                 S=[Sb         zeros(3,7);
%                    zeros(7,2) eye(7)];
%                 J=J_*S;
              
              %  J=J(:,end-6:end);
             %   J=J_(:,end-6:end);
 
                
% J = obj.robot.v2_jacobi_panda_pmb2_vw (qArm, qBase);               
                
                pinv_J = pinv(J);
               % pinv_J = J'*(J*J')^(-1);
                
              if flagSecundar==0 % brez sekundarne naloge
                 error('primarna naloga oz regulator za bazo ni definiran');                  
                 q_vel = pinv_J * velocityEE;
              elseif flagSecundar==1 % enostavna sekundarna le za bazo
                  
             %    N=eye(9)-pinv_J*J;
                 
                 q_ref=[xbr(i), ybr(i), fibr(i)]; % current reference pose for base 
                 e=[cos(qBase(3)) sin(qBase(3)) 0;...
                   -sin(qBase(3)) cos(qBase(3)) 0;... 
                    0            0      1]     * (q_ref-qBase)';    % error vector
                 e(3)=wrapToPi(e(3)); % correct angle
  
                % control 
                e_x=e(1); e_y=e(2); e_phi=e(3);
                zeta=0.9;
                g=85*.8;

                % current reference inputs  
                vref = vb_ref(i); wref = wb_ref(i);

                Kx=2*zeta*sqrt(wref^2+g*vref^2);
                Kphi=Kx;
                Ky=g;        

                v_ = vref*cos(e(3))*1 + Kx*e_x;
                w_ = wref*1 + Ky*vref*sinc(e_phi/pi)*e_y+Kphi*e_phi;
                 
             %   xi=[zeros(9,1)]; % sekundarne pseudo hitrosti;
            %    xi=[v_;w_;zeros(7,1)]; % sekundarne pseudo hitrosti;
           %     xi=[zeros(7,1)]; % sekundarne pseudo hitrosti;

%             velocityEE = 1*[eP*2 ; 1*eR*4].*[1 1 1 1 1 1]' + 0* [Rb'*dRef(1:3); dRef(4:5); dRef(6)-w_  ];  % p-regulator + feedforward na poziciji EE-ja
             velocityEE = 1*[eP*2 ; 1*eR*4].*[1 1 1 1 1 1]' + 0* [Ra'*dRef(1:3); dRef(4:5); dRef(6)-w_  ];  % p-regulator + feedforward na poziciji EE-ja
               
%                 velocityEE(1:2,:)=velocityEE(1:2,:)-v_*[cos(qBase(3));sin(qBase(3))]; % relativna hitrost za roko, ker je ze upostevana v bazi
%                 velocityEE(6,:)=velocityEE(6,:)-w_; % relativna hitrost za roko, ker je ze upostevana v bazi

%                 velocityEE(1:2,:)=velocityEE(1:2,:)+v_*[cos(qBase(3));sin(qBase(3))]; % relativna hitrost za roko, ker je ze upostevana v bazi
%                 velocityEE(6,:)=velocityEE(6,:)+w_; % relativna hitrost za roko, ker je ze upostevana v bazi




                
      %        velocityEE=velocityEE-J*[v_;w_;zeros(7,1)];
      %          q_vel = [  pinv_J * velocityEE + N*xi];  % ?? le sekundarna za roko, ki je sedaj ni sekundarna naloga
                q_vel = [  pinv_J * velocityEE ];  % 

         %        q_vel = [  pinv_J * (velocityEE - [v_*cos(qBase(3));v_*sin(qBase(3));0 ;0 ;0; w_] ) ];  % 
               
          %      q_vel=q_vel-pinv_J*[v_*cos(qBase(3));v_*sin(qBase(3));0 ;0 ;0; w_];
                
                
                
        %        q_vel = [ v_;w_; q_vel(3:end)];  % ?? le sekundarna za roko, ki je sedaj ni sekundarna naloga
                q_vel = [ v_;w_; q_vel];  % ?? le sekundarna za roko, ki je sedaj ni sekundarna naloga
                
         %   q_vel(1:2,1)= q_vel(1:2)-v_*[cos(qBase(3));sin(qBase(3))]; % relativna hitrost za roko
         
         
              else   % sekundarna v smeri negativnega gradienta za kriterij W
 error('sekundarna naloga se ni implementirana za roko');                  
%                   N=eye(7)-pinv_J*J;
%                  q_ref=[xbr(i), ybr(i), fibr(i)];
%                  
%                  % W=(q_ref-qBase)*(q_ref-qBase)';  
%                  % kriterij je vsota kvadraticnih odstopanj za lege za bazo
%                  % negativni gradient glede na konfiguracijo q=[qBase,qArm]
%                 % gradW=[-2*(q_ref(1)-qBase(1)), -2*(q_ref(2)-qBase(2)), -2*wrapToPi(q_ref(3)-qBase(3)), zeros(1,7)];
%                  gradW=[ zeros(1,7)]; % ni sekindarne
%                  
%                  lambda=1;
%                  xi=-lambda*(gradW*S*N)';  % sekundarne pseudo hitrosti;
%                 % xi=xi.*[1 1 1 1 1 1 1 1 1]';
%                 q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
              end


                % calculate new pose (numerical integration)
                % ----------------------------------

                % translation and angular velocity 
                
              if flagVLimit  % omejitve hitrosti
                vMax=[0.5, 2, 2,2,2,2,2,2,2]*1;  
                for j=1:9
                    if(abs(q_vel(j))>vMax(j))
                        q_vel(j)=vMax(j)*sign(q_vel(j));
                    end
                end
              end          
           
                base_wv = [ q_vel(1) ;
                            q_vel(2) ]; 

                % arm joints num integration
                qArm = qArm + q_vel(3:9)' * obj.dt;

                % base num integration (trapez integration)
                qBase = qBase + [base_wv(1) * cos(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(1) * sin(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(2) ]' * obj.dt;

                RecBase = [RecBase; qBase];
                
         %       RecEnd  = [RecEnd; cP'];
         RecEnd  = [RecEnd; rT(1:3,4)'];
                
                
                RecT  = [RecT; t(i)];
                RecU  = [RecU; q_vel'];

             if(stevc>=nShow||i==NN)
                obj.animateRobot([qArm,qBase]') ; 
                pause(obj.dt*.5);   
                drawnow();
                stevc=0;
             end
               
          end
            
      fs=14;     
      plot(RecBase(:,1),RecBase(:,2),'m:')
      plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b:')
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      figure(1),plot(RecT,RecU(:,1),RecT,RecU(:,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[rad/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
      figure(2),plot(RecT,RecU(:,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)

      ee=RecBase(:,1:2)-[xbr',ybr'];
      figure(3),plot(RecT,ee)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{bx}$$[m], $$e_{by}$$[m]','interpreter','latex','FontSize',fs)
      legend('$$e_{bx}$$','$$e_{by}$$','interpreter','latex','FontSize',fs)
      title('base tracking error')
      
      eee=sqrt(ee(:,1).^2+ee(:,2).^2);
      figure(4),plot(RecT,eee)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_d$$[m]','interpreter','latex','FontSize',fs)
      title('base tracking error')

      eea=RecEnd(:,1:3)-[xar',yar',zar'];
      figure(5),plot(RecT,eea)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{ax}$$[m], $$e_{ay}$$[m], $$e_{az}$$[m]','interpreter','latex','FontSize',fs)
      title('arm tracking error')

      [gR,gP;0 0 0 1]
      rT
      
      
      disp('Sum sqare error for base and endeffector tracking'); 
      eeea=eea(:,1).^2+eea(:,2).^2+eea(:,3).^2;
      SSE_base=sum(eee.^2)
      SSE_arm=sum(eeea)
      
      
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function optimizerTrajectoryPotencialBase(obj, qBegin, poseGoal,flagSecundar,flagVLimit, CMG,CMGmax,space,resolution,flagOnlyPositionEE,armRef)
        % ----------------------------------------------------
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        % ----------------------------------------------------
      
   %   flagOnlyPositionEE=1;            
          
      vMax=[0.8, 4, 2,2,2,2,2,2,2]*1.5;   % omejitve hitrosti
      aMax=[1, 2, 2,2,2,2,2,2,2]*1;       % omejitve pospeskov
      q_velOld=zeros(9,1);
        
         
       % referenca za roko
       % ?? referenca za orientacijo EE-ja je sedaj kar fiksna, definirana v poseGoal
       t  =armRef(1,:);
       xar=armRef(2,:);yar=armRef(3,:);zar=armRef(4,:);
       dxar=armRef(5,:);dyar=armRef(6,:);dzar=armRef(7,:);
 
       xbr=[]; ybr=[]; fibr=[];
       
        figure(obj.mainFig); 
        plot3(xar,yar,zar,'g--')
       
        % logs
        RecBase = [];RecArm = [];RecEnd = [];
        RecT=[];RecU=[];RecErrEE= [];RecMani= [];
        RecEulerR=[]; RecEuler=[];
        
        
        
        % decompose goal matrix
        gR = poseGoal(1:3,1:3);  % orientacija je sedaj fiksna, lahko se definira, da se spreminja tudi to

        qArm = qBegin(1:7);
        qBase = qBegin(8:10); % [x y phi]
        stevc=0; nShow=ceil(0.1/obj.dt)*5;
        NN=length(t);
           
      for i=1:NN
            gP =[xar(i);yar(i);zar(i)];  % EE reference pose
            stevc=stevc+1;
            % calculate distances from goal Pose
            % ----------------------------------
            % get transformation matrix
            [~, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);

            % decompose current transformation matrix
            cR = rT(1:3,1:3);
            cP = rT(1:3,4);

            % rotation error
            dR = gR*cR';
            if(1)  % original ali druga opcija ??
                dR = dR / norm(dR);
                eq = 2*log(quaternion(rotm2quat(dR)));
                [~, qB, qC, qD] = eq.parts;                
                eR = [qB, qC, qD];
            else
                OmegaDT=-(dR-eye(3));
                eR = wrapToPi([OmegaDT(2,3), OmegaDT(3,1), OmegaDT(1,2)]);
            end

            % translation error
            eP = gP-cP;

            if flagOnlyPositionEE           % za EE nas zanima le pozicija         
               dRef=[dxar(i);dyar(i);dzar(i)]; % feedforward 
               velocityEE = 1*[eP*2].*[1 1 1]' + dRef;  % p-regulator + feedforward na poziciji EE-ja
            else
               dRef=[dxar(i);dyar(i);dzar(i);0;0;0]; % feedforward za orientacijo je 0, ce je ta konstantna
               velocityEE = 1*[eP*2 ; eR'*2].*[1 1 1 1 1 1]' + dRef;  % p-regulator + feedforward na poziciji EE-ja
            %    velocityEE = 1*[eP*2 ; eR'*1].*[1 1 1 1 1 1]' + dRef;  % p-regulator + feedforward na poziciji EE-ja
           end
              
            % run inverse kinematics
            % ----------------------------------
            % po nasem clanku, J ocenim numericno  
            opcijaJ=2;  % opcije izracuna 1=original 2=preko R matrike AMS Knjiga, 3= preko kvaterninona AMS Knjiga, 4=preko kvaternionov ?? ne dela za orientacijo           

            if flagOnlyPositionEE   
                J_=obj.jacobianNumericOnlyPosition([qBase,qArm], opcijaJ);
            else
                J_=obj.jacobianNumeric([qBase,qArm], opcijaJ);
            end

            Sb=[cos(qBase(3)) 0;
                sin(qBase(3)) 0;
                0             1];
            S=[Sb         zeros(3,7);
               zeros(7,2) eye(7)];
            J=J_*S;

           % pinv_J = pinv(J); %%% pinv_J = J'*(J*J')^(-1);

           w1=sqrt(det(J*J'));  % manipulabilnost zacetne lege

           
           pinv_J=obj.pseudoInverzG(J,w1); % clanek: Mashali_2014
               
           if flagSecundar==0 % brez sekundarne naloge
                 q_vel = pinv_J * velocityEE;
           elseif flagSecundar==1 || flagSecundar==12 || flagSecundar==13 % enostavna sekundarna le za bazo
                 N=eye(9)-pinv_J*J;
                                 
                 q_velPrimar=pinv_J * velocityEE;  

                % dolocim gradient
               % cc=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(qBase(1),qBase(2),CMG,CMGmax,space,resolution);
                cc=tmp_GetInterpolatedBiLinearCostToGoalAndGrad_Test3(qBase(1),qBase(2),CMG,CMGmax,space,resolution);
                smer=-cc(2:3);

                fi_ref=atan2(smer(2),smer(1));
                efi=fi_ref-qBase(3);
                efi=wrapToPi(efi);
                
%                 w_ = efi*3;  % regulator po kotu
%                 v_=q_velPrimar(1)*1.1; % hitrost baze naredim cca isto kot jo doloci primarna naloga
%                 v_=v_*abs(cos(efi/1.0));

% scenarij==3
                w_ = efi*3;  % regulator po kotu
                v_=q_velPrimar(1)*1.2; % hitrost baze naredim cca isto kot jo doloci primarna naloga
                v_=v_*abs(cos(efi/1.0));
           
                
                xi=[v_;w_;zeros(7,1)]; % sekundarne pseudo hitrosti;
    
                
                q_vel1 = q_velPrimar + N*xi;  % roka prva prioriteta
                
             
                if 0 
                      v_=q_vel1(1); % za hitrost baze vzamem kar tisto izracunane iz roke 
                     % v_=q_velPrimar(1);

                      v2=[v_;w_];  % primarni task je zelena hitrost baze
                      J2=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0]; % v2=J2*dq;
                      piJ2=pinv(J2);
                      N2=eye(9)-piJ2*J2;

                      xi2=pinv_J * velocityEE;
                      q_vel2 = piJ2 * v2 + N2*xi2;  % baza prva prioriteta
                end
         
                q_vel=  q_vel1 ; % primarna naloga je roka
            %    q_vel=  q_vel2 ; % primarna naloga je baza
                
                
                %------------ 

                
                if flagSecundar==13 % se sekundarna naloga za hitrosti baze v in w (primarna pa je za hitrost ee-ja) in optimizacija maniulabilnost
                  df=0.01; % parturbacija sklepa
                  xi=[0;0;zeros(7,1)]; % sekundarne pseudo hitrosti;
         
                  % optimizacija manipulabilnosti, najprej za bazo, nato za roko                     
                  if 1
                     qBase2=qBase'+Sb*[df;0]; % parturbacija v za bazo
                     % J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
    % to popravi !                  
                      if flagOnlyPositionEE   
                         J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
                      else
                         J_2=obj.jacobianNumeric([qBase2',qArm], opcijaJ);
                      end


                      Sb2=[cos(qBase2(3)) 0;
                           sin(qBase2(3)) 0;
                           0             1];
                      S=[Sb2         zeros(3,7);
                         zeros(7,2)  eye(7)];
                      J2=J_2*S;
                      w2=sqrt(det(J2*J2'));
                      xi(1,1)=(w2-w1)/(df);   

                      qBase2=qBase'+Sb*[0;df]; % parturbacija w za bazo
                    %  J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
    % to popravi !                  
                      if flagOnlyPositionEE   
                         J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
                      else
                         J_2=obj.jacobianNumeric([qBase2',qArm], opcijaJ);
                      end


                      Sb2=[cos(qBase2(3)) 0;
                           sin(qBase2(3)) 0;
                           0             1];
                      S=[Sb2         zeros(3,7);
                         zeros(7,2)  eye(7)];
                      J2=J_2*S;
                      w2=sqrt(det(J2*J2'));
                      xi(2,1)=(w2-w1)/(df); 
                  end
                  
                  for s=1:7 % manipulabilnost naredim le za roko
                      qArm2=qArm;
                      qArm2(s)=qArm2(s)+df; % parturbacija sklepa
                      if flagOnlyPositionEE   
                         J_2=obj.jacobianNumericOnlyPosition([qBase,qArm2], opcijaJ);  
                      else      
                         J_2=obj.jacobianNumeric([qBase,qArm2], opcijaJ);
                      end
                      J2=J_2*S;
                      w2=sqrt(det(J2*J2'));
                      xi(s+2,1)=(w2-w1)/(df); %bolje je cimvec zato gledam smer + gradienta
                  end
                  
                  v1=velocityEE; % primarni task je zelena hitrost EE
                  J1=J;          % v1=J1*dq;
                  piJ1=pinv_J;
                  N1=N;

                  v2=[v_;w_];  % sekundarni task je zelena hitrost baze
                  J2=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0]; % v2=J2*dq;
                  J2_=J2*N1;
                  manipulJ2_=sqrt(det(J2_*J2_'));
                  piJ2s=obj.pseudoInverzG(J2_,manipulJ2_);%piJ2s=pinv(J2_);
                  N2=eye(9)-piJ2s*J2_;

                  % xi je tercialni task optimizacija manipulabilnosti
                  q_vel = piJ1*v1 + N1*piJ2s*(v2-J2*piJ1*v1)+N1*N2*xi;
               end
              %------------  
                
          else   % sekundarna v smeri negativnega gradienta za kriterij W
                 N=eye(9)-pinv_J*J;
                 q_ref=[xbr(i), ybr(i), fibr(i)];
                 
                 % W=(q_ref-qBase)*(q_ref-qBase)';  
                 % kriterij je vsota kvadraticnih odstopanj za lege za baz0
                 % negativni gradient glede na konfiguracijo q=[qBase,qArm]
                 gradW=[-2*(q_ref(1)-qBase(1)), -2*(q_ref(2)-qBase(2)), -2*wrapToPi(q_ref(3)-qBase(3)), zeros(1,7)];
                                  
                 lambda=1;
                 xi=-lambda*(gradW*S*N)';  % sekundarne pseudo hitrosti;
                % xi=xi.*[1 1 1 1 1 1 1 1 1]';
                q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
           end


        % calculate new pose (numerical integration)
        % ----------------------------------

        % translation and angular velocity 
                
     
     % omejitve pospeskov in hitrosti
      if flagVLimit  % omejitve pospeska
        acc=(q_vel-q_velOld)/obj.dt;     
        for j=1:9
            if(abs(acc(j))>aMax(j))
                q_vel(j)=q_velOld(j)+sign(acc(j))*aMax(j)*obj.dt;
            end
        end
      end          

      if flagVLimit  % omejitve hitrosti  [v w a1 a2.. a7]
        for j=1:9
            if(abs(q_vel(j))>vMax(j))
                q_vel(j)=vMax(j)*sign(q_vel(j));
            end
        end
      end          
             
    q_velOld=q_vel;          
              
                RecBase = [RecBase; qBase];     % najprej shranim Å¡ele na to propagiram
                RecArm = [RecArm; qArm];     % najprej shranim Å¡ele na to propagiram
                RecEnd  = [RecEnd; cP'];
                RecT  = [RecT; t(i)];
                RecU  = [RecU; q_vel'];
                RecErrEE=  [RecErrEE; [eP' , eR]];    % Endeffector pogresek  
                RecMani= [RecMani,w1];  %manipulabilnost

                RecEulerR=[RecEulerR; DCM2Euler(gR')]; %Euler koti 321, referenca
                RecEuler=[RecEuler; DCM2Euler(cR')];   %Euler koti 321
 

     
      
                
                base_wv = [ q_vel(1) ;
                            q_vel(2) ]; 
flagNoise=0;
  if flagNoise    
     Q=diag([0.3, 0.3 + 0.4*abs(q_vel(2)) ]) ;  % standard deviation where at higher angular velocities applied to the system the noice linearly increses for nagular velocity 
     base_wv = base_wv + Q*randn(2,1);     
  end                     
                        
                        
                % arm joints num integration
                qArm = qArm + q_vel(3:9)' * obj.dt;

                % base num integration (trapez integration)
                qBase = qBase + [base_wv(1) * cos(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(1) * sin(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(2) ]' * obj.dt;

            %    qBase = qBase + []*randn(2,1);             
                             

             if(stevc>=nShow||i==NN)
                obj.animateRobot([qArm,qBase]') ; 
                %pause(obj.dt*.05);   
                drawnow();
                stevc=0;
               % pause
             end
               
       end
     
   % izris slik  
   % hranjenje rezultatov
%   save Potencialno1NoSecundar RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler

  % save Potencialno1 RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler

  %  save PotencialnoScenarij3o RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler
  %  save PotencialnoScenarij3 RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler

  %   save PotencialnoScenarij1 RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler

  %   save PotencialnoScenarij4 RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler

   %   save PotencialnoScenarij1m RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler
 
   %    save PotencialnoScenarij2Sum RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler
  
   
   
      fs=14;     
      plot(RecBase(:,1),RecBase(:,2),'m:')
      plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b:')
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      figure(1),plot(RecT,RecU(:,1),RecT,RecU(:,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
      
      figure(2),plot(RecT,RecU(:,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)

      
%       figure(3),plot(RecBase(:,1),RecBase(:,2),'b')
%       xlabel('$$x_b$$[m]','interpreter','latex','FontSize',fs)
%       ylabel('$$y_b$$[m]','interpreter','latex','FontSize',fs)
      
      
%       figure(4),plot3(xar',yar',zar', 'g-',RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b')
%       xlabel('$$x_a$$[m]','interpreter','latex','FontSize',fs)
%       ylabel('$$y_a$$[m]','interpreter','latex','FontSize',fs)
%       zlabel('$$z_a$$[m]','interpreter','latex','FontSize',fs)
 
      
      figure,plot(RecT,RecArm)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$${\theta_i}$$[1]','interpreter','latex','FontSize',fs)
      legend('$${\theta_1}$$','$${\theta_2}$$','$${\theta_3}$$','$${\theta_4}$$','$${\theta_5}$$','$${\theta_6}$$','$${\theta_7}$$','interpreter','latex','FontSize',fs)
     
      
   

%       ea=RecEnd(:,1:3)-[xar',yar',zar'];
%       eaNorm=sqrt(ea(:,1).^2+ea(:,2).^2+ea(:,3).^2);
% %       figure(5),plot(RecT,ea)
% %       xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
% %       ylabel('$$e_{ax}$$[m], $$e_{ay}$$[m], $$e_{az}$$[m]','interpreter','latex','FontSize',fs)
%        figure,plot(RecT,eaNorm)
%        xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
%        ylabel('$$e_{dist_a}$$[m]','interpreter','latex','FontSize',fs)
%     %  title('arm tracking error')

      eea1=sqrt(RecErrEE(:,1).^2+RecErrEE(:,2).^2+RecErrEE(:,3).^2); % pozicijski pogresek, % kotni pogresek
      eea2=sqrt(RecErrEE(:,3).^2+RecErrEE(:,4).^2+RecErrEE(:,5).^2);
      figure,plot(RecT,eea1,RecT,eea2)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{dist}$$[m], $$e_{\phi}$$[rad]','interpreter','latex','FontSize',fs)
      legend('$$e_{dist}$$','$$e_{\phi}$$','interpreter','latex','FontSize',fs)
  
    
    
  
      figure,plot(RecT,RecMani)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('Manipulability','interpreter','latex','FontSize',fs)

      
      figure,plot(RecT,RecEulerR,'--',RecT,RecEuler)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\alpha_{x}$$, $$\alpha_{y}$$, $$\alpha_{z}$$ [1]','interpreter','latex','FontSize',fs)
      legend('$$\alpha_{xr}$$','$$\alpha_{yr}$$','$$\alpha_{zr}$$','$$\alpha_{x}$$','$$\alpha_{y}$$','$$\alpha_{z}$$','interpreter','latex','FontSize',fs)

 
      
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pinv_J=pseudoInverzG(obj,J,w1)
    % J - jacobian, w1 - manipulabilnist
           w10=0.38; % mejna manipulabilnost % ???
           if(w1<w10)
              K0=0.01;  % scale factor singularity ???   
              Ksr=K0*(1-w1/w10)^2; 
              I=eye(size(J,1));
              pinv_J = J'*( J*J' + Ksr*I )^(-1);  % clanek: Mashali_2014
           else
              pinv_J = pinv(J);
           end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function optimizerTockaPotencialBase(obj, qBegin, poseGoal,flagSecundar,flagVLimit, CMG,CMGmax,space,resolution,flagOnlyPositionEE,armRef)
        % ----------------------------------------------------
        % run classical differential kinematics in loop until robot
        % converges to searched position
        %
        % qBegin - [q1 ... q7 x y phi]
        %
        % poseGoal - [R1 ... ;
        %                ...
        %                ... 1]
        %
        % poseBegin - [R1 ... ;
        %                ...
        %                ... 1]
        % ----------------------------------------------------
      
          
      vMax=[0.8, 4, 2,2,2,2,2,2,2]*1;   % omejitve hitrosti
      aMax=[1, 2, 2,2,2,2,2,2,2]*1;       % omejitve pospeskov
      q_velOld=zeros(9,1);
        
         
       % referenca za roko
       % ?? referenca za orientacijo EE-ja je sedaj kar fiksna, definirana v poseGoal
     %  t  =armRef(1,:);
       xar=armRef(1);yar=armRef(2);zar=armRef(3);
     %  dxar=armRef(5,:);dyar=armRef(6,:);dzar=armRef(7,:);
 
       xbr=[]; ybr=[]; fibr=[];
       
        figure(obj.mainFig); 
        plot3(xar,yar,zar,'gx')
       
        % logs
        RecBase = [];RecArm = [];RecEnd = [];
        RecT=[];RecU=[];RecErrEE= [];RecMani= [];
        RecEulerR=[]; RecEuler=[];
        
        
        
        % decompose goal matrix
        gR = poseGoal(1:3,1:3);  % orientacija je sedaj fiksna, lahko se definira, da se spreminja tudi to

        qArm = qBegin(1:7);
        qBase = qBegin(8:10); % [x y phi]
        stevc=0; nShow=ceil(0.1/obj.dt)*5;
      
        NN=25/obj.dt; % do tu vodim
        t=0;
      for i=1:NN
            gP =[xar;yar;zar];  % EE reference pose
            stevc=stevc+1;
            % calculate distances from goal Pose
            % ----------------------------------
            % get transformation matrix
            [~, ~, ~, ~, ~, ~, ~, ~, rT] = obj.robot.GeometricRobot(qArm, qBase);

            % decompose current transformation matrix
            cR = rT(1:3,1:3);
            cP = rT(1:3,4);

            % rotation error
            dR = gR*cR';
            if(1)  % original ali druga opcija ??
                dR = dR / norm(dR);
                eq = 2*log(quaternion(rotm2quat(dR)));
                [~, qB, qC, qD] = eq.parts;                
                eR = [qB, qC, qD];
            else
                OmegaDT=-(dR-eye(3));
                eR = wrapToPi([OmegaDT(2,3), OmegaDT(3,1), OmegaDT(1,2)]);
            end

            % translation error
            eP = gP-cP;

            if flagOnlyPositionEE           % za EE nas zanima le pozicija         
               velocityEE = 1*[eP*2] ;  % p-regulator  na poziciji EE-ja
            else
               velocityEE = 1*[eP*2 ; eR'*4] ;  % p-regulator na poziciji EE-ja
               
%                distN=norm(eP);   % omejim skakanje roke, Ä?e je daleÄ? od reference
%                if distN>1
%                  velocityEE(4:6)=[0;0;0]; 
%                else
%                  velocityEE(4:6)=velocityEE(4:6)*(1-norm(eP));
%                end
               
            end
              
            
            
            
            % run inverse kinematics
            % ----------------------------------
            % po nasem clanku, J ocenim numericno  
            opcijaJ=2;  % opcije izracuna 1=original 2=preko R matrike AMS Knjiga, 3= preko kvaterninona AMS Knjiga, 4=preko kvaternionov ?? ne dela za orientacijo           

            if flagOnlyPositionEE   
                J_=obj.jacobianNumericOnlyPosition([qBase,qArm], opcijaJ);
            else
                J_=obj.jacobianNumeric([qBase,qArm], opcijaJ);
            end

            Sb=[cos(qBase(3)) 0;
                sin(qBase(3)) 0;
                0             1];
            S=[Sb         zeros(3,7);
               zeros(7,2) eye(7)];
            J=J_*S;


           w1=sqrt(det(J*J'));  % manipulabilnost zacetne lege

           
           pinv_J=obj.pseudoInverzG(J,w1); % clanek: Mashali_2014
               
           if flagSecundar==0 % brez sekundarne naloge
                 q_vel = pinv_J * velocityEE;
           elseif flagSecundar==1 || flagSecundar==13 % enostavna sekundarna le za bazo
                 N=eye(9)-pinv_J*J;
                                 
                 q_velPrimar=pinv_J * velocityEE;  

                % dolocim gradient
                cc=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(qBase(1),qBase(2),CMG,CMGmax,space,resolution);
                smer=-cc(2:3);

                fi_ref=atan2(smer(2),smer(1));
                efi=fi_ref-qBase(3);
                efi=wrapToPi(efi);
                w_ = efi*3;  % regulator po kotu


                v_=q_velPrimar(1)*1.1; % hitrost baze naredim cca isto kot jo doloci primarna naloga

                v_=v_*abs(cos(efi/1.0));
                xi=[v_;w_;zeros(7,1)]; % sekundarne pseudo hitrosti;
    

               distN=norm(eP(1:2));   % omejim skakanje roke, Ä?e je daleÄ? od reference
               if distN>1
                   v_= vMax(1);
                  % v_=v_*abs(cos(efi/1.0));
                   xi=[v_;w_;zeros(7,1)]; % sekundarne pseudo hitrosti
                   q_vel1 = xi;  % roka prva prioriteta               
               else
                   q_vel1 = q_velPrimar + N*xi;  % roka prva prioriteta
               end
     
                
                
          %      q_vel1 = q_velPrimar + N*xi;  % roka prva prioriteta
                
                
                
                
                
             
                if 0 
                    
                      v_=q_vel1(1); % za hitrost baze vzamem kar tisto izracunane iz roke 
                     % v_=q_velPrimar(1);                     
                      
                      v2=[v_;w_];  % primarni task je zelena hitrost baze
                      J2=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0]; % v2=J2*dq;
                      piJ2=pinv(J2);
                      N2=eye(9)-piJ2*J2;

                      xi2=pinv_J * velocityEE;
                      q_vel2 = piJ2 * v2 + N2*xi2;  % baza prva prioriteta
                end
         
                q_vel=  q_vel1 ; % primarna naloga je roka
               % q_vel=  q_vel2 ; % primarna naloga je baza
                
                
                %------------ 

                
                if flagSecundar==13 % se sekundarna naloga za hitrosti baze v in w (primarna pa je za hitrost ee-ja) in optimizacija maniulabilnost
                  df=0.01; % parturbacija sklepa
                  xi=[0;0;zeros(7,1)]; % sekundarne pseudo hitrosti;
         
                  % optimizacija manipulabilnosti                      
                  qBase2=qBase'+Sb*[df;0];
                 % J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
                  if flagOnlyPositionEE   
                     J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
                  else
                     J_2=obj.jacobianNumeric([qBase2',qArm], opcijaJ);
                  end
                  
                  
                  Sb2=[cos(qBase2(3)) 0;
                       sin(qBase2(3)) 0;
                       0             1];
                  S=[Sb2         zeros(3,7);
                     zeros(7,2)  eye(7)];
                  J2=J_2*S;
                  w2=sqrt(det(J2*J2'));
                  xi(1,1)=(w2-w1)/(df); 

                  qBase2=qBase'+Sb*[0;df];
                 % J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
                  if flagOnlyPositionEE   
                     J_2=obj.jacobianNumericOnlyPosition([qBase2',qArm], opcijaJ);
                  else
                     J_2=obj.jacobianNumeric([qBase2',qArm], opcijaJ);
                  end

                  Sb2=[cos(qBase2(3)) 0;
                       sin(qBase2(3)) 0;
                       0             1];
                  S=[Sb2         zeros(3,7);
                     zeros(7,2)  eye(7)];
                  J2=J_2*S;
                  w2=sqrt(det(J2*J2'));
                  xi(2,1)=(w2-w1)/(df); 
        
                  for s=1:7 % manipulabilnost naredim le za roko
                      qArm2=qArm;
                      qArm2(s)=qArm2(s)+df; % parturbacija sklepa
                      if flagOnlyPositionEE   
                         J_2=obj.jacobianNumericOnlyPosition([qBase,qArm2], opcijaJ);  
                      else      
                         J_2=obj.jacobianNumeric([qBase,qArm2], opcijaJ);
                      end
                      J2=J_2*S;
                      w2=sqrt(det(J2*J2'));
                      xi(s+2,1)=(w2-w1)/(df); %bolje je cimvec zato gledam smer + gradienta
                  end
                  
                  v1=velocityEE; % primarni task je zelena hitrost EE
                  J1=J;          % v1=J1*dq;
                  piJ1=pinv_J;
                  N1=N;

                  v2=[v_;w_];  % sekundarni task je zelena hitrost baze
                  J2=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0]; % v2=J2*dq;
                  J2_=J2*N1;
                  manipulJ2_=sqrt(det(J2_*J2_'));
                  piJ2s=obj.pseudoInverzG(J2_,manipulJ2_);%piJ2s=pinv(J2_);
                  N2=eye(9)-piJ2s*J2_;

                  % xi je tercialni task optimizacija manipulabilnosti
                  q_vel = piJ1*v1 + N1*piJ2s*(v2-J2*piJ1*v1)+N1*N2*xi;
               end
              %------------  
                
          else   % sekundarna v smeri negativnega gradienta za kriterij W
                 N=eye(9)-pinv_J*J;
                 q_ref=[xbr(i), ybr(i), fibr(i)];
                 
                 % W=(q_ref-qBase)*(q_ref-qBase)';  
                 % kriterij je vsota kvadraticnih odstopanj za lege za baz0
                 % negativni gradient glede na konfiguracijo q=[qBase,qArm]
                 gradW=[-2*(q_ref(1)-qBase(1)), -2*(q_ref(2)-qBase(2)), -2*wrapToPi(q_ref(3)-qBase(3)), zeros(1,7)];
                                  
                 lambda=1;
                 xi=-lambda*(gradW*S*N)';  % sekundarne pseudo hitrosti;
                % xi=xi.*[1 1 1 1 1 1 1 1 1]';
                q_vel = pinv_J * velocityEE + N*xi;  % se sekundarna naloga
           end


        % calculate new pose (numerical integration)
        % ----------------------------------

        % translation and angular velocity 
                
     
     % omejitve pospeskov in hitrosti
      if flagVLimit  % omejitve hitrosti
        acc=(q_vel-q_velOld)/obj.dt;     
        for j=1:9
            if(abs(acc(j))>aMax(j))
                q_vel(j)=q_velOld(j)+sign(acc(j))*aMax(j)*obj.dt;
            end
        end
      end          

      if flagVLimit  % omejitve hitrosti  [v w a1 a2.. a7]
        for j=1:9
            if(abs(q_vel(j))>vMax(j))
                q_vel(j)=vMax(j)*sign(q_vel(j));
            end
        end
      end          
             
    q_velOld=q_vel;          
              
                RecBase = [RecBase; qBase];     % najprej shranim Å¡ele na to propagiram
                RecArm = [RecArm; qArm];     % najprej shranim Å¡ele na to propagiram
                RecEnd  = [RecEnd; cP'];
                RecT  = [RecT; t];
                RecU  = [RecU; q_vel'];
                RecErrEE=  [RecErrEE; [eP' , eR]];    % Endeffector pogresek  
                RecMani= [RecMani,w1];  %manipulabilnost

                RecEulerR=[RecEulerR; DCM2Euler(gR')]; %Euler koti 321, referenca
                RecEuler=[RecEuler; DCM2Euler(cR')];   %Euler koti 321
 

     
      
                
                base_wv = [ q_vel(1) ;
                            q_vel(2) ]; 

                % arm joints num integration
                qArm = qArm + q_vel(3:9)' * obj.dt;

                % base num integration (trapez integration)
                qBase = qBase + [base_wv(1) * cos(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(1) * sin(qBase(3)+base_wv(2)*obj.dt/2);
                                 base_wv(2) ]' * obj.dt;


             if(stevc>=nShow||i==NN)
                obj.animateRobot([qArm,qBase]') ; 
                %pause(obj.dt*.05);   
                drawnow();
                stevc=0;
             end
                t=t+obj.dt;
       end
     
   % izris slik  
   % hranjenje rezultatov
%   save klasikaGregor RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar 

 %  save Potencialno1NoSecundar RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani

 
 %  save Potencialno1 RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler

 
 %  save PotencialnoTocka RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler
%   save PotencialnoTocka2 RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler
%   save PotencialnoTockaBlizu RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler
%   save PotencialnoTockaDalec RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar RecMani RecEulerR RecEuler


      fs=14;     
      plot(RecBase(:,1),RecBase(:,2),'m:')
      plot3(RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b:')
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      figure(1),plot(RecT,RecU(:,1),RecT,RecU(:,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
      
      figure(2),plot(RecT,RecU(:,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)

      
      figure(3),plot(RecBase(:,1),RecBase(:,2),'b')
      xlabel('$$x_b$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y_b$$[m]','interpreter','latex','FontSize',fs)
      
      
      figure(4),plot3(xar',yar',zar', 'g-',RecEnd(:,1),RecEnd(:,2),RecEnd(:,3),'b')
      xlabel('$$x_a$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y_a$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z_a$$[m]','interpreter','latex','FontSize',fs)
 
      
      figure,plot(RecT,RecArm)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$${\theta_i}$$[1]','interpreter','latex','FontSize',fs)
      legend('$${\theta_1}$$','$${\theta_2}$$','$${\theta_3}$$','$${\theta_4}$$','$${\theta_5}$$','$${\theta_6}$$','$${\theta_7}$$','interpreter','latex','FontSize',fs)
     
      
   

%       ea=RecEnd(:,1:3)-[xar',yar',zar'];
%       eaNorm=sqrt(ea(:,1).^2+ea(:,2).^2+ea(:,3).^2);
% %       figure(5),plot(RecT,ea)
% %       xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
% %       ylabel('$$e_{ax}$$[m], $$e_{ay}$$[m], $$e_{az}$$[m]','interpreter','latex','FontSize',fs)
%        figure,plot(RecT,eaNorm)
%        xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
%        ylabel('$$e_{dist_a}$$[m]','interpreter','latex','FontSize',fs)
%     %  title('arm tracking error')
% 
    
      eea1=sqrt(RecErrEE(:,1).^2+RecErrEE(:,2).^2+RecErrEE(:,3).^2); % pozicijski pogresek, % kotni pogresek
      eea2=sqrt(RecErrEE(:,3).^2+RecErrEE(:,4).^2+RecErrEE(:,5).^2);
      figure,plot(RecT(:),eea1(:),RecT(:),eea2(:))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{dist}$$[m], $$e_{\phi}$$[rad]','interpreter','latex','FontSize',fs)
      legend('$$e_{dist}$$','$$e_{\phi}$$','interpreter','latex','FontSize',fs)
 
    
    
    
  
      figure,plot(RecT,RecMani)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('Manipulability','interpreter','latex','FontSize',fs)

      
      figure,plot(RecT,RecEulerR,'--',RecT,RecEuler)
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\alpha_{x}$$, $$\alpha_{y}$$, $$\alpha_{z}$$ [1]','interpreter','latex','FontSize',fs)
      legend('$$\alpha_{xr}$$','$$\alpha_{yr}$$','$$\alpha_{zr}$$','$$\alpha_{x}$$','$$\alpha_{y}$$','$$\alpha_{z}$$','interpreter','latex','FontSize',fs)

 
      
end


end
end