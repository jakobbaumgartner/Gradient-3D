 function J=jacobianNumeric(q)
            % ----------------------------------------------------
            % animation get jacobian numerically for given configuration q
            % q - [ x y phi q1 ... q7]
            % J - jacobian for end effector velocities
            % ----------------------------------------------------
            opcija = 2; % izbira izračuna napake kotov
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
            
            [~, ~, ~, ~, ~, ~, ~, ~, T1] = GeometricRobot(qq1(4:10), qq1(1:3));
            [~, ~, ~, ~, ~, ~, ~, ~, T2] = GeometricRobot(qq2(4:10), qq2(1:3));
            
            % parcialni odvodi za pozicije
            J(1:3,i)= ( T2(1:3,4)-T1(1:3,4) )/ (qq2(i)-qq1(i));
       
            % parcialni odvodi za kote
            R1=T1(1:3,1:3);
            R2=T2(1:3,1:3);
            dR = R2*R1';  % R2=dR*R1 , od orientacije 1 proti orientaciji 2 (je kot referenca)
    
            dR = dR / norm(dR);

           if opcija==1     
                eq = 2*log(quaternion(rotm2quat(dR))); % kvaternion pogreška
                [~, qB, qC, qD] = eq.parts; 
                eR = [qB, qC, qD]';  
           elseif opcija==2      
                % 2. opcija za izracun kotov (AMS knjiga slo, str 165)
                OmegaDT=-(dR-eye(3));
                eR = [OmegaDT(2,3); OmegaDT(3,1); OmegaDT(1,2)];
           end     
                
            J(4:6,i)= eR / (qq2(i)-qq1(i));

         end                   
    end