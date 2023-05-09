
% le izrisem rezultate vodenja posnetih znotraj funkcije npr: classical.optimizerTrajectory

% save klasikaGregor RecBase RecArm RecEnd RecT RecU RecErrEE xbr ybr fibr xar yar zar 
   

%clear all, close all
close all

s=1;  % ? koliksen del trajektorije izrise


%%%%% trajektorija
 %%% load PotencialnoScenarij1m , nh=6;           %spirala, potencilno, ref poza
 %%% load PotencialnoScenarij1,   nh=6;           % potencilno, ref poza
 %%% load Potencialno1NoSecundar, nh=6; % potencialno brez pot. polja pri vodenju baze, baza ignorira ovire

%  load Potencialno1Scenarij2,  nh=6;          % potencilno, ref poza
 
 %%% load PotencialnoScenarij3, nh=8;  % brez orintacije ok
  load PotencialnoScenarij3o , nh=8; % z orintacijo ok

%  load PotencialnoScenarij4,  nh=3;    %kratka potencilno, ref poza, brez sekundarne (s potencialnim ne deluje)

 
%%%%% tocka - ali to kazat? - če je daleč vozi le bazo, ko je bližje (1m)
%%%%% doda še roko kot ptimarno
%    load PotencialnoTocka , nh=5; s=0.53;  % do cca 1/2
  %%% load PotencialnoTocka2 , nh=5; s=1;   % ni ok
%   load PotencialnoTockaBlizu   , nh=2; s=0.24;
%   load PotencialnoTockaDalec   , nh=8; s=0.89;

hFig = InitAnimation(10,nh);  % fig=10, n=7 (stevilo slikic robota)

if ~isempty(astarG)
  fig=10; astarG.redrawMap(fig,[]); % draw map on figure, add cost map CMG or put [] for none
end

%classical = Classical_optimizer();

      fs=14; 
      plot(xbr,ybr,'r'), plot3(xar,yar,zar,'b') % reference za bazo in roko
     %  plot(xbr,ybr,'r:'), plot3(xar,yar,zar,'b:') % reference za bazo in roko

    % pregledna slika  
      ind=round(linspace(1,size(RecBase,1)*s,nh));
      for i=1:nh
         qBase=RecBase(ind(i) ,:); qArm=RecArm(ind(i),:); animateRobot(hFig,[qArm,qBase]',i) ; % izris skeleta    
      end
      
     ii=1:round(size(RecBase,1)*s);
      
       plot(RecBase(ii,1),RecBase(ii,2),'r:')
       plot3(RecEnd(ii,1),RecEnd(ii,2),RecEnd(ii,3),'b:')    
      xlabel('$$x$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z$$[m]','interpreter','latex','FontSize',fs)

      axis([0 10 0 10 0 2])
       % print -depsc  ideaLayoutTop;   
       %  print -depsc  ideaLayoutSide;   
      
      
    % hitrosti baze  
      figure,plot(RecT(ii),RecU(ii,1),RecT(ii),RecU(ii,2),'--')
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$v$$[m/s], $$\omega$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$v$$','$$\omega$$','interpreter','latex','FontSize',fs)
    %hitrosti sklepov  
      figure,plot(RecT(ii),RecU(ii,3:9))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\dot{\theta_i}$$[1/s]','interpreter','latex','FontSize',fs)
      legend('$$\dot{\theta_1}$$','$$\dot{\theta_2}$$','$$\dot{\theta_3}$$','$$\dot{\theta_4}$$','$$\dot{\theta_5}$$','$$\dot{\theta_6}$$','$$\dot{\theta_7}$$','interpreter','latex','FontSize',fs)

    %koti sklepov   
      figure,plot(RecT(ii),RecArm(ii,:))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$${\theta_i}$$[1]','interpreter','latex','FontSize',fs)
      legend('$${\theta_1}$$','$${\theta_2}$$','$${\theta_3}$$','$${\theta_4}$$','$${\theta_5}$$','$${\theta_6}$$','$${\theta_7}$$','interpreter','latex','FontSize',fs)
 
      
   % pogreski baze   
   if ~isempty(xbr)
      % ee=RecBase(:,1:2)-[xbr',ybr'];
      ee=RecBase(:,1:3)-[xbr',ybr',fibr']; ee(:,3)=wrapToPi(ee(:,3));
      figure,plot(RecT(ii),ee(ii,:))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{x_b}$$[m], $$e_{y_b}$$[m],$$e_{\phi_b}$$[rad]','interpreter','latex','FontSize',fs)
      legend('$$e_{x_b}$$','$$e_{y_b}$$','$$e_{\phi_b}$$','interpreter','latex','FontSize',fs)
       
      % base possition distance error and orientation  
      eee=sqrt(ee(:,1).^2+ee(:,2).^2);
      figure(4),plot(RecT(ii),eee(ii,:),RecT(ii), ee(ii,3) )
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{dist_b}$$[m], $$e_{\phi_b}$$[rad]','interpreter','latex','FontSize',fs)
      legend('$$e_{dist_b}$$','$$e_{\phi_b}$$','interpreter','latex','FontSize',fs)
   end
   
   % pogreski roke   
      % arm possition distance error and orientation        
      figure,plot(RecT(ii),RecErrEE(ii,1:6))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{x}$$, $$e_{y}$$, $$e_{z}$$ [m], $$e_{\phi_x}$$, $$e_{\phi_y}$$, $$e_{\phi_z}$$ [rad]','interpreter','latex','FontSize',fs)
     % title('arm tracking error')
      
      eea1=sqrt(RecErrEE(:,1).^2+RecErrEE(:,2).^2+RecErrEE(:,3).^2); % pozicijski pogresek, % kotni pogresek
      eea2=sqrt(RecErrEE(:,3).^2+RecErrEE(:,4).^2+RecErrEE(:,5).^2);
      figure,plot(RecT(ii),eea1(ii),RecT(ii),eea2(ii))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$e_{dist}$$[m], $$e_{\phi}$$[rad]','interpreter','latex','FontSize',fs)
      legend('$$e_{dist}$$','$$e_{\phi}$$','interpreter','latex','FontSize',fs)

    % manipulabilnost  
      figure,plot(RecT(ii),RecMani(ii))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$m$[1]','interpreter','latex','FontSize',fs)

      
    % eulerjevi koti - ?? ni ok  
      figure,plot(RecT(ii),RecEulerR(ii,:),'--',RecT(ii),RecEuler(ii,:))
      xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
      ylabel('$$\alpha_{x}$$, $$\alpha_{y}$$, $$\alpha_{z}$$ [1]','interpreter','latex','FontSize',fs)
      legend('$$\alpha_{xr}$$','$$\alpha_{yr}$$','$$\alpha_{zr}$$','$$\alpha_{x}$$','$$\alpha_{y}$$','$$\alpha_{z}$$','interpreter','latex','FontSize',fs)
      
      
    if 0
       figure, hold on 
         plot(xbr,ybr,'r--') % reference za bazo 
        plot(RecBase(:,1),RecBase(:,2),'m:')
     
    end
      
    
    
    
    
    
    % lokalni pogled (relativno) iz platforme na roko
  RecEErel=[];
  for i=1:size(RecBase,1)  
    
    x=RecBase(i,1);
    y=RecBase(i,2);
    phi=RecBase(i,3);
   
    
    Tbase = [cos(phi), -sin(phi), 0, x ;
             sin(phi),  cos(phi), 0, y ;
             0,         0,        1, 0.833 ;
             0,         0,        0, 1 ];
    
    Pg=[RecEnd(i,:)';1];  % pozicija end effektorja v globalnem     
         
    EERelBase = Tbase^(-1)*Pg; 
    
%     T=[x;y;0.833];
%     R=[      cos(phi), -sin(phi), 0;
%              sin(phi),  cos(phi), 0;
%              0,         0,        1];
%     
%     Pg=RecEnd(i,:)' ;    
%     EERelBase=R'*(Pg -T);    
    
    RecEErel=[RecEErel;EERelBase(1:3)'];     
  end
    
 figure,plot3(RecEErel(ii,1),RecEErel(ii,2),RecEErel(ii,3))   
      xlabel('$$x_{r}$$[m]','interpreter','latex','FontSize',fs)
      ylabel('$$y_{r}$$[m]','interpreter','latex','FontSize',fs)
      zlabel('$$z_{r}$$[m]','interpreter','latex','FontSize',fs)
   grid 
%  figure,plot(RecT(ii),RecEErel(ii,:))   
%       xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
%       ylabel('$$x$$, $$y$$, $$z$$ [m]','interpreter','latex','FontSize',fs)
%       legend('$$x$$','$$y$$','$$z$$','interpreter','latex','FontSize',fs)
 
%  dist=sqrt((RecEnd(:,1)-RecBase(:,1)).^2+(RecEnd(:,2)-RecBase(:,2)).^2+(RecEnd(:,3)-0.833).^2);     
%       
%  figure,plot(RecT(ii),sqrt( RecEErel(ii,1).^2+RecEErel(ii,2).^2+RecEErel(ii,3).^2), RecT(ii),dist,'.' )   
%       xlabel('$$t$$[s]','interpreter','latex','FontSize',fs)
%       ylabel('$$d$$ [m]','interpreter','latex','FontSize',fs)
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% dodatne funkcije le za ta skript  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==============================
function obj = InitAnimation(fig,n)
    % za animacijo
     obj.hMainFig=figure(fig); clf; 
     hold on;axis equal ,grid
   %  axis([-1  1 -1 1 0 1]);
     xlabel('x (m)','FontSize', 8);ylabel('y (m)','FontSize', 8);
    %get(gca,'view')
     set(gca,'view',[-41.4380   34.7032]);  
     %get(gcf,'Position')
     set(gcf,'Position',[48.2000  316.2000  560.0000  420.0000]);
     
     for i=1:n
         obj.hArm(i)=plot3(nan,nan,nan,'b','LineWidth',2) ;     % handle na roko 
         obj.hRob(i)=plot3(nan,nan,nan,'r','LineWidth',2) ;     % handle na platformo 
         obj.hEnd(i)=plot3(nan,nan,nan,'g','LineWidth',2) ;     % handle na prijemalo 
     end
end
        
%=============================================================    
function animateRobot(obj, qBegin,i)
% ----------------------------------------------------
% animation
% qBegin - [q1 ... q7 x y phi]
% ----------------------------------------------------
    
    qa = qBegin(1:7);  
    qb = qBegin(8:10); % [x y phi]
    
   % [Abase, A01, A12, A23, A34, A45, A56, A67, rT] = obj.robot.GeometricRobot(qa, qb);
    [Abase, A01, A12, A23, A34, A45, A56, A67, rT] = GeometricRobot(qa, qb);

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
                
    egh=[eg; ones(1,size(eg,2))]; % homogene koordinate, dodam enice v zafnji stolpec
    E=(A*egh)'; %tocke prijemala za izris
    
    set(obj.hRob(i),'XData',R(:,1),'YData',R(:,2),'ZData',R(:,3))   % izris robota
    set(obj.hArm(i),'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3))   % izris roke
    set(obj.hEnd(i),'XData',E(:,1),'YData',E(:,2),'ZData',E(:,3))   % izris roke
  %  drawnow;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
function [Abase, A01, A12, A23, A34, A45, A56, A67, T] = GeometricRobot(qArm, qBase)


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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [A] = getTransformationA(a,d,alpha,fi)

        % getTransformationA - get one step of direct kinematic model
        % --------------------------------------------------------------
        % creates transformation between two Coordinate systems, 
        % based on DH parameters
        
        A = [ cos(fi) -sin(fi)*cos(alpha) sin(fi)*sin(alpha) a*cos(fi) ;
              sin(fi) cos(fi)*cos(alpha) -cos(fi)*sin(alpha) a*sin(fi) ;
              0       sin(alpha)          cos(alpha)         d ;
              0       0                   0                  1];
        
                    
   end 
