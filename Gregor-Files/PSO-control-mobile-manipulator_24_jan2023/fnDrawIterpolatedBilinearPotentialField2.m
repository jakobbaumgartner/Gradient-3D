function fnDrawIterpolatedBilinearPotentialField2(space,resolution,FF,FFmax,fig,flag)

    %delta=1/resolution/10; % How fine to draw
    delta=1/resolution/5-eps;
 
    % delta=1/resolution-eps;
   
    
    
 %%%   dd=1/resolution+1/resolution/2+.01;
%    dd=1/resolution+1/resolution/20*1;   % izrise do roba ?? preveri kdaj vrne velike vrednosti na robu okolice ???  
     
    dd=1/resolution/20*1;   % izrise do roba ?? preveri kdaj vrne velike vrednosti na robu okolice ???  
   
    x=space(1)+dd:delta:(space(2)-dd);
    y=space(3)+dd:delta:(space(4)-dd);

    ZZ=zeros(length(y),length(x));
    Gx=[];
    Gy=[];
    
    for i=1:length(x)
        for j=1:length(y)
               
       z=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(x(i),y(j),FF,FFmax,space,resolution);
              
        ZZ(j,i)=z(1);
        
        d=0.05;
        s=z(2:3)/sqrt(z(2)^2+z(3)^2);
        Gx=[Gx, [x(i); x(i)-s(1)*d]];
        Gy=[Gy, [y(j); y(j)-s(2)*d]];
       
        end
    end
    
  if flag==2
   
    figure(fig), 
   % surfc(x,y,ZZ,'EdgeColor','None')
   surf(x,y,ZZ,'EdgeColor','None')
 end  
   
 if flag==0
   
    figure(fig), 
   % surfc(x,y,ZZ,'EdgeColor','None')
  %  surf(x,y,ZZ)
  h=surfc(x,y,ZZ,'EdgeColor','None');
    h(2).LevelStep=0.4;
 end  
if flag==1
  figure(fig),hold on, plot(Gx(1,:),Gy(1,:),'.r',Gx,Gy,'r') 
end

end