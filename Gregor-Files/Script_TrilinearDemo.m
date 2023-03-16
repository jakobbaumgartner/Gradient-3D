

FF=zeros(4,4,4) % matrika vokslov okolja FF(i,j,k), i,j,k so indeksi na x,y,z koordinato voksla, vrednost voksla pa je potencial (npr. razdalja do cilja dobljen z Dijkstro )
space=[0 1 0 1 0 1]; % min max vrednosti
resolution=4; %stevilo celic na meter

eC=1/resolution; %dolzina stranice

for i=1:4  % simpl potencial razdalja do koordinatnega sredisca
 for j=1:4  
  for k=1:4  
    FF(i,j,k)=eC*sqrt((i-.5)^2+(j-.5)^2+(k-.5)^2);
  %  FF(i,j,k)=eC*sqrt((4-i+.5)^2+(4-j+.5)^2+(4-k+.5)^2);
  end
 end
end
FF


%FF(3,2,2)=100; % dodam eno oviro


%FFmax=max(FF(FF~=max(max(FF))))+1; % seccond highest value +1 (the highest value is obstacle)
 
FFmax=max(max(FF))+1;

xx=0.6, yy=0.6, zz=0.49

xx=1.499*eC, yy=2.5*eC, zz=1.5*eC

xx=2.9*eC, yy=2.5*eC, zz=1.5*eC

xx=1.5*eC, yy=1.5*eC, zz=1.5*eC


PotInGrad=tmp_GetInterpolatedTriLinearCostToGoal(xx,yy,zz,FF,FFmax,space,resolution);
Potencial=PotInGrad(1)





if 0
    
   x_min=space(1);x_max=space(2); y_min=space(3);y_max=space(4); z_min=space(5);z_max=space(6); 
   sizeX=(x_max-x_min)*resolution; sizeY=(y_max-y_min)*resolution; sizeZ=(z_max-z_min)*resolution; 
   
   
   % dobim pointer na matriko okolja 
   ix = round( (xx-x_min)/(x_max-x_min)*(sizeX-1*0)+0.5*1);
   iy = round( (yy-y_min)/(y_max-y_min)*(sizeY-1*0)+0.5*1);
   iz = round( (zz-z_min)/(z_max-z_min)*(sizeZ-1*0)+0.5*1);
  
   FF(ix,iy,iz) 
    
   FF(ix,iy,1) 
   
end



%Kako to izrisat?
% 
% surf(x,y,ZZ,'EdgeColor','None')
% 
% x=[((1:4)-.5)*eC;((1:4)-.5)*eC;((1:4)-.5)*eC;((1:4)-.5)*eC]
% y=[((1:4)-.5)*eC;((1:4)-.5)*eC;((1:4)-.5)*eC;((1:4)-.5)*eC]
% z=[((1:4)-.5)*eC;((1:4)-.5)*eC;((1:4)-.5)*eC;((1:4)-.5)*eC]
% scatter3(x,y,z,15,'filled'); 
% 
% 
% 
% 
% x = FF(:,:,:);
%  y = FF(:,:,:);
%  z = FF(:,:,:);
% [x, y, z] = meshgrid(FF(:,1), FF(:,2), FF(:,3));
% surface(x,y,z);
%  figure; 
% scatter3(x,y,z,'.-'); 
% figure;
% shading flag
% 


% x = [0.25:.25:1];
% y = [0.25:.25:1];
% z = [0.25:.25:1];
% A = cat(3,x,y,z);
% 
% x = A(:,:,2)


