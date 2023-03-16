
function rrr=tmp_GetInterpolatedTriLinearCostToGoal(xx,yy,zz,FF,maxFF,space,resolution)

   x_min=space(1);x_max=space(2); y_min=space(3);y_max=space(4); z_min=space(5);z_max=space(6); 
   sizeX=(x_max-x_min)*resolution; sizeY=(y_max-y_min)*resolution; sizeZ=(z_max-z_min)*resolution; 
   
   eC=1/resolution;  % predpostavim iszo velikost stranic voksla v x,y in z smeri
        
   % dobim pointer na matriko okolja 
   ix = round( (xx-x_min)/(x_max-x_min)*(sizeX-1*0)+0.5*1);
   iy = round( (yy-y_min)/(y_max-y_min)*(sizeY-1*0)+0.5*1);
   iz = round( (zz-z_min)/(z_max-z_min)*(sizeZ-1*0)+0.5*1);
  
   if( ix<1||ix>sizeX||iy<1||iy>sizeY ||iz<1||iz>sizeZ ) % ce je tocka izven okolja  
      rrr=[nan,0,0]; 
      disp('tocka ni v okolju');
      return; 
   end;   

   %  max tock oz. ovir ne gledaš
   if (FF(ix,iy,iz)>=maxFF), 
       rrr=[nan,0,0]; 
       disp('tocka je v zasedenm vokslu??');
       return; 
   end;  

   rrr=getInterpolatedPotential(xx,yy,zz,ix,iy,iz,FF,maxFF,eC,x_min,x_max,y_min,y_max,z_min,z_max,sizeX,sizeY,sizeZ);   
  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% sub functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function rrr=getInterpolatedPotential(xx,yy,zz,ix,iy,iz,FF,maxFF,eC,x_min,x_max,y_min,y_max,z_min,z_max,sizeX,sizeY,sizeZ);   

   xs=(ix-1)*eC+ eC/2 +x_min; ys=(iy-1)*eC+ eC/2+y_min; zs=(iz-1)*eC+ eC/2+z_min;  % sredisce celice
   
   % dolocitev 8ih okoliskih vokslov - indekse centrov celic
%    if xx <= xs && yy<=ys && zz<=zs% spodnji leva cetrtina celice       
%         x0= ix-1; x1=ix; y0=iy-1;  y1=iy; z0=iz-1;  z1=iz;  
%         xx0=xs-eC; xx1=xs;    yy0=ys-eC;  yy1=ys;  zz0=zs-eC;  zz1=zs; 
%    elseif xx > xs && yy<=ys && zz<=zs % spodnji desna cetrtina celice
%         x0=ix; x1=ix+1; y0=iy-1;  y1=iy;  z0=iz-1;  z1=iz;    
%         xx0=xs;    xx1=xs+eC; yy0=ys-eC;  yy1=ys; zz0=zs-eC;  zz1=zs;
%    elseif xx > xs && yy>ys && zz<=zs % zgornja desna cetrtina celice
%         x0=ix; x1=ix+1; y0=iy;  y1=iy+1;  z0=iz-1;  z1=iz;  
%         xx0=xs;    xx1=xs+eC; yy0=ys;     yy1=ys+eC; zz0=zs-eC;  zz1=zs;
%    elseif xx <= xs && yy>ys && zz<=zs % zgornja leva cetrtina celice
%         x0=ix-1; x1=ix; y0=iy;  y1=iy+1;  z0=iz-1;  z1=iz;  
%         xx0=xs-eC; xx1=xs;    yy0=ys;     yy1=ys+eC; zz0=zs-eC;  zz1=zs;
%     
%    elseif xx <= xs && yy<=ys && zz>zs% spodnji leva cetrtina celice       
%         x0= ix-1; x1=ix; y0=iy-1;  y1=iy; z0=iz-1;  z1=iz;  
%         xx0=xs-eC; xx1=xs;    yy0=ys-eC;  yy1=ys;  zz0=zs;  zz1=zs+eC; 
%    elseif xx > xs && yy<=ys && zz>zs % spodnji desna cetrtina celice
%         x0=ix; x1=ix+1; y0=iy-1;  y1=iy;  z0=iz-1;  z1=iz;    
%         xx0=xs;    xx1=xs+eC; yy0=ys-eC;  yy1=ys; zz0=zs;  zz1=zs+eC;
%    elseif xx > xs && yy>ys && zz>zs % zgornja desna cetrtina celice
%         x0=ix; x1=ix+1; y0=iy;  y1=iy+1;  z0=iz-1;  z1=iz;  
%         xx0=xs;    xx1=xs+eC; yy0=ys;     yy1=ys+eC; zz0=zs;  zz1=zs+eC;
%    elseif xx <= xs && yy>ys && zz>zs % zgornja leva cetrtina celice
%         x0=ix-1; x1=ix; y0=iy;  y1=iy+1;  z0=iz-1;  z1=iz;  
%         xx0=xs-eC; xx1=xs;    yy0=ys;     yy1=ys+eC; zz0=zs;  zz1=zs+eC;
%    end

    if xx <= xs
        x0= ix-1; x1=ix;   xx0=xs-eC; xx1=xs;    % x0, x1 x-indeks voxlov za interpolaijo , [xx0,yy0] središ?ana koordinata
    else
        x0=ix;    x1=ix+1; xx0=xs;    xx1=xs+eC;
    end
    
    if yy <= ys
        y0=iy-1;  y1=iy;   yy0=ys-eC;  yy1=ys;
    else
        y0=iy;    y1=iy+1; yy0=ys;     yy1=ys+eC;
    end

    if zz <= zs
        z0=iz-1;  z1=iz;   zz0=zs-eC;  zz1=zs;
    else
        z0=iz;    z1=iz+1; zz0=zs;     zz1=zs+eC;
    end


   % normirane koordinate: v x in y daj le odstopanja 0-ecell in normiraj, da tece od 0 do 1  
   x=(xx-xx0)/eC;  y=(yy-yy0)/eC;  z=(zz-zz0)/eC;
     

    % za tocke na robu popravim tocke za interpolacijo (premaknem jih notri, ce je katera izven okolja) 
  % Loc=0;  % location state 0 - znotraj okoja, 1=levo izven, 2=dol izven, 12=levo-dol,...  
   if(ix>1&&ix<sizeX && iy>1&&iy<sizeY && iz>1&&iz<sizeY)~=1 % ce sem na robu okolja (ali izven - to resi ze vrstica 14)
       
       disp('tocka je v robnem voxlu, kar za enkrat ne obravnavam')
       
       rrr=[nan,0,0]; 
       return; 
     
   end;   % kakšen gradient dat tu - oceni ga iz sosednjih celic
   
  
   
   if 0  % izpeljava
      %Trilinearna interpolacija (wikipwdia in http://paulbourke.net/miscellaneous/interpolation/)

      %   syms xx yy zz xx0 yy0 zz0 dc f000 f001 f010 f011 f100 f101 f110 f111 
      %   x=(xx-xx0)/dc; y=(yy-yy0)/dc; z=(zz-zz0)/dc; % normirane koordinate
       
       syms x y z dc f000 f001 f010 f011 f100 f101 f110 f111 
       %    x=(xx-xx0)/dc; y=(yy-yy0)/dc; z=(zz-zz0)/dc; % normirane koordinate
     
       
       f00=f000*(1-x)+f100*x;  % interpoliras po x
       f01=f001*(1-x)+f101*x;
       f10=f010*(1-x)+f110*x;
       f11=f011*(1-x)+f111*x;
      
       f0=f00*(1-y)+f10*y;   % interpoliras po y
       f1=f01*(1-y)+f11*y;
       
       f= f0*(1-z)+f1*z  % interpoliras po z
       
       pretty(simplify(f))
       
       
       % se poenostavi v (http://paulbourke.net/miscellaneous/interpolation/)
       syms x y z dc f000 f001 f010 f011 f100 f101 f110 f111 
       f=f000*(1-x)*(1-y)*(1-z)+f100*x*(1-y)*(1-z)+f010*(1-x)*y*(1-z)+f001*(1-x)*(1-y)*z+...
         f101*x*(1-y)*z + f011*(1-x)*y*z + f110*x*y*(1-z)+ f111*x*y*z
         
       gradX=diff(f,'x')
       gradY=diff(f,'y')
       gradZ=diff(f,'z')
   end

    % ???? tu se dodaj detekcijo ovir in prostora (da ne gres cetz mejo okolja)
    f000=sfGetCorrectedPotentialForObstacles(x0,y0,z0,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f100=sfGetCorrectedPotentialForObstacles(x1,y0,z0,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f010=sfGetCorrectedPotentialForObstacles(x0,y1,z0,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f001=sfGetCorrectedPotentialForObstacles(x0,y0,z1,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f101=sfGetCorrectedPotentialForObstacles(x1,y0,z1,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f011=sfGetCorrectedPotentialForObstacles(x0,y1,z1,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f110=sfGetCorrectedPotentialForObstacles(x1,y1,z0,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    f111=sfGetCorrectedPotentialForObstacles(x1,y1,z1,maxFF,sizeX,sizeY,sizeZ,eC,FF);
    
  
   w000=(1-x)*(1-y)*(1-z);
   w100=x*(1-y)*(1-z);
   w010=(1-x)*y*(1-z);
   w001=(1-x)*(1-y)*z;
   w101=x*(1-y)*z;
   w011=(1-x)*y*z;
   w110=x*y*(1-z);
   w111=x*y*z;


   
   % interpoliran cost za pozicijo  
   cost= [w000 w100 w010 w001 w101 w011 w110 w111]*[f000 f100 f010 f001 f101 f011 f110 f111]';

   % TODO : dodat še interpolacijo gradientov podobno kot za 2D (gradientje nezvezen - dodatna interpolacija)
   gradX = (f001*z*(y - 1) + f010*y*(z - 1) - f101*z*(y - 1) - f110*y*(z - 1) - f000*(y - 1)*(z - 1) + f100*(y - 1)*(z - 1) - f011*y*z + f111*y*z)/eC;   % normirane in utezi w
   gradY = (f001*z*(x - 1) - f011*z*(x - 1) + f100*x*(z - 1) - f110*x*(z - 1) - f000*(x - 1)*(z - 1) + f010*(x - 1)*(z - 1) - f101*x*z + f111*x*z)/eC;
   gradZ = (f010*y*(x - 1) - f011*y*(x - 1) + f100*x*(y - 1) - f101*x*(y - 1) - f000*(x - 1)*(y - 1) + f001*(x - 1)*(y - 1) - f110*x*y + f111*x*y)/eC;
   
 
  
   rrr=[cost,gradX,gradY,gradZ];   
  

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




function pot=sfGetCorrectedPotentialForObstacles(xi,yj,zk,maxFF,sizeX,sizeY,sizeZ,ecell,FF)

 pot=[];
 
 % robne to?ke se lahko enostavno reši tako, da je v podanemu okolju (matrika vokslov) za dva
 % voxla ve?ja v vse smeri kot je potrebno (1-ve?je je dovolj za interpolacijo potenciala, 2-ve?je potrebno za interpolacijo gradienta)
 
 
%    if( xi<=1 || xi>=sizeX || yj<=1 || yj>=sizeY )   % sem na robu okolja
% 
%        if(FF(xi,yj)>maxFF) % ovira na robu (oz. ena od stirih tock za interpolacijo je ovira na robu)
%           
%                   if(xi<=1)
%                       if(yj>=sizeY)
%                           ii=[FF(1,yj-1) ,FF(2,sizeY-1),FF(2,sizeY)];
%                       elseif(yj<=1)
%                           ii=[FF(1+1,1),FF(2,2), FF(xi,2)];
%                       else
%                           ii=[FF(1,yj-1) ,FF(2,yj-1),FF(2,yj),FF(2,yj+1), FF(1,yj+1)];
%                       end
%                   elseif(xi>=sizeX)   
%                       if(yj>=sizeY)
%                           ii=[FF(sizeX,sizeY-1) ,FF(sizeX-1,sizeY-1),FF(sizeX-1,sizeY)];
%                       elseif(yj<=1)
%                           ii=[FF(sizeX-1,1),FF(sizeX-1,2), FF(sizeX,2)];
%                       else
%                           ii=[FF(sizeX,yj-1) ,FF(sizeX-1,yj-1),FF(sizeX-1,yj),FF(sizeX-1,yj+1), FF(sizeX,yj+1)];
%                       end
% 
%                   elseif(yj>=sizeY)   
%                       ii=[FF(xi-1,sizeY),FF(xi-1,sizeY-1),FF(xi,sizeY-1),FF(xi+1,sizeY-1) ,FF(xi+1,sizeY)];
% 
%                   elseif(yj<=1)   
%                       ii=[FF(xi-1,1),FF(xi-1,2),FF(xi,2),FF(xi+1,2), FF(xi+1,1)];
%                   else
%                       ii=[FF(xi-1,yj+1),FF(xi-1,yj),FF(xi-1,yj-1),  FF(xi,yj-1) ,FF(xi+1,yj-1),FF(xi+1,yj),FF(xi+1,yj+1), FF(xi,yj+1)];
%                       disp('strange - check function tmp_GetInterpolatedBiLinearCostToGoal2_Test.m')
%                   end
% 
%                   [pot]=max(ii(ii<maxFF));   % ??? to na slepo povecanje ni ok, potrebno je pogledati gradient
% 
%                    %pot=pot*1.10; 
%                    pot=pot+ecell*sqrt(2); 
%       
%        else 
%           pot=FF(xi,yj); 
%        end  
%        
%        return;
%    end

      
      
% TODO: voxel za interpolacijo je ovira (ima neskon?en potencial) -> podobno kot za 2D potrebno rešit tudi za 3D

%     if(FF(xi,yj,zk)>maxFF && 1) % ce je celica ovira
%        % vzamem malo vec kot je maks okoliskih, ki niso na maxFF 
%        ii=[FF(xi-1,yj+1),FF(xi-1,yj),FF(xi-1,yj-1),  FF(xi,yj-1) ,FF(xi+1,yj-1),FF(xi+1,yj),FF(xi+1,yj+1), FF(xi,yj+1)];
%        [pot]=max(ii(ii<maxFF));   % ??? to na slepo povecanje ni ok, potrebno je pogledati gradient
%       
%        pot=pot+ecell*sqrt(3); 
%        
%        if isempty(pot),  % ce se slucajno zgodi, da so vse sosedne celice na oviri
%            pot=maxFF*1.1;  
%        end  
%        
%        % pot=maxFF*1.1;
%     else
%        pot=FF(xi,yj,zk); 
%     end

     pot=FF(xi,yj,zk); 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


