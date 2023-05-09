
function rrr=tmp_GetInterpolatedBiLinearCostToGoal2AndGrad_Test(xx,yy,FF,maxFF,space,resolution)

   x_min=space(1);x_max=space(2); y_min=space(3);y_max=space(4);  
   sizeX=(x_max-x_min)*resolution;sizeY=(y_max-y_min)*resolution;  
   
   eC=1/resolution;
        
   % dobim pointer na matriko okolja 
   ix = round( (xx-x_min)/(x_max-x_min)*(sizeX-1*0)+0.5*1);
   iy = round( (yy-y_min)/(y_max-y_min)*(sizeY-1*0)+0.5*1);
   
   if( ix<1||ix>sizeX||iy<1||iy>sizeY ) % ce je tocka izven okolja  
      rrr=[nan,0,0]; 
       return; 
   end;   

   %  max tock oz. ovir ne gledaš
   if (FF(ix,iy)>=maxFF), rrr=[nan,0,0]; return; end;  

   
   rrr=getInterpolatedPotential(xx,yy,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
   
   
   
   flagInterpolatedGradient=1;
   if(flagInterpolatedGradient)
   
      if( ix<2||ix>(sizeX-1)||iy<2||iy>(sizeY-1) ) % ce preblizu roba, potem vrne kar neinterpoliran gradient v rrr 
      %  return;   % ?? to lahko odstraniš
      end;   
      
      % doloci gradiente centrov celic za vse 4 celice
      
      
       % dolocitev 4ih okoliskih tock - indekse centrov celic
       xs=(ix-1)*eC+ eC/2 +x_min; ys=(iy-1)*eC+ eC/2+y_min;% sredisce celice
       if xx <= xs && yy<=ys % spodnji leva cetrtina celice       
            x0= ix-1; x1=ix; y0=iy-1;  y1=iy;  xx0=xs-eC; xx1=xs;       yy0=ys-eC;  yy1=ys; 
       elseif xx > xs && yy<=ys % spodnji desna cetrtina celice
            x0=ix; x1=ix+1; y0=iy-1;  y1=iy;   xx0=xs;       xx1=xs+eC; yy0=ys-eC;  yy1=ys; 
       elseif xx > xs && yy>ys % zgornja desna cetrtina celice
            x0=ix; x1=ix+1; y0=iy;  y1=iy+1;   xx0=xs;       xx1=xs+eC; yy0=ys;        yy1=ys+eC; 
       elseif xx <= xs && yy>ys % zgornja leva cetrtina celice
            x0=ix-1; x1=ix; y0=iy;  y1=iy+1;   xx0=xs-eC; xx1=xs;       yy0=ys;        yy1=ys+eC; 
       end
      
       
   %    [g00,g01,g10,g11]=getCellCenterGradients2(xx0,yy0,xx1,yy1,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);
       
       %[g00,g01,g10,g11]=getCellsCenterGradients(x0,y0,x1,y1, xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
       
       % doloci gradiente za 4 okoliske celice
       ax=x0; ay=y0; 
       g00=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
       ax=x0; ay=y1;
       g01=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
       ax=x1; ay=y0;
       g10=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
       ax=x1; ay=y1;
       g11=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);

       
       
       
       
       % normirane koordinate: v x in y daj le odstopanja 0-ecell in normiraj, da tece od 0 do 1  
       x=(xx-xx0)/eC;
       y=(yy-yy0)/eC;
       
       w00=(1-x)*(1-y);
       w01=(1-x)*y;
       w10=x*(1-y);
       w11=x*y;

   
       % interpoliran gradient za pozicijo  
       gx=w00*g00(1) + w01*g01(1) + w10*g10(1) + w11*g11(1);
       gy=w00*g00(2) + w01*g01(2) + w10*g10(2) + w11*g11(2);

       rrr=[rrr(1),gx,gy];  %Interpoliran cost in interpoliran gradient 

       
   end
   
   
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% sub functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function [g00,g01,g10,g11]=getCellsCenterGradients(x0,y0,x1,y1,  xx,yy,rrr  ,FF,sizeX,sizeY,x_min,y_min,eC,maxFF)
% 
%   % dodaj da ce levo in desno ovira je gx=0 podobno za gor dol ib gy
% 
%        ax=x0; ay=y0; 
%        g00=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
%        ax=x0; ay=y1;
%        g01=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
%        ax=x1; ay=y0;
%        g10=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
%        ax=x1; ay=y1;
%        g11=getCellGrad(ax,ay,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF);
%        
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [g]=getCellGrad(ax_,ay_,xx,yy,rrr, FF,sizeX,sizeY,x_min,y_min,eC,maxFF)
 
   %popravi še na robu, kjer vzami za ceno centra celic kar popravljeno z gradientom od pozicije

   % !! bolj enostavno in bolje bi bilo, da bi bilo dejansko okolje vedno
   % za 2 celici ve?je kot izrezano okolje oz. bi bilo okoli dve celice
   % ovir, ?e bi šlo za maks velikost okolja...
   
   ax=ax_; ay=ay_;
 
   % ce je ax<1 pomeni ali ax>sizeX pomeni da smo izven roba, 
   % tu poenostavim in se le premaknemo na rob in dolocimo gradient za tocko na robu   
   if ax<1
       ax=1;
   elseif ax>sizeX
       ax=sizeX;
   end
   if ay<1
       ay=1;
   elseif ay>sizeY
       ay=sizeY;
   end
      
   xs=(ax-1)*eC+ eC/2 +x_min; ys=(ay-1)*eC+ eC/2+y_min;% sredisce celice, ki ji dolocam gradient
      
   
       if(ax>1&&ax<sizeX) 
           fx=[FF(ax-1,ay),FF(ax,ay),FF(ax+1,ay)];
       elseif(ax<2) % ce je center celice ax<=1 -> sem na robu za gradient odcitam celico levo in celico desno 
           grad=rrr(2:3);  
           vect=[xs-eC;ys] -[xx;yy];
           fx_=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
           
%            if(ax<1)
%              vect=[xs-2*eC;ys] -[xx;yy];
%              fx__=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
%              fx=[fx__,fx_,FF(ax+1,ay)]; 
%            else
             fx=[fx_,FF(ax,ay),FF(ax+1,ay)];  % rekonstruiram manjkajocega, ker je mozno da se pelje tudi izven ce ni ograje na robu 
%           end
       else  
           grad=rrr(2:3);  
           vect=[xs+eC;ys] -[xx;yy];
           fx_=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru

%            if(ax>(sizeX+1))
%               vect=[xs+2*eC;ys] -[xx;yy];
%               fx__=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
%  
%               fx=[FF(ax-1,ay),fx_,fx__];  
%            else
              fx=[FF(ax-1,ay),FF(ax,ay),fx_]; 
%           end
       end
       
      if fx(2)>maxFF
        grad=rrr(2:3);  
        vect=[xs;ys] -[xx;yy];
        fx(2)=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
        
        % celico na drugi strani ovire povecam, da ni izbrana (lahko je blizje cilja)
        if(xx<xs) % sem pod oviro
          fx(3)=fx(2)+eC;      %   fy(1)=fy(2)+eC;  % povecam, da ju ne izberem
        else  % sem nad oviro
          fx(1)=fx(2)+eC;
        end

      end
       
       
       [ff,ii]=min(fx);
       gx=(ff-fx(2))*(ii-2)/eC; 

       



       %
       if(ay>1&&ay<sizeY)
           fy=[FF(ax,ay-1),FF(ax,ay),FF(ax,ay+1)];
       elseif(ay<2)
           
           grad=rrr(2:3);  
           vect=[xs;ys-eC] -[xx;yy];
           fy_=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
           
%            if(ay<1)
%               vect=[xs;ys-2*eC] -[xx;yy];
%               fy__=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
%  
%               fy=[fy__,fy_,FF(ax,ay+1)];
%            else
              fy=[fy_,FF(ax,ay),FF(ax,ay+1)];  
%           end
           
       else  
           
           grad=rrr(2:3);  
           vect=[xs;ys+eC] -[xx;yy];
           fy_=rrr(1) + grad*vect;   % popravek costa za celico, ki ji dolocam gradient v centru
%            if(ay>(sizeY+1))
%              vect=[xs;ys+2*eC] -[xx;yy];
%              fy__=rrr(1) + grad*vect;  
%              fy=[FF(ax,ay-1),fy_,fy__]; 
%            else
             fy=[FF(ax,ay-1),FF(ax,ay),fy_];  
%           end
       end
       
       if fy(2)>maxFF 
        grad=rrr(2:3);
        vect=[xs;ys] -[xx;yy];
        fy(2)=rrr(1) + grad*vect;  % popravek costa
        
        % celico na drugi strani ovire povecam, da ni izbrana(lahko je blizje cilja)
        if(yy<ys) % sem pod oviro
          fy(3)=fy(2)+eC;      %   fy(1)=fy(2)+eC;  % povecam, da jo ne izberem
        else  % sem nad oviro
          fy(1)=fy(2)+eC;
        end
       end

       
       [ff,ii]=min(fy);
       gy=(ff-fy(2))*(ii-2)/eC;
       
       
       g=[gx,gy];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 
% function [g00,g01,g10,g11]=getCellCenterGradients2(xx0,yy0,xx1,yy1,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY)
% 
%        d=eC/20; 
% 
%        rrr00a=getInterpolatedPotential(xx0-d,yy0-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr00b=getInterpolatedPotential(xx0+d,yy0-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr00c=getInterpolatedPotential(xx0+d,yy0+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr00d=getInterpolatedPotential(xx0-d,yy0+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        g00=(rrr00a(2:3)+rrr00b(2:3)+rrr00c(2:3)+rrr00d(2:3))/4;  %gradient centra celice
%        
%        rrr01a=getInterpolatedPotential(xx0-d,yy1-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr01b=getInterpolatedPotential(xx0+d,yy1-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr01c=getInterpolatedPotential(xx0+d,yy1+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr01d=getInterpolatedPotential(xx0-d,yy1+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        g01=(rrr01a(2:3)+rrr01b(2:3)+rrr01c(2:3)+rrr01d(2:3))/4; 
%        
%        rrr10a=getInterpolatedPotential(xx1-d,yy0-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr10b=getInterpolatedPotential(xx1+d,yy0-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr10c=getInterpolatedPotential(xx1+d,yy0+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr10d=getInterpolatedPotential(xx1-d,yy0+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        g10=(rrr10a(2:3)+rrr10b(2:3)+rrr01c(2:3)+rrr10d(2:3))/4; 
% 
%        rrr11a=getInterpolatedPotential(xx1-d,yy1-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr11b=getInterpolatedPotential(xx1+d,yy1-d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr11c=getInterpolatedPotential(xx1+d,yy1+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        rrr11d=getInterpolatedPotential(xx1-d,yy1+d,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY);   
%        g11=(rrr11a(2:3)+rrr11b(2:3)+rrr11c(2:3)+rrr11d(2:3))/4; 
% 
% 
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function rrr=getInterpolatedPotential(xx,yy,ix,iy,FF,maxFF,eC,x_min,x_max,y_min,y_max,sizeX,sizeY)

   xs=(ix-1)*eC+ eC/2 +x_min; ys=(iy-1)*eC+ eC/2+y_min;% sredisce celice
   
   % dolocitev 4ih okoliskih tock - indekse centrov celic
   if xx <= xs && yy<=ys % spodnji leva cetrtina celice       
        x0= ix-1; x1=ix; y0=iy-1;  y1=iy;  xx0=xs-eC; xx1=xs;       yy0=ys-eC;  yy1=ys; 
   elseif xx > xs && yy<=ys % spodnji desna cetrtina celice
        x0=ix; x1=ix+1; y0=iy-1;  y1=iy;   xx0=xs;       xx1=xs+eC; yy0=ys-eC;  yy1=ys; 
   elseif xx > xs && yy>ys % zgornja desna cetrtina celice
        x0=ix; x1=ix+1; y0=iy;  y1=iy+1;   xx0=xs;       xx1=xs+eC; yy0=ys;        yy1=ys+eC; 
   elseif xx <= xs && yy>ys % zgornja leva cetrtina celice
        x0=ix-1; x1=ix; y0=iy;  y1=iy+1;   xx0=xs-eC; xx1=xs;       yy0=ys;        yy1=ys+eC; 
   end

   % normirane koordinate: v x in y daj le odstopanja 0-ecell in normiraj, da tece od 0 do 1  
   x=(xx-xx0)/eC;
   y=(yy-yy0)/eC;
       

    % za tocke na robu popravim tocke za interpolacijo (premaknem jih notri, ce je katera izven okolja) 
   Loc=0;  % location state 0 - znotraj okoja, 1=levo izven, 2=dol izven, 12=levo-dol,...  
   if(ix>1&&ix<sizeX && iy>1&&iy<sizeY)~=1 % ce sem na robu okolja (ali izven - to resi ze vrstica 14)
            
       % Loc=0 (znotraj), =1000(levo), =1010 (levo dol), =1001(levo gor),..
       Loc= (x0<1)*1000  + (x1>sizeX)*100 + (y0<1)*10 + (y1>sizeY);
       xxNov=xx; yyNov=yy; %shrani prvotne

      % move=eC*.251;
       move=eC*.51;
       
        switch Loc
           case 1000 % cez levi rob
             x0= x0+1; x1=x1+1; xxNov=x_min+move; xx0=xx0+eC; % zamaknem tocke po x v desno, s tem vzamem gradient od sosednje celice, potencial pa bom ustrezno popravil glede na gradient    
           case  100 % cez desni rob
             x0= x0-1; x1=x1-1; xxNov=x_max-move; xx0=xx0-eC;  % zamaknem tocke po x v levo, s tem vzamem gradient od sosednje celice, potencial pa bom ustrezno popravil glede na gradient    
           case   10 % dol od roba
             y0= y0+1; y1=y1+1; yyNov=y_min+move; yy0=yy0+eC;      
           case    1 % gor od roba
             y0= y0-1; y1=y1-1; yyNov=y_max-move; yy0=yy0-eC;   
           
           case 1010 % ce vogal levo-dol rob
             x0= x0+1; x1=x1+1; y0= y0+1; y1=y1+1; xxNov=x_min+move; yyNov=y_min+move;    % zamaknem tocke po x v desno, s tem vzamem gradient od sosednje celice, potencial pa bom ustrezno popravil glede na gradient    
                xx0=xx0+eC;yy0=yy0+eC;
            case 0110 % ce vogal desno-dol rob
             x0= x0-1; x1=x1-1; y0= y0+1; y1=y1+1; xxNov=x_max-move; yyNov=y_min+move;  % zamaknem tocke po x v desno, s tem vzamem gradient od sosednje celice, potencial pa bom ustrezno popravil glede na gradient    
                xx0=xx0-eC; yy0=yy0+eC;
           case 1001 % ce vogal levo-gor rob
             x0= x0+1; x1=x1+1; y0= y0-1; y1=y1-1; xxNov=x_min+move; yyNov=y_max-move;  % zamaknem tocke po x v desno, s tem vzamem gradient od sosednje celice, potencial pa bom ustrezno popravil glede na gradient    
                xx0=xx0+eC; yy0=yy0-eC;
            case 0101 % ce vogal desno-gor rob
             x0= x0-1; x1=x1-1; y0= y0-1; y1=y1-1; xxNov=x_max-move; yyNov=y_max-move;  % zamaknem tocke po x v desno, s tem vzamem gradient od sosednje celice, potencial pa bom ustrezno popravil glede na gradient    
                xx0=xx0-eC; yy0=yy0-eC;
        end
      
         x=(xxNov-xx0)/eC;
         y=(yyNov-yy0)/eC;
 
   end;   % kakšen gradient dat tu - oceni ga iz sosednjih celic
   
  
   
   % doloci se gradient (izpeljava: Script_DolocitevGradientaBicubic.m)
   if 0
       syms xx yy xx0 yy0 dc a00 a10 a01 a11 
       x=(xx-xx0)/dc;y=(yy-yy0)/dc;
       f=a00 + a10*x + a01*y + a11*x*y;
       gradX=diff(f,'xx')
       gradY=diff(f,'yy')
       
       latex(gradX)
       
       pretty(gradX)
       pretty(simplify(gradX))
       pretty(simplifyFraction(gradX))
        
        
        % oz. v normiranih koordinatah
       syms x y dc a00 a10 a01 a11 
       f=a00 + a10*x + a01*y + a11*x*y;
       gradX=diff(f,'x')
       gradY=diff(f,'y')
       
       % oz. v normiranih koordinatah z utezmi
        syms x y dc f00 f01 f10 f11
       w00=(1-x)*(1-y);
       w01=(1-x)*y;
       w10=x*(1-y);
       w11=x*y;
       f=w00*f00 + w01*f01 + w10*f10 + w11*f11;
       gradX=diff(f,'x')
       gradY=diff(f,'y')

   end

   
   f00=sfGetCorrectedPotentialForObstacles(x0,y0,maxFF,sizeX,sizeY,eC,FF);
   f01=sfGetCorrectedPotentialForObstacles(x0,y1,maxFF,sizeX,sizeY,eC,FF);
   f10=sfGetCorrectedPotentialForObstacles(x1,y0,maxFF,sizeX,sizeY,eC,FF);
   f11=sfGetCorrectedPotentialForObstacles(x1,y1,maxFF,sizeX,sizeY,eC,FF);
  
   w00=(1-x)*(1-y);
   w01=(1-x)*y;
   w10=x*(1-y);
   w11=x*y;

   
   % interpoliran cost za pozicijo  
   cost=w00*f00 + w01*f01 + w10*f10 + w11*f11;

   
   
   %gradX = a10/ecell + (a11*(yy - yy0))/ecell^2; % v absolutnih koordinatah
   %gradY = a01/ecell + (a11*(xx - xx0))/ecell^2;
   %gradX = (a10 + a11*y)/ecell;                  % normirane in a-ji
   %gradY = (a01 + a11*x)/ecell;                 

   gradX = ( f11*y - f01*y + f00*(y - 1) - f10*(y - 1) )/eC;   % normirane in utezi w
   gradY = ( f11*x - f10*x + f00*(x - 1) - f01*(x - 1) )/eC;

   
 % popravek za cost na robu
 if(Loc~=0)
   grad=[gradX,gradY];
   %vect=[xxNov;yyNov]-[xx;yy];
   %cost=cost - grad*vect;  % popravek costa
   
   vect=[xx;yy]-[xxNov;yyNov];
   cost=cost + grad*vect;  % popravek costa

 end

 
  
 rrr=[cost,gradX,gradY];   
  

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




function pot=sfGetCorrectedPotentialForObstacles(xi,yj,maxFF,sizeX,sizeY,ecell,FF)

pot=[];
   if( xi<=1 || xi>=sizeX || yj<=1 || yj>=sizeY )   % sem na robu okolja

       if(FF(xi,yj)>maxFF) % ovira na robu (oz. ena od stirih tock za interpolacijo je ovira na robu)
          
                  if(xi<=1)
                      if(yj>=sizeY)
                          ii=[FF(1,yj-1) ,FF(2,sizeY-1),FF(2,sizeY)];
                      elseif(yj<=1)
                          ii=[FF(1+1,1),FF(2,2), FF(xi,2)];
                      else
                          ii=[FF(1,yj-1) ,FF(2,yj-1),FF(2,yj),FF(2,yj+1), FF(1,yj+1)];
                      end
                  elseif(xi>=sizeX)   
                      if(yj>=sizeY)
                          ii=[FF(sizeX,sizeY-1) ,FF(sizeX-1,sizeY-1),FF(sizeX-1,sizeY)];
                      elseif(yj<=1)
                          ii=[FF(sizeX-1,1),FF(sizeX-1,2), FF(sizeX,2)];
                      else
                          ii=[FF(sizeX,yj-1) ,FF(sizeX-1,yj-1),FF(sizeX-1,yj),FF(sizeX-1,yj+1), FF(sizeX,yj+1)];
                      end

                  elseif(yj>=sizeY)   
                      ii=[FF(xi-1,sizeY),FF(xi-1,sizeY-1),FF(xi,sizeY-1),FF(xi+1,sizeY-1) ,FF(xi+1,sizeY)];

                  elseif(yj<=1)   
                      ii=[FF(xi-1,1),FF(xi-1,2),FF(xi,2),FF(xi+1,2), FF(xi+1,1)];
                  else
                      ii=[FF(xi-1,yj+1),FF(xi-1,yj),FF(xi-1,yj-1),  FF(xi,yj-1) ,FF(xi+1,yj-1),FF(xi+1,yj),FF(xi+1,yj+1), FF(xi,yj+1)];
                      disp('strange - check function tmp_GetInterpolatedBiLinearCostToGoal2_Test.m')
                  end

                  [pot]=max(ii(ii<maxFF));   % ??? to na slepo povecanje ni ok, potrebno je pogledati gradient

                   %pot=pot*1.10; 
                   pot=pot+ecell*sqrt(2); 
      
       else 
          pot=FF(xi,yj); 
       end  
       
       return;
   end

      
      
    
    if(FF(xi,yj)>maxFF && 1) % ce je celica ovira
       % vzamem malo vec kot je maks okoliskih, ki niso na maxFF 
       %ii=[FF(xi-1,yj),FF(xi+1,yj) ,FF(xi,yj-1),FF(xi,yj+1)];
       ii=[FF(xi-1,yj+1),FF(xi-1,yj),FF(xi-1,yj-1),  FF(xi,yj-1) ,FF(xi+1,yj-1),FF(xi+1,yj),FF(xi+1,yj+1), FF(xi,yj+1)];
       [pot]=max(ii(ii<maxFF));   % ??? to na slepo povecanje ni ok, potrebno je pogledati gradient
      
      % pot=pot*1.10; 
       pot=pot+ecell*sqrt(2); 
       
       if isempty(pot),  % ce se slucajno zgodi, da so vse sosedne celice na oviri
           pot=maxFF*1.1;  
       end  
       
       % pot=maxFF*1.1;
    else
       pot=FF(xi,yj); 
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


