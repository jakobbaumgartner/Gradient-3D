close all

% sestavim neko pot iz tock 
ts=0.1;
L=[]; X=[]; Y=[];
tt=0:ts:4;
xx= tt*0.5;
yy=0.2*sin(2*tt);

L=[L,tt]; X=[X,xx]; Y=[Y,yy];

%figure(1),plot(X,Y)

tt=ts:ts:4; 
xx= X(end)*ones(size(tt));
yy=Y(end) + tt*.5;
L=[L,tt+L(end)]; X=[X,xx]; Y=[Y,yy];

%figure(1),plot(X,Y)




% nafitam polinom na podatke
N=9
ax=polyfit(L,X,N);
ay=polyfit(L,Y,N);

% preverim ustreznost
x= polyval(ax,L);
y= polyval(ay,L);
figure, plot(x,y,X,Y,'.')

%return
% odvodi polinomske poti

L=L; % parameter vzdolz poti
Ks=1; % pohitritev/upocasnitev po isti poti se pelješ hitreje ali po?asneje
%t=Ks*L;  % cas
ts=0.01;

t=0:ts:L(end)*Ks; % cas 
l=0:ts/Ks:L(end); % parameter vzdolz poti 


N=length(ax)-1;
x=zeros(size(l)); y=zeros(size(l));
for n=1:N+1
     x=x+ax(n)*l.^(N+1-n);
     y=y+ay(n)*l.^(N+1-n);
end

figure, plot(x,y,X,Y,'.')

% prvi odvodi
dx=zeros(size(l)); dy=zeros(size(l));
for n=1:N
    dx=dx+1/Ks*(N+1-n)*ax(n)*l.^(N-n);
    dy=dy+1/Ks*(N+1-n)*ay(n)*l.^(N-n);
end

v=sqrt(dx.^2+dy.^2);
figure, plot(t,v)


% drugi odvodi
ddx=zeros(size(l)); ddy=zeros(size(l));
for n=1:N-1
    ddx=ddx+1/Ks^2*(N+1-n)*(N-n)* ax(n)*l.^(N-n-1);
    ddy=ddy+1/Ks^2*(N+1-n)*(N-n)* ay(n)*l.^(N-n-1);
end


a=sqrt(ddx.^2+ddy.^2);
figure, plot(t,a)



if 0

l=0:ts:10;
Ks=1;

t=Ks*l;  % pohitritev

% krivulja polinom 4 reda
ax=[.1 2 .3 4 2];
ay=[.3 2 1 6 3];
az=[.1 .2 1 3 1];

x= ax(1)*l.^4+ax(2)*l.^3+ax(3)*l.^2+ax(4)*l+ax(5)*ones(size(t));
y= ay(1)*l.^4+ay(2)*l.^3+ay(3)*l.^2+ay(4)*l+ay(5)*ones(size(t));
z= az(1)*l.^4+az(2)*l.^3+az(3)*l.^2+az(4)*l+az(5)*ones(size(t));


dx=Ks*(4*ax(1)*l.^3+3*ax(2)*l.^2+2*ax(3)*l.^1+ax(4)*ones(size(t)));
dy=Ks*(4*ay(1)*l.^3+3*ay(2)*l.^2+2*ay(3)*l.^1+ay(4)*ones(size(t)));
dz=Ks*(4*az(1)*l.^3+3*az(2)*l.^2+2*az(3)*l.^1+az(4)*ones(size(t)));

plot3(x,y,z)
plot(x,y)

v=sqrt(dx.^2+dy.^2+dz.^2);
figure, plot(t,v)
end