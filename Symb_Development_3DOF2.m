clear;close all;clc;
% 
syms M1 M2 M3;
syms I1 I2 I3;
syms x1 x1d x1dd x2 x2d x2dd x3 x3d x3dd;
syms L1 L2 L3;
syms k1 k2 k3;
syms u1 u2 u3;
syms g

p1x = (L1/2)*sin(x1);
p1y = -(L1/2)*cos(x1);
p2x = 2*p1x+(L2/2)*sin(x2);
p2y = 2*p1y-(L2/2)*cos(x2);
p3x = 2*p2x-2*p1x+(L3/2)*sin(x3);
p3y = 2*p2y-2*p1y-(L3/2)*cos(x3);

v1x = (L1/2)*cos(x1)*x1d;
v1y = (L1/2)*sin(x1)*x1d;
v2x = 2*v1x+(L2/2)*cos(x2)*x2d;
v2y = 2*v1y+(L2/2)*sin(x2)*x2d;
v3x = 2*v2x-2*v1x+ (L3/2)*cos(x3)*x3d;
v3y = 2*v2y-2*v1y+ (L3/2)*sin(x3)*x3d;

TKE = 0.5*M1*( v1x^2 + v1y^2) + 0.5*M2*( v2x^2 + v2y^2) + 0.5*M3*( v3x^2 + v3y^2);
RKE = 0.5*I1*x1d^2 + 0.5*I2*x2d^2 +0.5*I3*x3d^2;
KE = simplify(TKE+RKE);

PE = M1*g*p1y + M2*g*p2y + M3*g*p3y;
PE = simplify(PE);

D = 0.5*k1*x1d^2 + 0.5*k2*x2d^2 +0.5*k3*x3d^2;

Px1 = u1;
Px2 = u2;
Px3 = u3;

pKEpx1d = diff(KE,x1d);
ddtpKEpx1d = diff(pKEpx1d,x1)*x1d+ ...
             diff(pKEpx1d,x1d)*x1dd+ ...
             diff(pKEpx1d,x2)*x2d + ...
             diff(pKEpx1d,x2d)*x2dd + ...
             diff(pKEpx1d,x3)*x3d + ...
             diff(pKEpx1d,x3d)*x3dd;
pKEpx1 = diff(KE,x1);
pPEpx1 = diff(PE,x1);
pDpx1d = diff(D,x1d);

pKEpx2d = diff(KE,x2d);
ddtpKEpx2d = diff(pKEpx2d,x1)*x1d+ ...
             diff(pKEpx2d,x1d)*x1dd+ ...
             diff(pKEpx2d,x2)*x2d + ...
             diff(pKEpx2d,x2d)*x2dd + ...
             diff(pKEpx2d,x3)*x3d + ...
             diff(pKEpx2d,x3d)*x3dd;
pKEpx2 = diff(KE,x2);
pPEpx2 = diff(PE,x2);
pDpx2d = diff(D,x2d);

pKEpx3d = diff(KE,x3d);
ddtpKEpx3d = diff(pKEpx3d,x1)*x1d+ ...
             diff(pKEpx3d,x1d)*x1dd+ ...
             diff(pKEpx3d,x2)*x2d + ...
             diff(pKEpx3d,x2d)*x2dd + ...
             diff(pKEpx3d,x3)*x3d + ...
             diff(pKEpx3d,x3d)*x3dd;
pKEpx3 = diff(KE,x3);
pPEpx3 = diff(PE,x3);
pDpx3d = diff(D,x3d);

eqx1 = simplify( ddtpKEpx1d - pKEpx1 + pPEpx1 + pDpx1d - Px1)
eqx2 = simplify( ddtpKEpx2d - pKEpx2 + pPEpx2 + pDpx2d - Px2)
eqx3 = simplify( ddtpKEpx3d - pKEpx3 + pPEpx3 + pDpx3d - Px3)

Sol = solve(eqx1,eqx2,eqx3,x1dd,x2dd,x3dd);
Sol.x1dd = simplify(Sol.x1dd);
Sol.x2dd = simplify(Sol.x2dd);
Sol.x3dd = simplify(Sol.x3dd);

x_dd = [Sol.x1dd;Sol.x2dd;Sol.x3dd];

% 
syms y1 y2 y3 y4 y5 y6
fx1=subs(Sol.x1dd,{x1,x1d,x2,x2d,x3,x3d},{y1,y2,y3,y4,y5,y6})
fx2=subs(Sol.x2dd,{x1,x1d,x2,x2d,x3,x3d},{y1,y2,y3,y4,y5,y6})
fx3=subs(Sol.x3dd,{x1,x1d,x2,x2d,x3,x3d},{y1,y2,y3,y4,y5,y6})

%% Determining in matrix form
M_thdd = [collect(eqx1,{'x1dd','x2dd','x3dd'});
            collect(eqx2,{'x1dd','x2dd','x3dd'});
            collect(eqx3,{'x1dd','x2dd','x3dd'})]; % This gives the main coefficients for each th_dd which are copied to form following matrices

Mx = [(I1 + (L1^2*M1)/4 + L1^2*M2 + L1^2*M3), ((L1*L2*M2*cos(x1 - x2))/2 + L1*L2*M3*cos(x1 - x2)), ((L1*L3*M3*cos(x1 - x3))/2);
        ((L1*L2*M2*cos(x1 - x2))/2 + L1*L2*M3*cos(x1 - x2)), (I2 + (L2^2*M2)/4 + L2^2*M3), ((L2*L3*M3*cos(x2 - x3))/2);
        ((L1*L3*M3*cos(x1 - x3))/2), ((L2*L3*M3*cos(x2 - x3))/2), ((M3*L3^2)/4 + I3)];

Fx =  [k1*x1d + (L1*M1*g*sin(x1))/2 + L1*M2*g*sin(x1) + L1*M3*g*sin(x1) + (L1*L2*M2*x2d^2*sin(x1 - x2))/2 + L1*L2*M3*x2d^2*sin(x1 - x2) + (L1*L3*M3*x3d^2*sin(x1 - x3))/2;
        k2*x2d + (L2*M2*g*sin(x2))/2 + L2*M3*g*sin(x2) - (L1*L2*M2*x1d^2*sin(x1 - x2))/2 - L1*L2*M3*x1d^2*sin(x1 - x2) + (L2*L3*M3*x3d^2*sin(x2 - x3))/2;
        k3*x3d - (L2*L3*M3*sin(x2 - x3)*x2d^2)/2 - (L1*L3*M3*sin(x1 - x3)*x1d^2)/2 + (L3*M3*g*sin(x3))/2];

M = subs(Mx,{x1,x1d,x2,x2d,x3,x3d},{y1,y2,y3,y4,y5,y6})
F = subs(Fx,{x1,x1d,x2,x2d,x3,x3d},{y1,y2,y3,y4,y5,y6})