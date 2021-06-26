function u=u_calc_pseudo(t,y)
y1 = y(1);
y2 = y(2);
y3 = y(3);
y4 = y(4);
y5 = y(5);
y6 = y(6);

three_dof_pen_parameters;

M =    [I1 + (L1^2*M1)/4 + L1^2*M2 + L1^2*M3, (L1*L2*M2*cos(y1 - y3))/2 + L1*L2*M3*cos(y1 - y3), (L1*L3*M3*cos(y1 - y5))/2
        (L1*L2*M2*cos(y1 - y3))/2 + L1*L2*M3*cos(y1 - y3),                        I2 + (L2^2*M2)/4 + L2^2*M3, (L2*L3*M3*cos(y3 - y5))/2
                        (L1*L3*M3*cos(y1 - y5))/2,                         (L2*L3*M3*cos(y3 - y5))/2,          (M3*L3^2)/4 + I3];

F =    [k1*y2 + (L1*M1*g*sin(y1))/2 + L1*M2*g*sin(y1) + L1*M3*g*sin(y1) + (L1*L2*M2*y4^2*sin(y1 - y3))/2 + L1*L2*M3*y4^2*sin(y1 - y3) + (L1*L3*M3*y6^2*sin(y1 - y5))/2
                  k2*y4 + (L2*M2*g*sin(y3))/2 + L2*M3*g*sin(y3) - (L1*L2*M2*y2^2*sin(y1 - y3))/2 - L1*L2*M3*y2^2*sin(y1 - y3) + (L2*L3*M3*y6^2*sin(y3 - y5))/2
                                                                 k3*y6 + (L3*M3*g*sin(y5))/2 - (L1*L3*M3*y2^2*sin(y1 - y5))/2 - (L2*L3*M3*y4^2*sin(y3 - y5))/2];

N = eye(3,3);
N_half_inv = N;

Kp=10*eye(3,3);
Kv=2*sqrt(Kp);

y_des=[t 0 -0.2*t]';
% y_des=[pi/2 pi/1.5 pi/1.1]'; % as x0
% y_des=[pi pi pi]'; % vertically above
% y_des=[0 0 0]'; %vertically suspended
yd_des=[0 0 0]';

A=eye(3);
b=-Kp*([y1 y3 y5]'-[y_des])-Kv*([y2 y4 y6]'-[yd_des]);
% b=-K*( [y1 y3 y5]'-[y_des]);

M_inv = inv(M);
tau = N_half_inv * pinv(A* M_inv * N_half_inv)*(b+A*M_inv*F);
% tau = M*b+F;
u=tau;