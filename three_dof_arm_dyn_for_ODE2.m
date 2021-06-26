function dy = three_dof_arm_dyn_for_ODE2(t,y)

y1 = y(1);
y2 = y(2);
y3 = y(3);
y4 = y(4);
y5 = y(5);
y6 = y(6);

three_dof_pen_parameters;

% u= u_calc(t,y);
% u=[-5*y(2) -1*y(4) -1*y(6)]';
u=[0 0 0]';

dy = three_dof_arm_dyn2(y,u);