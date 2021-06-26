clear;close all;clc;

three_dof_pen_parameters;

tau=[];
init_t=0;
final_t=20;
dt=0.001;
N= (final_t-init_t)/dt;
t_span=linspace(init_t,final_t,N);

x01=[pi/2 0 pi/1.5 0 pi 0]';
% x01=[pi pi pi]'; % vertically above
% x01=[0 0 0 0 0 0]'; %vertically suspended

[t,y1] = ode45(@three_dof_arm_dyn_for_ODE2,t_span,x01);

x1=y1;

myVideo = VideoWriter('threeDOFarm2_single_wD_woT.avi');
mov_cnt = 1;
open(myVideo)
figure; hold on;
set(gca,'Color','k')
h1=animatedline('Color','b','Linewidth',2);

range=1:50:N-1;
for i=range
    x11=x1(i,1); x21=x1(i,3); x31=x1(i,5);
    p0x1=0; p0y1=0;
    p0x2=0; p0y2=0;
    p0x3=0; p0y3=0;
    p0x4=0; p0y4=0;
    p0x5=0; p0y5=0;
    p1x1 = L1*sin(x11);
    p1y1 = -L1*cos(x11);
    p2x1 = p1x1+L2*sin(x21);
    p2y1 = p1y1-L2*cos(x21);
    p3x1 = p2x1+L3*sin(x31);
    p3y1 = p2y1-L3*cos(x31);
    px1=[p0x1 p1x1 p2x1 p3x1];
    py1=[p0y1 p1y1 p2y1 p3y1];
    f1=plot(px1,py1,'w.-');
    addpoints(h1,px1(4),py1(4));
    drawnow;
    axis([-4 4 -4 4]);
%     axis([-1 1 -1 1]);
    pause(0.001);
    MM(mov_cnt)=getframe;
    frame=getframe(gcf);
    writeVideo(myVideo,frame);
    mov_cnt=mov_cnt+1;
    if i~=range(end)
        delete(f1);
    end

end
close(myVideo);

% movie(MM)
% movie2avi(MM,'threeDOFarm.avi');
%%
figure(2)
subplot(2,1,1)
plot(t,wrapToPi(y1(:,[1,3,5])));
xlabel('Time(t)');
ylabel('Joint Position($\theta$)','Interpreter','latex');
grid on;
legend('\theta_1','\theta_2','\theta_3')

subplot(2,1,2)
plot(t,y1(:,[2,4,6]));
xlabel('Time(t)');
ylabel('Joint Angular Velocity($\dot{\theta}$)','Interpreter','latex')
grid on;
legend('$\dot{\theta_1}$','$\dot{\theta_2}$','$\dot{\theta_3}$','Interpreter','latex');

figure(3)
tau=[];
for j=1:length(t)
    tau= [tau;u_calc(t(j),y1(j,:))'];
end
plot(t,tau);
xlabel('Time(t)');
ylabel('Joint Torques($\tau$)','Interpreter','latex')
grid on;
legend('$\tau_1$','$\tau_2$','$\tau_3$','Interpreter','latex');

