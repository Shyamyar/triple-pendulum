clear;close all;clc;

three_dof_pen_parameters;

init_t=0;
final_t=20;
dt=0.001;
N= (final_t-init_t)/dt;
t_span=linspace(init_t,final_t,N);

x01=[pi/2 0 pi/1.5 0 pi 0]'; % for undamped
% x01=[pi 0 pi/1.5 0 pi/2 0]'; % for damped
x02=1.001*x01;
x03=1.002*x01;
x04=1.003*x01;
x05=1.004*x01;

[t,y1] = ode45(@three_dof_arm_dyn_for_ODE2,t_span,x01);
[t,y2] = ode45(@three_dof_arm_dyn_for_ODE2,t_span,x02);
[t,y3] = ode45(@three_dof_arm_dyn_for_ODE2,t_span,x03);
[t,y4] = ode45(@three_dof_arm_dyn_for_ODE2,t_span,x04);
[t,y5] = ode45(@three_dof_arm_dyn_for_ODE2,t_span,x05);

x1=y1; x2=y2; x3=y3; x4=y4; x5=y5;

myVideo = VideoWriter('threeDOFarm2_multiple_wD_woT_traj.avi');
mov_cnt = 1;
figure; hold on;
set(gca,'Color','k')
h1=animatedline('Color','b');
h2=animatedline('Color','g');
h3=animatedline('Color','y');
h4=animatedline('Color','r');
h5=animatedline('Color','c');

for i=1:N-1
    open(myVideo)
    if(mod(i,50)==1)
        x11=x1(i,1); x21=x1(i,3); x31=x1(i,5);
        x12=x2(i,1); x22=x2(i,3); x32=x2(i,5);
        x13=x3(i,1); x23=x3(i,3); x33=x3(i,5);
        x14=x4(i,1); x24=x4(i,3); x34=x4(i,5);
        x15=x5(i,1); x25=x5(i,3); x35=x5(i,5);
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
        p1x2 = L1*sin(x12);
        p1y2 = -L1*cos(x12);
        p2x2 = p1x2+L2*sin(x22);
        p2y2 = p1y2-L2*cos(x22);
        p3x2 = p2x2+L3*sin(x32);
        p3y2 = p2y2-L3*cos(x32);
        p1x3 = L1*sin(x13);
        p1y3 = -L1*cos(x13);
        p2x3 = p1x3+L2*sin(x23);
        p2y3 = p1y3-L2*cos(x23);
        p3x3 = p2x3+L3*sin(x33);
        p3y3 = p2y3-L3*cos(x33);
        p1x4 = L1*sin(x14);
        p1y4 = -L1*cos(x14);
        p2x4 = p1x4+L2*sin(x24);
        p2y4 = p1y4-L2*cos(x24);
        p3x4 = p2x4+L3*sin(x34);
        p3y4 = p2y4-L3*cos(x34);
        p1x5 = L1*sin(x15);
        p1y5 = -L1*cos(x15);
        p2x5 = p1x5+L2*sin(x25);
        p2y5 = p1y5-L2*cos(x25);
        p3x5 = p2x5+L3*sin(x35);
        p3y5 = p2y5-L3*cos(x35);
        px1=[p0x1 p1x1 p2x1 p3x1];
        py1=[p0y1 p1y1 p2y1 p3y1];
        f1=plot(px1,py1,'w.-');
        addpoints(h1,px1(4),py1(4));
        px2=[p0x2 p1x2 p2x2 p3x2];
        py2=[p0y2 p1y2 p2y2 p3y2];
        f2=plot(px2,py2,'w.-');
        addpoints(h2,px2(4),py2(4));
        px3=[p0x3 p1x3 p2x3 p3x3];
        py3=[p0y3 p1y3 p2y3 p3y3];
        f3=plot(px3,py3,'w.-');
        addpoints(h3,px3(4),py3(4));
        px4=[p0x4 p1x4 p2x4 p3x4];
        py4=[p0y4 p1y4 p2y4 p3y4];
        f4=plot(px4,py4,'w.-');
        addpoints(h4,px4(4),py4(4));
        px5=[p0x5 p1x5 p2x5 p3x5];
        py5=[p0y5 p1y5 p2y5 p3y5];
        f5=plot(px5,py5,'w.-');
        addpoints(h5,px5(4),py5(4));
        drawnow;
        axis([-4 4 -4 4]);
%         axis([-1 1 -1 1]);
        pause(0.001);
        MM(mov_cnt)=getframe;
        frame=getframe(gcf);
        writeVideo(myVideo,frame);
        mov_cnt=mov_cnt+1;
        delete(f1);
        delete(f2);
        delete(f3);
        delete(f4);
        delete(f5);

    end
end
close(myVideo);
% movie(MM)
