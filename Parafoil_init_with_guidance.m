%Initialize variables for Parafoil.slx
clear all; close all; clc

%Contorl Inputs
% dr = 0; % right motor control input from guidance
% dl = 0; % left motor control input from guidance
% da = dr-dl; %asymmetric brake deflection
db = 0.5; %symmetric brake deflection\
gamma_0 = -25*pi/180; %nomianl incident angle

% Trajectory planning
p_h = [30;25;0]; % Landing point

% PID coefficients
Kp = 1.0;
Ki = 0.0009;
Kd = 0.0;


%Initial conditions
v_0 = [7.135; 0; 4.222]; %initial velocity
w_0 = [0;0;0]; % initial angular rate

%initial angles
phi_0 = 0*pi/180; 
theta_0 = 0*pi/180;
psi_0 = 0;

%lift coefficients
CL_0 = 0;
CL_a = 3.9;
CL_a3 = 20;
CL_db = 0;
CL_adb = .39;
CL_a2db = 2.062;
%drag coefficients
CD_0 = .153;
CD_a2 = 2.4;
CD_db = .043;
CD_p = .5;
%slide slip
CY_b = -1;
%moment coefficients
Cl_b = .001;% not listed in Cacan
Cl_r = 0;
Cl_da= 0;
Cl_p = -.1;
Cm_q = -2.5;
Cm_db = 0;
Cn_b = .02;
Cn_p = 0;
Cn_r = -.11;
Cn_da = .005;
Cn_da2 = 0;

%wind velocity
Vw_x = 0;
Vw_y = 0;
Vw_z = 0;

%model parameters
m = 2.7;
g = 9.81;
rho = 1.3;
b = 1.8;
c = .75;
Sc = b*c;
Sp = .01;
r_cg2c = [.5;0;-4];
r_cg2p = [-2; 0; 1];

Ixx = 1.93; Iyy = .57; Izz = .37; 
Ixy = 0; Iyx = 0; 
Iyz = 0; Izy = 0; 
Ixz = .104; Izx = .104; 

I_B = [Ixx Ixy Ixz
       Iyx Iyy Iyz
       Izx Izy Izz]; 
I_AM = diag([.05 .35 1.85]); 
I_AI = diag([.07 .06 .046]);
%Eq 2.36 LHS
S_rcg2c = [0 -r_cg2c(3) r_cg2c(2)
           r_cg2c(3) 0 -r_cg2c(1)
           -r_cg2c(2) r_cg2c(1) 0]; %distance from cg to canopy cross product operator
% A = inv([m*eye(3,3) + I_AM.',-I_AM.'*S_rcg2c
%     S_rcg2c*I_AM.', I_B+I_AI.' - S_rcg2c*I_AM.'*S_rcg2c]);  


%% run simulation
sim('Parafoil_NL_6DoF_with_Guidance',400)
%% data

t = ans.simout.Time;
x = squeeze(ans.simout.Data(1,1,:));
y = squeeze(ans.simout.Data(2,1,:));
z = squeeze(ans.simout.Data(3,1,:));
% xx = ans.simout.Data(:,1);
% yy = ans.simout.Data(:,2);
% zz = ans.simout.Data(:,3);
% plot3(xx,yy,-zz);
plot3(x,y,-z);
xlabel('x');ylabel('y');zlabel('Altitude')
grid on; hold on
p_h(3) = -z(end);
plot3(p_h(1),p_h(2),p_h(3),'ro','MarkerSize',4,'MarkerFaceColor','r')
plot3(x(1),y(1),-z(1),'kd','MarkerSize',4,'MarkerFaceColor','k')
legend('Flight Path','Landing Point','Start Point','Location','Best')
% %% 
% % close all
% % t = out.simout.Time;
% % x = squeeze(out.simout.Data(1,1,:));
% % y = squeeze(out.simout.Data(2,1,:));
% % z = squeeze(out.simout.Data(3,1,:));
% % xx = out.simout.Data(:,1);
% % yy = out.simout.Data(:,2);
% % zz = out.simout.Data(:,3);
% % 
% % plot3(xx,yy,-zz);
% % xlabel('x');ylabel('y');zlabel('Altitude')
% % grid on;