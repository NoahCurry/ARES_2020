clear all; close all; clc;

%% Initialize variables %%
c = 3; % Fall rate (m/s)
% vel = 6; % velocity (m/s)
xf = 0; x1 = xf + (90-xf)*rand; x2 = x1 + (1-x1)*rand;
yf = 0; y1 = yf + (90-yf)*rand; y2 = y1 + (1-y1)*rand;
zf = 0; z1 = 1000 + 100*rand; z2 = z1;

r0 = sqrt((x1-xf)^2+(y1-yf)^2); % perpendicular radius from landing point axis (m)
rf = 100; % landing zone radius (m)
T = (z1-zf)/c; % Time until ground (s)

P1 = [x1 y1 z1]; % Initial point (m,m,m)
P2 = [x2 y2 z2]; % Initial heading (randomly generated m,m,m)
Pf = [xf yf zf]; % Designated landing point (m,m,m)
Af = @(x,y)((x)^2+(y)^2); % Designated landing zone when z = 0

%% Part 1: Create ideal helical path %%

% Helix functions %

% Ideal position function r(t) 
xrf = @(t)(r0*cos(t));
yrf = @(t)(r0*sin(t));
zrf = @(t)(z1-c*t);
% Ideal velocity function v(t) 
xvf = @(t)(-r0*sin(t));
yvf = @(t)(r0*cos(t)); 
zvf = -c;

% Ideal position and velocity array
n = 1000; % number of points in array
t = 0:T/(n-1):T; % time vector
dt = T/n; % sample time interval
pi = zeros(3, n);
vi = zeros(3, n);
for k = 1:length(t) % Creates ideal position and tangent/velocity vector arrays
    pi(:,k) = [xrf(t(k)); yrf(t(k)); zrf(t(k))];
    vi(:,k) = [xvf(t(k)); yvf(t(k)); zvf];
end


%% Part 2: Create simulated initial position and simulated and ideal flight vectors
pa = zeros(3,n); pa(:,1) = P1; pa(:,2) = P2; % actual position points P(t)
fi = zeros(3,n); 
ha = zeros(3,n);
psi = zeros(1,n);
% zeta = zeros(1,n);
theta1 = zeros(1,n);
theta2 = zeros(1,n);
Kp = 0.9; Ki = 0.1; Kd = 0.5; 

for k = 2:length(t)
    ha(:,k) = [pa(:,k)]-[pa(:,k-1)]; % actual heading vectors
    fi(:,k) = [pi(:,k)]-[pa(:,k-1)]; % correction vectors
    %fi(:,k) = vi(:,k); % ideal head vectors
    vel = norm(fi(:,k));
    pa(3,k) = z1-c*t(k); 
    
    psi = acosd(dot(fi(:,k),ha(:,k))/(norm(fi(:,k))*norm(ha(:,k)))); % angle error
    psi(k) = psi(1);
    
    %zeta = Kp*psi + Ki*trapz(dt, psi) + Kd*(psi/dt); % course correction from PID
    
    %zeta(k) = zeta(1);
    
    theta1(k-1) = atan2d(ha(2,k),ha(1,k));
    theta2(k-1) = atan2d(fi(2,k),fi(1,k));
    if theta2(k-1) < 0
        theta2(k-1) = 360+theta2(k-1);
    end
    if theta1(k-1)-theta2(k-1) > 0  %% Turn right
        pa(1,k+1) = pa(1,k) + vel*dt*cosd(theta1(k-1)-30);
        pa(2,k+1) = pa(2,k) + vel*dt*sind(theta1(k-1)-30);
    elseif theta1(k-1)-theta2(k-1) < 0 %% Turn left
        pa(1,k+1) = pa(1,k) + vel*dt*cosd(theta1(k-1)+30);
        pa(2,k+1) = pa(2,k) + vel*dt*sind(theta1(k-1)+30);
    end
    
end
th = 0:pi/500:2*4*atan(1);
xunit = rf * cos(th);
yunit = rf * sin(th);
zer = zeros(1,length(th));
plot3(pi(1,:),pi(2,:),pi(3,:), 'k-.')
hold on
plot3(pa(1,:),pa(2,:),pa(3,:), 'r-')
hold on
plot3(xunit,yunit,zer,'b-')
legend('Ideal Path', 'Actual Path', 'Landing Zone')
    


