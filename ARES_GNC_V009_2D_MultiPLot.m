% This is a multi-call script of the ARES_GNC used to plot multiple flight
% paths on a single plot to show the different starting points and paths. 
close all; clc

for L = 1:20
% This is a 2D version of the ARES_GNC. A simple vector approach is still
% taken to determine the heading angle error and a PID controller is used to
% track the error through time and provide an actuation input. 


% Set desired landing point coordinates in decimal degrees and convert to
% radians
latH = 32.9904*(pi/180); lonH = 101.9750*(pi/180);

% Set initial lat/lon and generate second lat/lon reading. This determines
% the initial heading and velocity. If an increase in initial velocity is
% desired, you may decrease the divisor of the rand value.
latlat = 24; lonlon = 100;
lat1 = latlat*pi/180; lon1 = lonlon*pi/180;
if L < 5
    lat2 = lat1-(rand/500); lon2 = lon1+(rand/500);
    Q = 1;
elseif (5 < L) && (L < 10)
    lat2 = lat1+(rand/500); lon2 = lon1-(rand/500);
    Q = 2;
elseif (10 < L) && (L < 15)
    lat2 = lat1-(rand/500); lon2 = lon1-(rand/500);
    Q = 3;
else
    lat2 = lat1+(rand/500); lon2 = lon1+(rand/500);
    Q = 4;
end
% Set number of loop iterations, prealocate vectors for speed, and set PID
% coefficients. These coefficients will need to be tuned in accordance to
% flight test data (but they seem to be no too shabby as is).
G = 500;
PSI = zeros(1,G);
Position = zeros(G,2);
ZETA = zeros(1,G);
THETA = zeros(G,2);
Kp = 0.050; Ki = 0.003; Kd = 0.001; bias = 0;

%%%%%%%%% GUIDANCE %%%%%%%%%%%%
for p = 1:G
    % Set position vectors, calculate the flight vector FV, and store
    % lat/lon values in an array to plot the path of travel
    P1 = [lon1 lat1]; P2 = [lon2 lat2]; PH = [lonH latH];
    FV = P2-P1;
    HV = PH-P2;
    Position(p,1) = lon1; Position(p,2) = lat1;
    
    % Calculate the distance traveled and the velocity. Also, define the
    % sample time interval (data frequency) dt
    dist = norm(FV);
    dt = 1;
    Vel = dist/dt;
    
    % Calculate the heading angle error psi. This is the main error that is
    % used as the input to the PID controller. 
    psi = acos(dot(FV,HV)/(dist*norm(HV)));
    PSI(p) = psi;
    
    % compute course correction from basic PID control law. This value will
    % be scaled in proportion to the rotational behavior of the DC motors
    % and then sent to the motors to actuate. This is the control value for
    % phase 1 of flight. A different type of controller will be implimented
    % for phase 2 (spiralled decent) and phase 3 (landing)
    if p == 1
        zeta = Kp*psi + Ki*trapz(dt,PSI) + Kd*(PSI(p)/dt) - bias;
    else
        zeta = Kp*psi + Ki*trapz(dt,PSI) + Kd*((PSI(p)-PSI(p-1))/dt) - bias;
    end
    
%    % Set a maximum turn angle to prevent canopie collapse and stalling.
%    % This angle will need to be tuned based on test flight data.
%     if zeta > 30*(pi/180)
%         zeta = 30*(pi/180);
%     elseif zeta < -30*(pi/180)
%         zeta = -30*(pi/180);
%     end
    ZETA(p) = zeta;
 
    % Calculate the interior right-angle theta's of the FV and the landing
    % point vector to determine which direction the system needs to turn.
    % These angles are slightly difficult to describe in a comment but
    % essentially I am using the longitudinal axis as the X-axis and the
    % latitudinal axis as the Y-axis. From this, numerous right triangles
    % can be made. Specifically, two right triangles can be made using the
    % flight vector FV and the homing vector from the previous location.
    % Comparing the magnitudes of these angles can tell the system wether
    % it needs to turn left or right.
    theta1 = atan2((lat2-lat1),(lon2-lon1));
    THETA(p,1) = theta1;
    theta2 = atan2((latH-lat1),(lonH-lon1));
    THETA(p,2) = theta2;
    
    % Check if we are going the right direction
    [rightwayEW, rightwayNS] = checkDirection(lon1,lon2,lonH,lat1,lat2,latH);
   
    if rightwayEW && rightwayNS
        if theta1-theta2 > 0
            fprintf('Turn Right \n')
            lon3 = lon2 + dist*cos(theta1-zeta);
            lat3 = lat2 + dist*sin(theta1-zeta);
        elseif theta1-theta2 < 0
            fprintf('Turn Left \n')
            lon3 = lon2 + dist*cos(theta1+zeta);
            lat3 = lat2 + dist*sin(theta1+zeta);
        end
    else
        % We're not going the right direction so head North
        Kp2 = 0.8;
        [North,dlat,dlon,th] = findNorth(lon1,lon2,lat1,lat2,Kp2);
        if rightwayNS 
            if North == pi/2
                fprintf('North Acquired... ')
                fprintf('Turn Right \n')
                lon3 = lon2 + dist*cos(theta1-zeta);
                lat3 = lat2 + dist*sin(theta1-zeta);
            else
                fprintf('Wrong Way EW... ')
                fprintf('Finding North... Turn Right %d',North*180/pi)
                fprintf(' deg \n')
                fprintf('th = %d \n',th*180/pi)
                lon3 = lon2 + dist*cos(North);
                lat3 = lat2 + dist*sin(North);
            end
        elseif rightwayEW
            if North == pi/2
                fprintf('North Acquired... ')
                fprintf('Turn Right \n')
                lon3 = lon2 + dist*cos(theta1-zeta);
                lat3 = lat2 + dist*sin(theta1-zeta);
            else
                fprintf('Wrong Way EW... ')
                fprintf('Finding North... Turn Left %d',North*180/pi)
                fprintf(' deg \n')
                fprintf('th = %d \n',th*180/pi)
                lon3 = lon2 + dist*sin(North);
                lat3 = lat2 + dist*cos(North);
            end
        else
            fprintf('Wrong Way EW and NS... ')
            fprintf('Finding North... Turn Right %d',Kp2*North*180/pi)
            fprintf(' deg \n')
            lon3 = lon2 + dist*cos(North);
            lat3 = lat2 + abs(dist*sin(North));
%                 else
%                     fprintf('Finding North... Turn Left %d',Kp2*North*180/pi)
%                     fprintf(' deg \n')
%                     lon3 = lon2 + dist*cos(North);
%                     lat3 = lat2 + dist*sin(North);
 
%                 if th > pi/2
%                     fprintf('Finding North... Turn Right %d',Kp2*North*180/pi)
%                     lon3 = lon2 + dist*abs(cos(North));
%                     lat3 = lat2 + dist*sin(North);
%                 else
%                     fprintf('Finding North... Turn Left %d',Kp2*North*180/pi)
%                     lon3 = lon2 + dist*cos(North);
%                     lat3 = lat2 - dist*sin(North);
%                 end
%             end
        end
    end
    lat1 = lat2; lon1 = lon2;
    lat2 = lat3; lon2 = lon3;
    
    % When the system has reached the perimeter of the flight phase 2
    % circle, break the loop and begin entering the circular holding
    % pattern above the landing point.
    if abs(latH-lat1) < 0.01 && abs(lonH-lon1) < 0.02
        fprintf('Target Aquired: ')
        if abs(latH-lat1) < 0.01
            fprintf('LatErr = %d degrees',abs(latH-lat1)*180/pi)
        else
            fprintf(' LonErr = %d degrees',abs(lonH-lon1)*180/pi)
        end
        fprintf('\n')
        break
    end
    
    % Set initial holding pattern radius and calculate flight phase 2
    % perimeter
    rlat = 0.004;
    rlon = 0.02;
    th = linspace(0,2*pi);
    xx = rlat*cos(th)+lonH;
    yy = rlon*sin(th)+latH;
    
    
    % INSERT PHASE 2 CONTROLLER HERE





end
figure(1)
plot(lonlon*pi/180,latlat*pi/180,'b*')
grid on; hold on
plot(Position(1:p,1),Position(1:p,2))
plot(lonH,latH,'ro','MarkerFaceColor','r')
plot(xx,yy,'r--')
end

xlabel('Longitude');ylabel('Latitude')
title('ARES Flight Phase 1: PID heading')
%legend('Starting Point','Path of Travel','Landing Point','Landing Zone','Location','Best')

