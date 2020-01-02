function [rightwayEW, rightwayNS] = checkDirection(lon1,lon2,lonH,lat1,lat2,latH)
dlon2 = abs(lonH-lon2);
dlon1 = abs(lonH-lon1);
dlat2 = abs(latH-lat2);
dlat1 = abs(latH-lat1);

% Check longitudinal direction
if dlon2 > dlon1
    rightwayEW = false;
elseif dlon1 > dlon2
    rightwayEW = true;
else
    rightwayEW = 0;
end
    
% Check latitudinal direction
if dlat2 > dlat1
    rightwayNS = false;
elseif dlat1 > dlat2
    rightwayNS = true;
else
    rightwayNS = 0;
end
     
end