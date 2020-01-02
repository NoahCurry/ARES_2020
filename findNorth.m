function [North,dlat,dlon,th] = findNorth(lon1,lon2,lat1,lat2,Kp)
dlon = lon2-lon1;
dlat = lat2-lat1;
th = atan2(dlat,dlon);
        
North = Kp*th;    
end