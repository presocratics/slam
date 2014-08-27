%data = mportdata('Jul1920140545_CrystalLakeEast/loc',',',1);
%data = importdata('loc',',',1);
%data = data.data;
data = loc;
stepStart = max(find(data(:,2)==0))+1;
stepEnd = length(data)-1;
time = (data(stepEnd,1) - data(1,1))/1e6;
a = 6378137;
f = 1/298.257223563; b = a*(1 - f); e2 = 1 - (b/a)^2;
for k = stepStart:stepEnd
    GPS_latitude = data(k,2)/1e7;
    altitude = data(k,4)/1e3 + 1.021;
    
    lat = getSphericalLatitude(GPS_latitude, altitude);
    lon = data(k,3)/1e7;
    GPShist(k,:) = [lat lon];
    
    %R = 6,378.1370*1000; %m equator
    %% To cartesian Coord
    %x = R * cos(lat) * cos(lon);
    %y = R * cos(lat) * sin(lon);
    %z = R *sin(lat);
    Nphi = a ./ sqrt(1 - e2*sin(lat).^2);
    lat = lat*pi/180; 
    lon = lon*pi/180;
    
    compare(k,:) = [lat lon];
    xbwHist(k,:) = [(Nphi + altitude).*cos(lat).*cos(lon) ...
                    (Nphi + altitude).*cos(lat).*sin(lon) ...
                    altitude];
    Nphi

end

xbwHist(stepStart:stepEnd,1:2) = xbwHist(stepStart:stepEnd,1:2) - ones(stepEnd-stepStart+1,1)*xbwHist(stepStart,1:2);

stepStart = 11470;
stepEnd = 20640;

%% travelled distance
travel(stepStart) = 0;
for k = stepStart+1:stepEnd
    travel(k) = travel(k-1) + norm(xbwHist(k,1:2) - xbwHist(k-1,1:2));
end
travel(stepEnd)

%% plotting
timeStart = data(stepStart,1);
timeEnd = data(stepEnd,1);
time = (timeEnd-timeStart)/10^6
k = stepStart:stepEnd;

figure
plot(travel')
title('travelled distance'), xlabel('step'), ylabel('distance (m)')

figure
plot(GPShist(stepStart:stepEnd,2),GPShist(stepStart:stepEnd,1)), hold on
plot(GPShist(stepStart,2),GPShist(stepStart,1),'o'), hold on
plot(GPShist(stepEnd,2),GPShist(stepEnd,1),'ro'), hold on
title('GPS'), axis equal

figure
plot(stepStart:stepEnd,xbwHist(stepStart:stepEnd,:)), hold on
plot(stepStart:stepEnd,zeros(stepEnd-stepStart+1,1))
title('location (m)'), xlabel('x'), ylabel('y')

figure
plot3(xbwHist(k,1),xbwHist(k,2),xbwHist(k,3)), hold on
plot3(xbwHist(stepStart,1),xbwHist(stepStart,2),xbwHist(stepStart,3),'o'), hold on
plot3(xbwHist(stepEnd,1),xbwHist(stepEnd,2),xbwHist(stepEnd,3),'ro'), hold on
title('3D location (m)'), axis equal, view(2), xlabel('x'), ylabel('y')

figure
plot(stepStart:stepEnd,xbwHist(stepStart:stepEnd,3)), hold on
plot(stepStart:stepEnd,zeros(stepEnd-stepStart+1,1))
title('altitude (m)')
