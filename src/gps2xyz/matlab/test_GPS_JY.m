clc; clear all; 
close all;

% dataGPS = importdata('Jul1920140545_CrystalLakeEast/loc',',',1);
dataGPS = importdata('loc',',',1);
dataGPS = dataGPS.data;

stepStart = max(find(dataGPS(:,2)==0))+1;
stepEnd = length(dataGPS)-1;
time = (dataGPS(stepEnd,1) - dataGPS(1,1))/10^6;
Hz = stepEnd/time

a = 6378137; f = 1/298.257223563; b = a*(1 - f); e2 = 1 - (b/a)^2;

for k = stepStart:stepEnd
    latitude = dataGPS(k,2)/10^7;
    longitude = dataGPS(k,3)/10^7;
    GPShist(k,:) = [latitude longitude];
    latitude = latitude*pi/180; 
    longitude = longitude*pi/180;
    altitude = dataGPS(k,4)/10^3 + 1.021;
    Nphi = a ./ sqrt(1 - e2*sin(latitude).^2);
    
    xbwHist(k,:) = [(Nphi + altitude).*cos(latitude).*cos(longitude) ...
                    (Nphi + altitude).*cos(latitude).*sin(longitude) ...
                    altitude];

%     lon = dataGPS(k,2)/10^7;
%     lat = dataGPS(k,3)/10^7;
%     originShift = 2 * pi * 6378137 / 2.0;
%     mx = lon * originShift / 180.0;
%     my = log( tan((90 + lat) * pi / 360.0 )) / (pi / 180.0);
%     my = my * originShift / 180.0;
% 	xbw2Hist(k,:) = [mx my altitude];
end

xbwHist(stepStart:stepEnd,1:2) = xbwHist(stepStart:stepEnd,1:2) - ones(stepEnd-stepStart+1,1)*xbwHist(stepStart,1:2);

% stepStart = 3200;
% stepEnd = 3588;
% stepEnd = 4100;

%! Crystal Lake data followed with a boat
% stepStart = 1

%! steady signal from the deck
% stepStart = 5831
% stepEnd = 8761 

%! hovering - P shape - hovering
stepStart = 11470
% stepStart = 11790
% stepStart = 11830
% stepEnd = 12500
% stepEnd = 14960
% stepEnd = 15230
% stepEnd = 18770
% stepEnd = 19880
% stepEnd = 20290
stepEnd = 20640
% stepEnd = 14400

%% travelled distance
travel(stepStart) = 0;
for k = stepStart+1:stepEnd
    travel(k) = travel(k-1) + norm(xbwHist(k,1:2) - xbwHist(k-1,1:2));
end
travel(stepEnd)

%% plotting
timeStart = dataGPS(stepStart,1);
timeEnd = dataGPS(stepEnd,1);
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

% figure
% plot3(xbw2Hist(k,1),xbw2Hist(k,2),xbw2Hist(k,3)), hold on

%! images for the selected GPS interval

dataFrame = importdata('Jul3120142100_CrystalLakeBoat\framedata',',',1);
dataFrame = dataFrame.data;

imageStepStart = min(find(dataFrame(:,1) > timeStart));
imageStepEnd = min(find(dataFrame(:,1) > timeEnd));
imageStart = dataFrame(imageStepStart,2) %! image 1751
imageEnd = dataFrame(imageStepEnd,2) %! 3001

%% Crstal Lake towards East data
for east = 1:1
% dataFrame = importdata('Jul1920140545_CrystalLakeEast\framedata',',',1);
% dataFrame = dataFrame.data;
% 
% %! images for the selected GPS interval
% imageStepStart = min(find(dataFrame(:,1) > timeStart));
% imageStepEnd = min(find(dataFrame(:,1) > timeEnd));
% imageStart = dataFrame(imageStepStart,2) %! image 2330
% imageEnd = dataFrame(imageStepEnd,2) %! 2942
% 
% %! quadcopter left the deck at image 2000
% timeLeave = dataFrame(find(dataFrame(:,2) == 2000),1);
% min(find(dataGPS(:,1) > timeLeave)) %! 2740
% 
% %! quadcopter started to fly forward at image 2300
% timeForward = dataFrame(find(dataFrame(:,2) == 2300),1);
% min(find(dataGPS(:,1) > timeForward)) %! 3159
% 
% %! quadcopter flew over the boats at image 2400
% timeBoat = dataFrame(find(dataFrame(:,2) == 2400),1);
% min(find(dataGPS(:,1) > timeBoat)) %! 3316
% 
% %! quadcopter started to turn back at image 2600
% timeTurn = dataFrame(find(dataFrame(:,2) == 2600),1);
% min(find(dataGPS(:,1) > timeTurn)) %! 3588
% 
% %! quadcopter started to come back at image 2700
% timeBack = dataFrame(find(dataFrame(:,2) == 2700),1);
% min(find(dataGPS(:,1) > timeBack)) %! 3741
% 
% %! quadcopter started to come back at image 2850
% timeBoat2 = dataFrame(find(dataFrame(:,2) == 2850),1);
% min(find(dataGPS(:,1) > timeBoat2)) %! 3966
% 
% %! quadcopter started to turn forward at image 2940
% timeTurn2 = dataFrame(find(dataFrame(:,2) == 2940),1);
% min(find(dataGPS(:,1) > timeTurn2)) %! 4098
% 
% %! quadcopter started to come down at image 3000
% timeDown = dataFrame(find(dataFrame(:,2) == 3000),1);
% min(find(dataGPS(:,1) > timeDown)) %! 4194
% 
% %! quadcopter started to fly forward again at image 3250
% timeForward2 = dataFrame(find(dataFrame(:,2) == 3250),1);
% min(find(dataGPS(:,1) > timeForward2)) %! 4553
% 
% %! quadcopter started to turn back again at image 3530
% timeTurn2 = dataFrame(find(dataFrame(:,2) == 3530),1);
% min(find(dataGPS(:,1) > timeTurn2)) %! 4940
% 
% %! quadcopter started to come back again at image 3600
% timeBack2 = dataFrame(find(dataFrame(:,2) == 3600),1);
% min(find(dataGPS(:,1) > timeBack2)) %! 5037
% 
% %! quadcopter dropped into the water at image 3880
% timeDrop = dataFrame(find(dataFrame(:,2) == 3880),1);
% min(find(dataGPS(:,1) > timeDrop)) %! 5442
% 
% %! quadcopter arrived to the deck at image 4140
% timeArrive = dataFrame(find(dataFrame(:,2) == 4140),1);
% min(find(dataGPS(:,1) > timeArrive)) %! 5849
% 
% %! quadcopter landed at image 4200
% timeLand = dataFrame(find(dataFrame(:,2) == 4200),1);
% min(find(dataGPS(:,1) > timeLand)) %! 5948
end