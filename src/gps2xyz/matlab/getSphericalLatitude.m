function [ sphericalLat ] = getSphericalLatitude( lat, h )
%GETSPHERICALLATITUDE Summary of this function goes here
%   lat = GPS_LAT
%   h = Altitude

PI = pi;
% semi-major axis
A = 6378137; 

% 1/f reciprocal of flattening
f = 1/298.257223563;

e2 = f*(2-f);
Rc = A / (sqrt(1 - (e2 ^ 2) * (sin(lat * PI / 180) ^ 2)));
p = (Rc + h) * cos(lat * PI / 180);
z = (Rc * (1 - e2 ^ 2) + h) * sin(lat * PI / 180);
r = sqrt(p ^ 2 + z ^ 2);

sphericalLat = asin(z / r) * 180 / PI; 
end

