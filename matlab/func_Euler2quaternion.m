function q = func_Euler2quaternion( r )

% based on the quaternion to Euler angles wiki page

    q = [sin(r(1)/2)*cos(r(2)/2)*cos(r(3)/2) - cos(r(1)/2)*sin(r(2)/2)*sin(r(3)/2)
        cos(r(1)/2)*sin(r(2)/2)*cos(r(3)/2) + sin(r(1)/2)*cos(r(2)/2)*sin(r(3)/2)
        cos(r(1)/2)*cos(r(2)/2)*sin(r(3)/2) - sin(r(1)/2)*sin(r(2)/2)*cos(r(3)/2)
        cos(r(1)/2)*cos(r(2)/2)*cos(r(3)/2) + sin(r(1)/2)*sin(r(2)/2)*sin(r(3)/2)];
end
   
