function r = func_quaternion2Euler( q )

% based on the quaternion to Euler angles wiki page

    r = [atan2(2*q(2)*q(3)+2*q(4)*q(1),q(3)^2-q(2)^2-q(1)^2+q(4)^2);
        -asin(2*q(1)*q(3)-2*q(4)*q(2));
        atan2(2*q(1)*q(2)+2*q(4)*q(3),q(1)^2+q(4)^2-q(3)^2-q(2)^2)];
end
   
