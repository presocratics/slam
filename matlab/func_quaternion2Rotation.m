function R = func_quaternion2Rotation( q )

% based on the quaternion to Euler angles wiki page

    R = [q(4)^2+q(1)^2-q(2)^2-q(3)^2     2*(q(1)*q(2)-q(4)*q(3))         2*(q(4)*q(2)+q(1)*q(3));
        2*(q(1)*q(2)+q(4)*q(3))         q(4)^2-q(1)^2+q(2)^2-q(3)^2     2*(q(2)*q(3)-q(4)*q(1));
        2*(q(1)*q(3)-q(4)*q(2))         2*(q(4)*q(1)+q(2)*q(3))         q(4)^2-q(1)^2-q(2)^2+q(3)^2];
end
   
