function [f F] = func_motionModel( mu, qbw, a, w, pibHat, nf, dt)

%! rename variables for better legibility
v1 = mu(4); v2 = mu(5); v3 = mu(6); w1 = w(1); w2 = w(2); w3 = w(3);
qbw1 = qbw(1); qbw2 = qbw(2); qbw3 = qbw(3); qbw4 = qbw(4);
gw = [0;0;-9.80665];

Rb2w = func_quaternion2Rotation(qbw);   Rw2b = Rb2w';
A = [0 -w3 w2; w3 0 -w1; -w2 w1 0];

f = [Rb2w*mu(4:6);              % UAS location
    
    -A*mu(4:6) + a - Rw2b*gw];	% linear velocity 
Fb = [...
[ 0, 0, 0, qbw1^2 - qbw2^2 - qbw3^2 + qbw4^2,           2*qbw1*qbw2 - 2*qbw3*qbw4,           2*qbw1*qbw3 + 2*qbw2*qbw4]
[ 0, 0, 0,         2*qbw1*qbw2 + 2*qbw3*qbw4, - qbw1^2 + qbw2^2 - qbw3^2 + qbw4^2,           2*qbw2*qbw3 - 2*qbw1*qbw4]
[ 0, 0, 0,         2*qbw1*qbw3 - 2*qbw2*qbw4,           2*qbw1*qbw4 + 2*qbw2*qbw3, - qbw1^2 - qbw2^2 + qbw3^2 + qbw4^2]
[ 0, 0, 0,                                 0,                                  w3,                                 -w2]
[ 0, 0, 0,                               -w3,                                   0,                                  w1]
[ 0, 0, 0,                                w2,                                 -w1,                                   0]];


for i = 1:nf
    pib1 = pibHat(1,i);	pib2 = pibHat(2,i); pib3 = pibHat(3,i);


    
    fi = [...
    (-v2+pib1*v1)*pib3 + pib2*w1 - (1+pib1^2)*w3 + pib1*pib2*w2;
    (-v3+pib2*v1)*pib3 - pib1*w1 + (1+pib2^2)*w2 - pib1*pib2*w3;
    (-w3*pib1+w2*pib2)*pib3 + v1*pib3^2];


    FiTemp = [...
    [ pib3*v3 - 2*pib1*w2 + pib2*w1,                  w3 + pib1*w1,                  pib1*v3 - v1]
    [                - w3 - pib2*w2, pib3*v3 - pib1*w2 + 2*pib2*w1,                  pib2*v3 - v2]
    [                      -pib3*w2,                       pib3*w1, 2*pib3*v3 - pib1*w2 + pib2*w1]];

    Fib(3*i-2:3*i,:) = [...
    [ 0, 0, 0, -pib3,     0, pib1*pib3]
    [ 0, 0, 0,     0, -pib3, pib2*pib3]
    [ 0, 0, 0,     0,     0,    pib3^2]];

    Fi(3*i-2:3*i,:) = [zeros(3,3*(i-1)) FiTemp zeros(3,3*(nf-i))];
    
    f(6+3*i-2:6+3*i,1) = fi;    % including features
end


F = eye(size(mu,1)) + [Fb zeros(6,3*nf); Fib Fi]*dt;    %! Jacobian of (mu + f*dt)


% size(F)

