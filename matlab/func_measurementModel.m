function [meas hmu H pibHat xiwHat] = func_measurementModel( k, nf, alt, pibHist, pib0, ppbHist, mu, qbw, xb0wHat, xbb0Hat, qb0w, Rb2b0, refFlag, flagMeas )
    
n = [0;0;1];	S = eye(3)-2*n*n';
Rb2w = func_quaternion2Rotation(qbw);	Rw2b = Rb2w';

for i = 1:nf
    pibHat(1,i) = mu(6+3*i-2); 
    pibHat(2,i) = mu(6+3*i-1);
    pibHat(3,i) = mu(6+3*i-0);

    xibHat(:,i) = [1/pibHat(3,i); pibHat(1,i)/pibHat(3,i); pibHat(2,i)/pibHat(3,i)];
    xib0Hat(:,i) = xbb0Hat(:,i) + Rb2b0(:,:,i)*xibHat(:,i);
    pib0Hat(:,i) = [xib0Hat(2,i)/xib0Hat(1,i); xib0Hat(3,i)/xib0Hat(1,i)];

    xpbHat(:,i) = Rw2b*(S*Rb2w*xibHat(:,i)-2*n*n'*mu(1:3));
    ppbHat(:,i) = [xpbHat(2,i)/xpbHat(1,i); xpbHat(3,i)/xpbHat(1,i)];

    xiwHat(:,i) = mu(1:3) + Rb2w*xibHat(:,i);  
    [Hb Hi] = func_JacobianH( mu, qbw, xb0wHat(:,i), qb0w(:,i), i);
    
    %! 0: all;  
    if flagMeas == 0
        meas(1,1) = alt;                 	%! altitude
        meas(1+6*i-5,1) = pibHist(1,k,i);	%! current view
        meas(1+6*i-4,1) = pibHist(2,k,i);
        meas(1+6*i-3,1) = pib0(1,i);        %! initial view
        meas(1+6*i-2,1) = pib0(2,i);
        meas(1+6*i-1,1) = ppbHist(1,k,i);   %! reflection
        meas(1+6*i-0,1) = ppbHist(2,k,i);
        hmu(1,1) = -mu(3);
        hmu(1+6*i-5,1) = pibHat(1,i);
        hmu(1+6*i-4,1) = pibHat(2,i);
        hmu(1+6*i-3,1) = pib0Hat(1,i);
        hmu(1+6*i-2,1) = pib0Hat(2,i);
        hmu(1+6*i-1,1) = ppbHat(1,i);
        hmu(1+6*i-0,1) = ppbHat(2,i);
        H(1,:) = [0 0 -1 zeros(1,3+3*nf)];
        H(1+6*i-5,:) = [zeros(1,6) zeros(1,3*(i-1)) [1 0 0] zeros(1,3*(nf-i))];
        H(1+6*i-4,:) = [zeros(1,6) zeros(1,3*(i-1)) [0 1 0] zeros(1,3*(nf-i))];
        H(1+6*i-3,:) = [Hb(1,:) zeros(1,3*(i-1)) Hi(1,:) zeros(1,3*(nf-i))];
        H(1+6*i-2,:) = [Hb(2,:) zeros(1,3*(i-1)) Hi(2,:) zeros(1,3*(nf-i))];
        H(1+6*i-1,:) = [Hb(3,:) zeros(1,3*(i-1)) Hi(3,:) zeros(1,3*(nf-i))];
        H(1+6*i-0,:) = [Hb(4,:) zeros(1,3*(i-1)) Hi(4,:) zeros(1,3*(nf-i))];
        size(H)
        H
        pause
        str=sprintf('data/varout/var%d%d.hex',k,i);
        matlab2txt(H,str,'w');
        %! features that don't have reflections
        if refFlag(1,i) == 0	
            meas(1+6*i-1,1) = 0;
            meas(1+6*i-0,1) = 0;
%             meas(1+6*i-3,1) = 0;
%             meas(1+6*i-2,1) = 0;
        end
    %! 1: w/o altitude;    
    elseif flagMeas == 1
        meas(6*i-5,1) = pibHist(1,k,i);     %! current view
        meas(6*i-4,1) = pibHist(2,k,i);
        meas(6*i-3,1) = pib0(1,i);          %! initial view
        meas(6*i-2,1) = pib0(2,i);
        meas(6*i-1,1) = ppbHist(1,k,i);     %! reflection
        meas(6*i-0,1) = ppbHist(2,k,i);
        hmu(6*i-5,1) = pibHat(1,i);
        hmu(6*i-4,1) = pibHat(2,i);
        hmu(6*i-3,1) = pib0Hat(1,i);
        hmu(6*i-2,1) = pib0Hat(2,i);
        hmu(6*i-1,1) = ppbHat(1,i);
        hmu(6*i-0,1) = ppbHat(2,i);
        H(6*i-5,:) = [zeros(1,6) zeros(1,3*(i-1)) [1 0 0] zeros(1,3*(nf-i))];
        H(6*i-4,:) = [zeros(1,6) zeros(1,3*(i-1)) [0 1 0] zeros(1,3*(nf-i))];
        H(6*i-3,:) = [Hb(1,:) zeros(1,3*(i-1)) Hi(1,:) zeros(1,3*(nf-i))];
        H(6*i-2,:) = [Hb(2,:) zeros(1,3*(i-1)) Hi(2,:) zeros(1,3*(nf-i))];
        H(6*i-1,:) = [Hb(3,:) zeros(1,3*(i-1)) Hi(3,:) zeros(1,3*(nf-i))];
        H(6*i-0,:) = [Hb(4,:) zeros(1,3*(i-1)) Hi(4,:) zeros(1,3*(nf-i))];

        %! features that don't have reflections
        if refFlag(1,i) == 0	
            meas(6*i-1,1) = 0;
            meas(6*i-0,1) = 0;
        end
        
    %! 2: w/o reflection;  
    elseif flagMeas == 2
        meas(1,1) = alt;                 	%! altitude
        meas(1+4*i-3,1) = pibHist(1,k,i);	%! current view
        meas(1+4*i-2,1) = pibHist(2,k,i);
        meas(1+4*i-1,1) = pib0(1,i);        %! initial view
        meas(1+4*i-0,1) = pib0(2,i);
        hmu(1,1) = -mu(3);
        hmu(1+4*i-3,1) = pibHat(1,i);
        hmu(1+4*i-2,1) = pibHat(2,i);
        hmu(1+4*i-1,1) = pib0Hat(1,i);
        hmu(1+4*i-0,1) = pib0Hat(2,i);
        H(1,:) = [0 0 -1 zeros(1,3+3*nf)];
        H(1+4*i-3,:) = [zeros(1,6) zeros(1,3*(i-1)) [1 0 0] zeros(1,3*(nf-i))];
        H(1+4*i-2,:) = [zeros(1,6) zeros(1,3*(i-1)) [0 1 0] zeros(1,3*(nf-i))];
        H(1+4*i-1,:) = [Hb(1,:) zeros(1,3*(i-1)) Hi(1,:) zeros(1,3*(nf-i))];
        H(1+4*i-0,:) = [Hb(2,:) zeros(1,3*(i-1)) Hi(2,:) zeros(1,3*(nf-i))];
        
    %! 3: w/o altitude and reflection; 
    elseif flagMeas == 3
        meas(4*i-3,1) = pibHist(1,k,i);     %! current view
        meas(4*i-2,1) = pibHist(2,k,i);
        meas(4*i-1,1) = pib0(1,i);          %! initial view
        meas(4*i-0,1) = pib0(2,i);
        hmu(4*i-3,1) = pibHat(1,i);
        hmu(4*i-2,1) = pibHat(2,i);
        hmu(4*i-1,1) = pib0Hat(1,i);
        hmu(4*i-0,1) = pib0Hat(2,i);
        H(4*i-3,:) = [zeros(1,6) zeros(1,3*(i-1)) [1 0 0] zeros(1,3*(nf-i))];
        H(4*i-2,:) = [zeros(1,6) zeros(1,3*(i-1)) [0 1 0] zeros(1,3*(nf-i))];
        H(4*i-1,:) = [Hb(1,:) zeros(1,3*(i-1)) Hi(1,:) zeros(1,3*(nf-i))];
        H(4*i-0,:) = [Hb(2,:) zeros(1,3*(i-1)) Hi(2,:) zeros(1,3*(nf-i))];
        
    %! 4: w/o initial view
    elseif flagMeas == 4
        meas(1,1) = alt;                 	%! altitude
        meas(1+4*i-3,1) = pibHist(1,k,i);	%! current view
        meas(1+4*i-2,1) = pibHist(2,k,i);
        meas(1+4*i-1,1) = ppbHist(1,k,i);   %! reflection
        meas(1+4*i-0,1) = ppbHist(2,k,i);
        hmu(1,1) = -mu(3);
        hmu(1+4*i-3,1) = pibHat(1,i);
        hmu(1+4*i-2,1) = pibHat(2,i);
        hmu(1+4*i-1,1) = ppbHat(1,i);
        hmu(1+4*i-0,1) = ppbHat(2,i);
        H(1,:) = [0 0 -1 zeros(1,3+3*nf)];
        H(1+4*i-3,:) = [zeros(1,6) zeros(1,3*(i-1)) [1 0 0] zeros(1,3*(nf-i))];
        H(1+4*i-2,:) = [zeros(1,6) zeros(1,3*(i-1)) [0 1 0] zeros(1,3*(nf-i))];
        H(1+4*i-1,:) = [Hb(3,:) zeros(1,3*(i-1)) Hi(3,:) zeros(1,3*(nf-i))];
        H(1+4*i-0,:) = [Hb(4,:) zeros(1,3*(i-1)) Hi(4,:) zeros(1,3*(nf-i))];
        
        %! features that don't have reflections
        if refFlag(1,i) == 0	
            meas(1+4*i-1,1) = 0;
            meas(1+4*i-0,1) = 0;
        end
    end
end

%! remove the reflection measruements if they are not available
% hmu = hmu(~ismember(1:size(hmu, 1), find(meas == 0)), :);
% H = H(~ismember(1:size(H, 1), find(meas == 0)), :);
% meas = meas(~ismember(1:size(meas, 1), find(meas == 0)), :);

