clear all;
close all;
clc;
plotFlag = 0;
flagBias = 1;

%! load experimental data (IMU, altimeter)
load('hb/backup71_C/data_experiment/2ndStreet/data.mat');

altHist = altHist + 0.05*ones(size(altHist));

%! load experimental data (vision: 1700~4340 automatic reflection and shore feautres)
load('hb/backup71_C/data_experiment/2ndStreet/vision_feat5_ref5.mat');
noise = zeros(1+6*50,stepEnd);

tic

%% state initialization
mu = zeros(6,1);
mu(3) = -altHist(:,stepStart);

d0 = 1;
for i = 1:nf
    mu(6+3*i-2:6+3*i,1) = [pibHist(1,stepStart,i); pibHist(2,stepStart,i); 1/d0];
end

%! covariance initialization
P0 = 1;
Q0 = 1;
R0 = 10;

P = eye(6+3*nf);
P(1:3,1:3) = 10^(-4)*eye(3);                %! UAV location
P(6+3:3:6+3*nf,6+3:3:6+3*nf) = P0*eye(nf);  %! inverse depth

j = 1;
time = 0;
d_init = 5;

if flagBias == 1
    mu(6+3*nf+1:6+3*nf+3,1) = zeros(3,1);
    P(6+3*nf+1:6+3*nf+3,6+3*nf+1:6+3*nf+3) = 10^(-4)*eye(3);
end

%%
for k = stepStart:stepEnd
    
    %! computing the time
    dt = dtHist(k,1);
    time = time+dt;     tHist(k,1) = time;
    
    %! reading sensor measurements
    qbw = qbwHist(:,k);    

    %qbw = qbw/norm(qbw); %% Remove norm?
     
    Rb2w = func_quaternion2Rotation(qbw);       Rw2b = Rb2w';

    w = wHist(:,k);
    a = aHist(:,k);
    
    qbwHist(:,k) = qbw;
    wHist(:,k) = w;
    aHist(:,k) = a;
    
    for i = 1:nf
        r = func_quaternion2Euler(qbw)*180/pi;
        pibr = atan2(pibHist(2,k,i),1)*180/pi + r;
        d0 = -altHist(1,k)/sin(pibr(2)/180*pi)*2;
        if (d0 > d_init)
            d0 = d_init;
        end
        
        d0Hist(k,i) = 1/mu(6+3*i,1);
        
        %! experiment: renew elements are piecewise constant
        renewZero = renewHist(k-1,i);
        renewZero2 = renewHist(k,i);
        
        %! updating new features
        if k == stepStart || renewHist(k,i) ~= renewZero
            %! current signature that exist before
            [renewRow renewCol] = find(renewHist == renewHist(k,i));
            renewk = max(renewRow(find(renewRow < k)));

            %! if current signature existed before
            if isempty(renewk) == 0 && k <= 4340
                renewi = renewCol(find(renewRow == renewk));
                d0 = d0Hist(renewk,renewi);
                
            end

            
            %! location and orientation of each anchor
            xb0wHat(:,i) = mu(1:3);
            qb0w(:,i) = qbw;
            Rw2b0(:,:,i) = func_quaternion2Rotation(qb0w(:,i))';
            pib0(:,i) = pibHist(1:2,k,i);
            j = j+1;
            
            %! re-initialize the state for a new feature
            mu(6+3*i-2:6+3*i,1) = [pibHist(1,k,i); pibHist(2,k,i); 1/d0];
        end
        
        %size(Rw2b0)
        xb0wHatHist(k,:,i) = xb0wHat(:,i)';
        
        %! position of the feature w.r.t. the anchor
        xbb0Hat(:,i) = Rw2b0(:,:,i)*(mu(1:3) - xb0wHat(:,i));
        Rb2b0(:,:,i) = Rw2b0(:,:,i)*Rb2w;

        
        %! setting max and min depth
        d_max = 15;
        d_min = 0.5;
        
        if mu(6+3*i) < 1/d_max
            mu(6+3*i) = 1/d_max;
        elseif mu(6+3*i) > 1/d_min
            mu(6+3*i) = 1/d_min;
        end
        
        %! leaving the final estimate of each feature's location
        if (k < stepEnd && renewHist(k+1,i) ~= renewZero2) || k == stepEnd
            xiwHatHist(:,i,j) = xiwHat(:,i);
            
            %! removing bad features
            if mu(6+3*i) < 1/10 || mu(6+3*i) > 1/d_min
                xiwHatHist(:,i,j) = zeros(3,1);
            end
        end
    end
    
    %! saving the history of the estimates
    
    muHist(:,k) = mu;
    for i = 1:size(mu)
        PHist(i,k) = sqrt(P(i,i));
    end
    
    %! EKF prediction
    for i = 1:nf
        pibHat(1,i) = mu(6+3*i-2);
        pibHat(2,i) = mu(6+3*i-1);
        pibHat(3,i) = mu(6+3*i-0);
    end
    
    %! motion model
    [f F] = func_motionModel( mu(1:6+3*nf), qbw, a, w, pibHat, nf, dt);
    
    if flagBias == 1
        F(6+3*nf+1:6+3*nf+3,6+3*nf+1:6+3*nf+3) = eye(3);
        F(4:6,6+3*nf+1:6+3*nf+3) = -eye(3)*dt;
        abiasHat = mu(6+3*nf+1:6+3*nf+3,1);
        f(4:6,1) = f(4:6,1) - abiasHat;
        f(6+3*nf+1:6+3*nf+3,1) = zeros(3,1);
    end
    mu = mu + f*dt;

    %! measurement model
    [meas hmu H pibHat xiwHat] = func_measurementModel( k, nf, altHist(k), pibHist, pib0, ppbHist, mu(1:6+3*nf,1), qbw, xb0wHat, xbb0Hat, qb0w, Rb2b0, refFlag(k,:), 0 );
    if flagBias == 1
        H(1:size(H,1),6+3*nf+1:6+3*nf+3) = zeros(size(H,1),3);
    end
    measHist(1:length(meas),k) = meas;
    altHist(k) = meas(1);
    
    
    %! noise covariance
    G = eye(6+3*nf)*dt;     G(1:3,1:3) = eye(3)*1/2*dt^2;
    if flagBias == 1
        G(6+3*nf+1:6+3*nf+3,6+3*nf+1:6+3*nf+3) = eye(3)*1/2*dt^2;
    end
    
    %! for the 2nd street data set
    if k > stepStart && altHist(k)-altHist(k-1) < -0.6
        Q0 = 20;
    end
    
    Q = Q0*eye(length(mu));
    if flagBias == 1
        Q(6+3*nf+1:6+3*nf+3,6+3*nf+1:6+3*nf+3) = 0.002*eye(3);
    end
    R = 0.1/770*R0*eye(length(meas));
    
    %! altimeter noise covariance
    R(1) = 0.0001*R0;
    for i = 1:nf
        %! current view measurement noise covariance
        R(1+6*i-5:1+6*i-4, 1+6*i-5:1+6*i-4) = 0.1/770*R0*eye(2);
        
        %! initial view measurement noise covariance
        R(1+6*i-3:1+6*i-2, 1+6*i-3:1+6*i-2) = 10/770*R0*eye(2);
        
        %! reflection measurment noise covariance
        R(1+6*i-1:1+6*i-0, 1+6*i-1:1+6*i-0) = 10/770*R0*eye(2);
    end
    
    %! EKF measurement update
    P = F*P*F'+G*Q*G';
    K = P*H'*inv(H*P*H'+R);
  
    mu = mu + K*(meas-hmu);
    P = (eye(size(K*H))-K*H)*P;	P = (P'+P)/2;
    

    %! displaying time-step
    if rem(k,300) == 0
        k
    end
    
end

toc

k = stepStart:stepEnd;  t = (k-1)*dt;

if plotFlag == 1
    figure
    ax = axes;
    plot3(muHist(1,:),muHist(2,:),muHist(3,:),'Color',[0.8,0.2,0.2],'linewidth',3); hold on
    for i = 1:1:size(xiwHatHist,2)
        for j = 1:1:size(xiwHatHist,3)
            if xiwHatHist(1,i,j) ~= zeros(3,1)
                plot3(xiwHatHist(1,i,j),xiwHatHist(2,i,j),xiwHatHist(3,i,j),...
                    'o','MarkerSize',4,'MarkerEdgeColor',[0.3+0.2*rand,0.5,0.75+i/4/size(xiwHatHist,2)],'MarkerFaceColor',[0.3+0.2*rand,0.5,0.75+i/4/size(xiwHatHist,2)]); hold on
            end
        end
    end
    plot3(muHist(1,stepEnd),muHist(2,stepEnd),muHist(3,stepEnd),'s','Color',[0.8,0.2,0.2],'linewidth',1.5); hold on
    grid on; view(2); set(ax,'Ydir','rev','Zdir','rev'); axis equal;
    
    view([-90 90])
    
    figure
    plot(k,altHist(k))
end