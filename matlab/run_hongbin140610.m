clear all;
close all;
clc; 

%! load experimental data (vision: 1700~4340 multiple non-reflection features for mapping)
load('data_experiment/2ndStreet/vision_feat50.mat');
nfMap = nf; 
% nfMap = 10
pibHistMap = pibHist(:,:,1:nfMap);
ppbHistMap = ppbHist(:,:,1:nfMap);
renewHistMap = renewHist(:,1:nfMap);
clearvars -except nfMap pibHistMap ppbHistMap renewHistMap

%! load experimental data (IMU, altimeter)
load('data_experiment/2ndStreet/data.mat');
altHist = altHist + 0.05*ones(1,length(altHist));

%! load experimental data (vision: 1700~4340 automatic reflection and shore features)
load('data_experiment/2ndStreet/vision_feat5_ref5.mat');
noise = zeros(1+6*50,stepEnd);        

flagBias = 1;
flagMap = 1;

%% estimate initialization

warning('off')
tic

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
d_max = 30;
d_min = 0.3;
xiwHat = zeros(3,nf);
xiwHatMap = zeros(3,nfMap);
xiwHatHist = zeros(3,nf,stepEnd);
xiwHatHistMap = zeros(3,nfMap,stepEnd);

if flagBias == 1
    mu(6+3*nf+1:6+3*nf+3,1) = zeros(3,1);
    P(6+3*nf+1:6+3*nf+3,6+3*nf+1:6+3*nf+3) = 10^(-4)*eye(3);
end

%% estimate initialization for mapping

if flagMap == 1
    d0 = 1;
    for i = 1:nfMap
        muMap(:,i) = [pibHistMap(1,stepStart,i); pibHistMap(2,stepStart,i); 1/d0];
        PMap(:,:,i) = eye(3);
        PMap(3,3,i) = P0;  %! inverse depth
    end

    %! process noise covariance
    QMap = Q0*eye(3);

    %! current / initial view measurement noise covariance
    RMap(1:2, 1:2) = 0.1/770*R0*eye(2);
    RMap(3:4, 3:4) = 10/770*R0*eye(2);

    jMap = 1;
    ibad = zeros(1,nfMap);
end

%% time-step for each vision data

for k = stepStart:stepEnd    
    
    %! computing the time
    dt = dtHist(k,1);   Hz = 1/dt;
    time = time+dt;     tHist(k,1) = time;
    
    %! reading sensor measurements
    qbw = qbwHist(:,k);    qbw = qbw/norm(qbw);
    Rb2w = func_quaternion2Rotation(qbw);       Rw2b = Rb2w';
    w = wHist(:,k);
    a = aHist(:,k);
    
    qbwHist(:,k) = qbw;
    wHist(:,k) = w;
    aHist(:,k) = a;
    
    %% initialization of reflection features
    
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
            
            %! re-initialize the state for a new feature
            mu(6+3*i-2:6+3*i,1) = [pibHist(1,k,i); pibHist(2,k,i); 1/d0];
        end
        xb0wHatHist(k,:,i) = xb0wHat(:,i)';
        
        %! position of the feature w.r.t. the anchor
        xbb0Hat(:,i) = Rw2b0(:,:,i)*(mu(1:3) - xb0wHat(:,i));
        Rb2b0(:,:,i) = Rw2b0(:,:,i)*Rb2w;
        
%         %! setting max and min depth
%         if mu(6+3*i) < 1/d_max
%             mu(6+3*i) = 1/d_max;
%         elseif mu(6+3*i) > 1/d_min
%             mu(6+3*i) = 1/d_min;
%         end

        %! leaving the final estimate of each relevant feature's location
        if renewHist(k,i) == renewZero || k == stepEnd
            if mu(6+3*i) > 1/d_max && mu(6+3*i) < 1/d_min
                xiwHatHist(:,i,renewHist(k,i)) = xiwHat(:,i);
            end
        end
    end
    
    %% initialization of non-reflection features
    
    if flagMap == 1
        for i = 1:nfMap
            r = func_quaternion2Euler(qbw)*180/pi;
            pibr = atan2(pibHistMap(2,k,i),1)*180/pi + r;
            d0 = -altHist(1,k)/sin(pibr(2)/180*pi)*2;
            if (d0 > d_init)
                d0 = d_init;
            end       

            d0HistMap(k,i) = 1/muMap(3,i);

            %! experiment: renew elements are piecewise constant
            renewZeroMap = renewHistMap(k-1,i);

            %! updating new features
            if k == stepStart || renewHistMap(k,i) ~= renewZeroMap
                %! current signature that exist before
                [renewRow renewCol] = find(renewHistMap == renewHistMap(k,i));
                renewkMap = max(renewRow(find(renewRow < k)));

                %! if current signature existed before
                if isempty(renewkMap) == 0 && k <= 4340
                    renewiMap = renewCol(find(renewRow == renewkMap));
                    d0 = d0HistMap(renewkMap,renewiMap);
                end

                %! location and orientation of each anchor
                xb0wHatMap(:,i) = mu(1:3);
                qb0wMap(:,i) = qbw;
                Rw2b0Map(:,:,i) = func_quaternion2Rotation(qb0wMap(:,i))';
                pib0Map(:,i) = pibHistMap(1:2,k,i);

                %! re-initialize the state for a new feature
                muMap(:,i) = [pibHistMap(1,k,i); pibHistMap(2,k,i); 1/d0];
            end
            xb0wHatHistMap(k,:,i) = xb0wHatMap(:,i)';

            %! position of the feature w.r.t. the anchor
            xbb0HatMap(:,i) = Rw2b0Map(:,:,i)*(mu(1:3) - xb0wHatMap(:,i));
            Rb2b0Map(:,:,i) = Rw2b0Map(:,:,i)*Rb2w;
            
%             %! setting max and min depth 
%             if muMap(3,i) < 1/d_max
%                 muMap(3,i) = 1/d_max;
%             elseif muMap(3,i) > 1/d_min
%                 muMap(3,i) = 1/d_min;
%             end

            %! leaving the final estimate of each relevant feature's location
            if renewHistMap(k,i) == renewZeroMap || k == stepEnd
                if muMap(3,i) > 1/d_max && muMap(3,i) < 1/d_min
                    xiwHatHistMap(:,i,renewHistMap(k,i)) = xiwHatMap(:,i);
                end
            end
        end
    end
    
	%% SLAM w/ reflection features
    
	%! saving the history of the estimates
    muHist(:,k) = mu;
    for i = 1:length(mu)
        PHist(i,k) = sqrt(P(i,i));
    end
    
    %! motion model
    for i = 1:nf
        pibHat(1,i) = mu(6+3*i-2); 
        pibHat(2,i) = mu(6+3*i-1);
        pibHat(3,i) = mu(6+3*i-0);
    end    
    
    [f F] = func_motionModel( mu(1:6+3*nf), qbw, a, w, pibHat, nf, dt);
    if flagBias == 1
        F(6+3*nf+1:6+3*nf+3,6+3*nf+1:6+3*nf+3) = eye(3);
        F(4:6,6+3*nf+1:6+3*nf+3) = -eye(3)*dt;
        abiasHat = mu(6+3*nf+1:6+3*nf+3,1);
        f(4:6,1) = f(4:6,1) - abiasHat;
        f(6+3*nf+1:6+3*nf+3,1) = zeros(3,1);
    end
    
    %! process noise covariance
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
    
    %! EKF prediction
    mu = mu + f*dt;
    muPred = mu; 
    P = F*P*F'+G*Q*G';
    
    %! measurement model
    [meas hmu H pibHat xiwHat] = func_measurementModel( k, nf, altHist(k), pibHist, pib0, ppbHist, mu(1:6+3*nf,1), qbw, xb0wHat, xbb0Hat, qb0w, Rb2b0, refFlag(k,:), 0 );
    if flagBias == 1
        H(1:size(H,1),6+3*nf+1:6+3*nf+3) = zeros(size(H,1),3);
    end
%     measHist(1:length(meas),k) = meas;
    
    %! measurement noise covariance
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
    K = P*H'*inv(H*P*H'+R);
    mu = mu + K*(meas-hmu);    
    P = (eye(size(K*H))-K*H)*P;	P = (P'+P)/2;
    
    %% mapping of non-reflection features
    
    if flagMap == 1
        %! saving the history of the estimates
        muHistMap(:,:,k) = muMap;
        for ii = 1:3
            for i = 1:length(muMap)
                PHistMap(ii,k,i) = sqrt(PMap(ii,ii));
            end
        end

        %! motion model
        pibHatMap = muMap; 
        [fMap FMap] = func_motionModelMap( mu(1:6+3*nf), w, pibHatMap, nfMap, dt);

        %! EKF prediction
        for i = 1:nfMap
            muMap(:,i) = muMap(:,i) + fMap(:,i)*dt;
        end
        PMap(:,:,i) = FMap(:,:,i)*PMap(:,:,i)*FMap(:,:,i)' + QMap*dt^2;

        %! measurement model
        [measMap hmuMap HMap pibHatMap xiwHatMap] = func_measurementModelMap( k, nfMap, pibHistMap, pib0Map, mu, qbw, xb0wHatMap, xbb0HatMap, qb0wMap, Rb2b0Map, muMap);
        measMap = measMap(1:2,:);
        hmuMap = hmuMap(1:2,:);
        HMap = HMap(1:2,:,:);
        RMap = RMap(1:2,1:2);
%         measHistMap(:,k,:) = measMap;

        for i = 1:nfMap   
            %! EKF measurement update
            KMap = PMap(:,:,i)*HMap(:,:,i)'*inv(HMap(:,:,i)*PMap(:,:,i)*HMap(:,:,i)' + RMap);
            muMap(:,i) = muMap(:,i) + KMap*(measMap(:,i) - hmuMap(:,i));
            PMap(:,:,i) = (eye(3) - KMap*HMap(:,:,i))*PMap(:,:,i);     PMap(:,:,i) = (PMap(:,:,i)' + PMap(:,:,i))/2;

        end
    end
    
    %% displaying time-step
    if rem(k,300) == 0
        k
    end
end

time
toc

%% plotting the results
figure
k = stepStart:stepEnd;  t = (k-1)*dt;
ax = axes;
plot3(muHist(1,:),muHist(2,:),muHist(3,:),'Color',[0.8,0.2,0.2],'linewidth',3); hold on
for i = 1:size(xiwHatHist,2)
    for j = unique(sort(renewHist(:,i),'ascend'))'
        if j > 0 && xiwHatHist(1,i,j) ~= 0
            plot3(xiwHatHist(1,i,j),xiwHatHist(2,i,j),xiwHatHist(3,i,j),...
                'o','MarkerSize',4,'MarkerEdgeColor',[0.3+0.2*rand,0.5,0.75+i/4/size(xiwHatHist,2)],'MarkerFaceColor',[0.3+0.2*rand,0.5,0.75+i/4/size(xiwHatHist,2)]); hold on
        end
    end
end
if flagMap == 1
    for i = 1:size(xiwHatHistMap,2)
        for j = unique(sort(renewHistMap(:,i),'ascend'))'
            if j > 0 && xiwHatHistMap(1,i,j) ~= 0
                plot3(xiwHatHistMap(1,i,j),xiwHatHistMap(2,i,j),xiwHatHistMap(3,i,j),...
                    'o','MarkerSize',4,'MarkerEdgeColor',[0.3+0.2*rand,0.5,0.75+i/4/size(xiwHatHistMap,2)],'MarkerFaceColor',[0.3+0.2*rand,0.5,0.75+i/4/size(xiwHatHistMap,2)]); hold on
            end
        end
    end
end
plot3(muHist(1,stepEnd),muHist(2,stepEnd),muHist(3,stepEnd),'s','Color',[0.8,0.2,0.2],'linewidth',1.5); hold on
grid on; view(2); set(ax,'Ydir','rev','Zdir','rev'); axis equal; 
view([-90 90])
ylim([-40 20]), xlim([-30 20])