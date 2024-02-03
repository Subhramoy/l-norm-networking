function [estPosition,eul_angle,imu_closestIndex_xRobot,imu_closestIndex_yRobot,imu_STASnr_rndd,distanceToGoal,imu_closestIndex_xRobot_mrc,imu_closestIndex_yRobot_mrc]=ImuGpsFusion_path_mrc(i,Params,truePosition,trueVel,Rpos,Rvel,estPosition,count,snr_per_mcs_5GHz_indoor_channelB, AP_x,AP_y,imu_STASnr_rndd,gndFusion,accelData,gyroData,gps,imu_closestIndex_xRobot,imu_closestIndex_yRobot,imu_closestIndex_xRobot_mrc,imu_closestIndex_yRobot_mrc)

%% Define a set of waypoints for path #1

x1 = [-17 -15 -10 -6 -3 0 5 10 16];
y1 = [0 10 19 10 -10 -17 -22 -20 -17];
path_x1 = -17:0.5:16;
path_y1 = interp1(x1,y1,path_x1,'spline');
path_x1 = path_x1.';
path_y1 = path_y1.';
path = [path_x1 path_y1];

% Input parameters %
txPower = 25;               % In mW (dB)
opBW = 2e7;                 % Operational BW in MHz
NoisePower = (10.^(7/10)*1.3803e-23*290*opBW); %noisefigure*thermal noise*290*bw
% Defining SNR range
snr_range = (-35:1:45);
sampleTime = 0.1;
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

%% IMU GPS sensor fusion based path correction

if count == 1
%     [minValue_xRobot,imu_closestIndex_xRobot] = min(abs((estPosition(1))-AP_x));
%     [minValue_yRobot,imu_closestIndex_yRobot] = min(abs(estPosition(2)-AP_y));
    for ap_list = 1:length(AP_x)
        dist_ap(ap_list) = sqrt(((estPosition(1)- AP_x(ap_list))^2)+(estPosition(2)- AP_y(ap_list))^2);
    end
    [minValue_xRobot,imu_closestIndex_xRobot] = min(dist_ap);
    [minValue_yRobot,imu_closestIndex_yRobot] = min(dist_ap);    
    imu_distance_closestAP(count,i) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
    imu_Ploss(count,i) = propagation_loss (imu_distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
    imu_SNR(count,i) = txPower-imu_Ploss(count,i)-30-10*log10(NoisePower);
    imu_STASnr_rndd(count,i) = ceil(str2num(sprintf('%.1f',imu_SNR(count,i))));
    
    if imu_STASnr_rndd(count,i) > 45
        imu_STASnr_rndd(count,i) = 45;
    end
    
    % MCS calculation for AX
    locate_snr_classical = find(snr_range==imu_STASnr_rndd(count,i));
    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
    
    if isempty(best_mcs_classical_temp)
        best_mcs_classical_temp = 1;
    else
        best_mcs_classical_temp = best_mcs_classical_temp(end);
    end
    
    imu_best_mcs_classical(count,i) = best_mcs_classical_temp -1;

    % Packet error rate with AX
    imu_AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

    % .11ax MU UL frame duration calculation to be added later
    
    if imu_AX_per(count,i) >= 0.001       % Simulating packet drop due to high PER
        accelData = accelData*rand; 
        gyroData = gyroData*rand; 
    end
else
    if imu_STASnr_rndd(count-1,i) >= 20
        imu_distance_closestAP(count,i) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
        imu_Ploss(count,i) = propagation_loss (imu_distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
        imu_SNR(count,i) = txPower-imu_Ploss(count,i)-30-10*log10(NoisePower);
        imu_STASnr_rndd(count,i) = ceil(str2num(sprintf('%.1f',imu_SNR(count,i))));
        if imu_STASnr_rndd(count,i) > 45
            imu_STASnr_rndd(count,i) = 45;
        end
        if imu_STASnr_rndd(count,i) < -35
            imu_STASnr_rndd(count,i) = -35;
        end
        
        % MCS calculation for AX
        locate_snr_classical = find(snr_range==imu_STASnr_rndd(count,i));
        best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
        
        if isempty(best_mcs_classical_temp)
            best_mcs_classical_temp = 1;
        else
            best_mcs_classical_temp = best_mcs_classical_temp(end);
        end
        
        imu_best_mcs_classical(count,i) = best_mcs_classical_temp -1;

        % Packet error rate with AX
        imu_AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

        if imu_AX_per(count,i) >= 0.001      % Simulating packet drop due to high PER
%             accelData = accelData*rand; 
%             gyroData = gyroData*rand;    
            AX_per_temp = imu_AX_per(count,i);
            AP_x_temp = AP_x;
            AP_y_temp = AP_y;
            closestIndex_xRobot_old = []; % Flushing out old values from temporary AP indices
            closestIndex_yRobot_old = []; % Flushing out old values from temporary AP indices
            STASnr_rndd_old = [];
            STASnr_rndd_old = [STASnr_rndd_old;imu_STASnr_rndd(count,i)];
            closestIndex_xRobot_old = [closestIndex_xRobot_old;imu_closestIndex_xRobot];
            closestIndex_yRobot_old = [closestIndex_yRobot_old;imu_closestIndex_yRobot];
            while AX_per_temp >= 0.001
                AP_x_temp(imu_closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again for MRC
                AP_y_temp(imu_closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again for MRC
%               [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x_temp));
%               [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y_temp));
                for ap_list = 1:length(AP_x)
                    dist_ap(ap_list) = sqrt(((estPosition(1)- AP_x_temp(ap_list))^2)+(estPosition(2)- AP_y_temp(ap_list))^2);
                end
                [minValue_xRobot,imu_closestIndex_xRobot] = min(dist_ap);
                [minValue_yRobot,imu_closestIndex_yRobot] = min(dist_ap);
                distance_closestAP = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
                Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
                SNR = txPower-Ploss-30-10*log10(NoisePower);
                STASnr = ceil(str2num(sprintf('%.1f',SNR)));
                if STASnr > 45
                    STASnr = 45;
                end
                closestIndex_xRobot_old = [closestIndex_xRobot_old;imu_closestIndex_xRobot];
                closestIndex_yRobot_old = [closestIndex_yRobot_old;imu_closestIndex_yRobot];                    
                % MRC-SNR calculation
                STASnr_rndd_old = [STASnr_rndd_old;STASnr];
                for mrc_calc = 1:length(STASnr_rndd_old)
                    mrc_weights(mrc_calc) = 10^(STASnr_rndd_old(mrc_calc)/10); % Weight by SNR (but not in dB!)
                end
                SNR_after_MRC_temp = 10*log10(sum(mrc_weights));
                SNR_after_MRC = ceil(str2num(sprintf('%.1f',SNR_after_MRC_temp)));
                    
                % MCS calculation for AX
                locate_snr_classical = find(snr_range==SNR_after_MRC);
                best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                if isempty(best_mcs_classical_temp)
                    best_mcs_classical_temp = 1;
                else
                    best_mcs_classical_temp = best_mcs_classical_temp(end);
                end
                best_mcs_classical = best_mcs_classical_temp -1;

                % Packet error rate with AX
                AX_per_temp = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);                    
            end
            imu_AX_per(count,i) = AX_per_temp;
            imu_STASnr_rndd(count,i) = SNR_after_MRC;
            imu_closestIndex_xRobot_mrc = closestIndex_xRobot_old;
            imu_closestIndex_yRobot_mrc = closestIndex_yRobot_old;
        end
    else
        if imu_STASnr_rndd(count-1,i) < 20   % Re-association
            [minValue_xRobot,imu_closestIndex_xRobot] = min(abs(estPosition(1)-AP_x));
            [minValue_yRobot,imu_closestIndex_yRobot] = min(abs(estPosition(2)-AP_y));
            imu_distance_closestAP(count,i) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
            imu_Ploss(count,i) = propagation_loss (imu_distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
            imu_SNR(count,i) = txPower-imu_Ploss(count,i)-30-10*log10(NoisePower);
            imu_STASnr_rndd(count,i) = ceil(str2num(sprintf('%.1f',imu_SNR(count,i))));
            if imu_STASnr_rndd(count,i) > 45
                imu_STASnr_rndd(count,i) = 45;
            end
            if imu_STASnr_rndd(count,i) < -35
                imu_STASnr_rndd(count,i) = -35;
            end  
            % MCS calculation for AX
            locate_snr_classical = find(snr_range==imu_STASnr_rndd(count,i));
            best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
            if isempty(best_mcs_classical_temp)
                best_mcs_classical_temp = 1;
            else
                best_mcs_classical_temp = best_mcs_classical_temp(end);
            end
            
            imu_best_mcs_classical(count,i) = best_mcs_classical_temp -1;

            % Packet error rate with AX
            imu_AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

            if imu_AX_per(count,i) >= 0.001        % Simulating packet drop due to high PER
%                 accelData = accelData*rand; 
%                 gyroData = gyroData*rand;   
                AX_per_temp = imu_AX_per(count,i);
                AP_x_temp = AP_x;
                AP_y_temp = AP_y;
                closestIndex_xRobot_old = []; % Flushing out old values from temporary AP indices
                closestIndex_yRobot_old = []; % Flushing out old values from temporary AP indices
                STASnr_rndd_old = [];
                STASnr_rndd_old = [STASnr_rndd_old;imu_STASnr_rndd(count,i)];
                closestIndex_xRobot_old = [closestIndex_xRobot_old;imu_closestIndex_xRobot];
                closestIndex_yRobot_old = [closestIndex_yRobot_old;imu_closestIndex_yRobot];
                while AX_per_temp >= 0.001
                    AP_x_temp(imu_closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again for MRC
                    AP_y_temp(imu_closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again for MRC
    %               [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x_temp));
    %               [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y_temp));
                    for ap_list = 1:length(AP_x)
                        dist_ap(ap_list) = sqrt(((estPosition(1)- AP_x_temp(ap_list))^2)+(estPosition(2)- AP_y_temp(ap_list))^2);
                    end
                    [minValue_xRobot,imu_closestIndex_xRobot] = min(dist_ap);
                    [minValue_yRobot,imu_closestIndex_yRobot] = min(dist_ap);
                    distance_closestAP = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
                    Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
                    SNR = txPower-Ploss-30-10*log10(NoisePower);
                    STASnr = ceil(str2num(sprintf('%.1f',SNR)));
                    if STASnr > 45
                        STASnr = 45;
                    end
                    closestIndex_xRobot_old = [closestIndex_xRobot_old;imu_closestIndex_xRobot];
                    closestIndex_yRobot_old = [closestIndex_yRobot_old;imu_closestIndex_yRobot];                    
                    % MRC-SNR calculation
                    STASnr_rndd_old = [STASnr_rndd_old;STASnr];
                    for mrc_calc = 1:length(STASnr_rndd_old)
                        mrc_weights(mrc_calc) = 10^(STASnr_rndd_old(mrc_calc)/10); % Weight by SNR (but not in dB!)
                    end
                    SNR_after_MRC_temp = 10*log10(sum(mrc_weights));
                    SNR_after_MRC = ceil(str2num(sprintf('%.1f',SNR_after_MRC_temp)));

                    % MCS calculation for AX
                    locate_snr_classical = find(snr_range==SNR_after_MRC);
                    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                    if isempty(best_mcs_classical_temp)
                        best_mcs_classical_temp = 1;
                    else
                        best_mcs_classical_temp = best_mcs_classical_temp(end);
                    end
                    best_mcs_classical = best_mcs_classical_temp -1;

                    % Packet error rate with AX
                    AX_per_temp = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);                    
                end
                imu_AX_per(count,i) = AX_per_temp;
                imu_STASnr_rndd(count,i) = SNR_after_MRC;
                imu_closestIndex_xRobot_mrc = closestIndex_xRobot_old;
                imu_closestIndex_yRobot_mrc = closestIndex_yRobot_old;
            end
        end
    end
end

% Use the predict method to estimate the filter state based on the accelData and gyroData arrays.
predict(gndFusion, accelData, gyroData);

% Log the estimated orientation and position.
[estPosition, estOrientation] = pose(gndFusion);

estPosition = estPosition.'; 
disp("estPosition")
disp(estPosition)

eul_angle = max(quat2eul(estOrientation));
        
prob_eul_angle = quat2eul(estOrientation);
        
eul_angle = abs(prob_eul_angle(1));

if estPosition(1) <= -10
    eul_angle = 1.0370;
else
    if estPosition(1) > -10 && estPosition(1) <=0
        eul_angle = -1.3370;
    else 
        if estPosition(1) > 0 && estPosition(1) <=7
            eul_angle = -1.0370;
        else
            if estPosition(1) > 7 && estPosition(1) <=20
                eul_angle = 1.0470;    
            end
        end
    end
end

% This next step happens at the GPS sample rate. Simulate the GPS output based on the current pose.
[lla, gpsVel] = gps(truePosition, trueVel);
        
if isnan(lla)
    lla = [0,0,0];
end
        
if isnan(gpsVel)
    gpsVel = [0,0,0];
end
        
% Update the filter states based on the GPS data.
fusegps(gndFusion, lla, Rpos, gpsVel, Rvel);
    
    
[imu_minValue_x,imu_closestIndex_x(count,i)] = min(abs(estPosition(1)-path_x1));
[imu_minValue_y,imu_closestIndex_y(count,i)] = min(abs(estPosition(2)-path_y1));
deviation_fusion_method(count,i) = sqrt(((estPosition(1)- path_x1(imu_closestIndex_x(count,i)))^2)+(estPosition(2)- path_y1(imu_closestIndex_y(count,i)))^2);
    
% fusion_pos = [fusion_pos; estPosition(1) estPosition(2)];

% Re-compute the distance to the goal
distanceToGoal = norm(estPosition(1:2) - robotGoal(:));