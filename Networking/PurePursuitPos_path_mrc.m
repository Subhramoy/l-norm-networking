function [robotCurrentPose,distanceToGoal,vel,closestIndex_xRobot,closestIndex_yRobot,STASnr_rndd,closestIndex_xRobot_mrc,closestIndex_yRobot_mrc,best_mcs_classical]=PurePursuitPos_path_mrc(i,Params,robot,robotCurrentPose,count,snr_per_mcs_5GHz_indoor_channelB,controller,AP_x,AP_y,STASnr_rndd,closestIndex_xRobot,closestIndex_yRobot,closestIndex_xRobot_mrc,closestIndex_yRobot_mrc)

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

%% Controller based path correction

if count == 1
%     [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x));
%     [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y));
    for ap_list = 1:length(AP_x)
        dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x(ap_list))^2)+(robotCurrentPose(2)- AP_y(ap_list))^2);
    end
    [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
    [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);
    distance_closestAP(count,i) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
    Ploss(count,i) = propagation_loss (distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
    SNR(count,i) = txPower-Ploss(count,i)-30-10*log10(NoisePower);
    STASnr_rndd(count,i) = ceil(str2num(sprintf('%.1f',SNR(count,i))));
    if STASnr_rndd(count,i) > 45
        STASnr_rndd(count,i) = 45;
    end

    % MCS calculation for AX
    locate_snr_classical = find(snr_range==STASnr_rndd(count,i));
    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
    if isempty(best_mcs_classical_temp)
        best_mcs_classical_temp = 1;
    else
        best_mcs_classical_temp = best_mcs_classical_temp(end);
    end
    best_mcs_classical(count,i) = best_mcs_classical_temp -1;
    

    % Packet error rate with AX
    AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

    % .11ax MU UL frame duration calculation to be added later

    if AX_per(count,i) <= 0.001
    % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = controller(robotCurrentPose);
    else
        if AX_per(count,i) > 0.001       % Simulating packet drop due to high PER
            omega = randi([0,180],1,1);
            v = randi([0,10],1,1);    
        end
    end
else
    if STASnr_rndd(count-1,i) >= 20
        distance_closestAP(count,i) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2) + (robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
        Ploss(count,i) = propagation_loss (distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
        SNR(count,i) = txPower-Ploss(count,i)-30-10*log10(NoisePower);
        STASnr_rndd(count,i) = ceil(str2num(sprintf('%.1f',SNR(count,i))));
        if STASnr_rndd(count,i) > 45
            STASnr_rndd(count,i) = 45;
        end
        % MCS calculation for AX
        locate_snr_classical = find(snr_range==STASnr_rndd(count,i));
        best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
        if isempty(best_mcs_classical_temp)
            best_mcs_classical_temp = 1;
        else
            best_mcs_classical_temp = best_mcs_classical_temp(end);
        end
        best_mcs_classical(count,i) = best_mcs_classical_temp -1;
    
        % Packet error rate with AX
        AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
        if AX_per(count,i) < 0.001
        % Compute the controller outputs, i.e., the inputs to the robot
            [v, omega] = controller(robotCurrentPose);
        else
            if AX_per(count,i) >= 0.001      % Simulating packet drop due to high PER
%                 omega = randi([0,180],1,1);
%                 v = randi([0,10],1,1);
                AX_per_temp = AX_per(count,i);
                AP_x_temp = AP_x;
                AP_y_temp = AP_y;
                closestIndex_xRobot_old = []; % Flushing out old values from temporary AP indices
                closestIndex_yRobot_old = []; % Flushing out old values from temporary AP indices
                STASnr_rndd_old = [];
                STASnr_rndd_old = [STASnr_rndd_old;STASnr_rndd(count,i)];
                closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
                closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];
                while AX_per_temp >= 0.001                  
                    AP_x_temp(closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again for MRC
                    AP_y_temp(closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again for MRC
%                     [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x_temp));
%                     [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y_temp));
                    for ap_list = 1:length(AP_x)
                        dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x_temp(ap_list))^2)+(robotCurrentPose(2)- AP_y_temp(ap_list))^2);
                    end
                    [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
                    [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);
                    distance_closestAP = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
                    Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
                    SNR = txPower-Ploss-30-10*log10(NoisePower);
                    STASnr = ceil(str2num(sprintf('%.1f',SNR)));
                    if STASnr > 45
                        STASnr = 45;
                    end
                    closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
                    closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];                    
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
                AX_per(count,i) = AX_per_temp;
                STASnr_rndd(count,i) = SNR_after_MRC;
                closestIndex_xRobot_mrc = closestIndex_xRobot_old;
                closestIndex_yRobot_mrc = closestIndex_yRobot_old;
                [v, omega] = controller(robotCurrentPose);
            end
        end
    else
        if STASnr_rndd(count-1,i) < 20   % Re-association
            [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x));
            [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y));
            distance_closestAP(count,i) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
            Ploss(count,i) = propagation_loss (distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
            SNR(count,i) = txPower-Ploss(count,i)-30-10*log10(NoisePower);
            STASnr_rndd(count,i) = ceil(str2num(sprintf('%.1f',SNR(count,i))));
            if STASnr_rndd(count,i) > 45
                STASnr_rndd(count,i) = 45;
            end
            % MCS calculation for AX
            locate_snr_classical = find(snr_range==STASnr_rndd(count,i));
            best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
            if isempty(best_mcs_classical_temp)
                best_mcs_classical_temp = 1;
            else
                best_mcs_classical_temp = best_mcs_classical_temp(end);
            end
            best_mcs_classical(count,i) = best_mcs_classical_temp -1;
    
            % Packet error rate with AX
            AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
            if AX_per(count,i) < 0.001
            % Compute the controller outputs, i.e., the inputs to the robot
                [v, omega] = controller(robotCurrentPose);
            else
                if AX_per(count,i) >= 0.001        % Simulating packet drop due to high PER
%                     omega = randi([0,180],1,1);
%                     v = randi([0,10],1,1);
%                 omega = randi([0,180],1,1);
%                 v = randi([0,10],1,1);
                    AX_per_temp = AX_per(count,i);
                    AP_x_temp = AP_x;
                    AP_y_temp = AP_y;
                    closestIndex_xRobot_old = []; % Flushing out old values from temporary AP indices
                    closestIndex_yRobot_old = []; % Flushing out old values from temporary AP indices
                    STASnr_rndd_old = [];
                    STASnr_rndd_old = [STASnr_rndd_old;STASnr_rndd(count,i)];
                    while AX_per_temp >= 0.001
                        closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
                        closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];                  
                        AP_x_temp(closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again for MRC
                        AP_y_temp(closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again for MRC
%                         [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x_temp));
%                         [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y_temp));
                        for ap_list = 1:length(AP_x)
                            dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x_temp(ap_list))^2)+(robotCurrentPose(2)- AP_y_temp(ap_list))^2);
                        end
                        [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
                        [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);
                        distance_closestAP = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
                        Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
                        SNR = txPower-Ploss-30-10*log10(NoisePower);
                        STASnr = ceil(str2num(sprintf('%.1f',SNR)));
                        if STASnr > 45
                            STASnr = 45;
                        end

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
                    AX_per(count,i) = AX_per_temp;
                    STASnr_rndd(count,i) = SNR_after_MRC;
                    closestIndex_xRobot_mrc = closestIndex_xRobot_old;
                    closestIndex_yRobot_mrc = closestIndex_yRobot_old;
                    [v, omega] = controller(robotCurrentPose);
                end
            end
        end
    end
end
    
disp("Step:")
disp([count,i])
disp("SNR:")
disp(STASnr_rndd(count,i))
                
% Get the robot's velocity using controller inputs
vel = derivative(robot, robotCurrentPose, [v omega]);
    
% Update the current pose
robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
[minValue_x,closestIndex_x(count,i)] = min(abs(robotCurrentPose(1)-path_x1));
[minValue_y,closestIndex_y(count,i)] = min(abs(robotCurrentPose(2)-path_y1));
% deviation_controller_method(count) = sqrt(((robotCurrentPose(1)- path_x1(closestIndex_x(count)))^2)+(robotCurrentPose(2)- path_y1(closestIndex_y(count)))^2);
                
% Re-compute the distance to the goal
distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));