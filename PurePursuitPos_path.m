function [robotCurrentPose,distanceToGoal,vel,closestIndex_xRobot,closestIndex_yRobot,STASnr_rndd,AP_label]=PurePursuitPos_path(i,Params,robot,robotCurrentPose,count,snr_per_mcs_5GHz_indoor_channelB,controller,AP_x,AP_y,STASnr_rndd,closestIndex_xRobot,closestIndex_yRobot,AP_label)

%% Define a set of waypoints for path #1

x1 = [-17 -15 -10 -6 -3 0 5 10 16];
y1 = [0 10 19 10 -10 -17 -22 -20 -17];
path_x1 = -17:0.5:16;
path_y1 = interp1(x1,y1,path_x1,'spline');
path_x1 = path_x1.';
path_y1 = path_y1.';
path = [path_x1 path_y1];

% Classical IEEE 802.11ax parameters
Nss = 1;                                % Number of spatial streams
max_num_sta_in_max_packed_ppdus = 9;    % If 2 MHz RU in 20 MHz BW
GI = 0.8;                               % Guard interval duration in us
per_target = 0.001;                     % PER threshold 10^(-3)
packet_size = 100;                      % Bytes
bandwidth = 20;                         % RU -- 2/4/8/20 MHz
num_sta = 1;                            % Using 1 robot now for testing code


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
    
    % Classical .11ax MU UL frame duration calculation 
    AXrate = AXGetMyRate(best_mcs_classical(count,i),bandwidth,Nss,GI);
    AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration in usecs

    % Packet error rate with AX
    AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
    % Updating the assigned AP load
    AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);

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
        
        % Classical .11ax MU UL frame duration calculation 
        AXrate = AXGetMyRate(best_mcs_classical(count,i),bandwidth,Nss,GI);
        AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration in usecs
    
        % Packet error rate with AX
        AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
        
        % Updating the assigned AP load
        AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);
    
        if AX_per(count,i) <= 0.001
        % Compute the controller outputs, i.e., the inputs to the robot
            [v, omega] = controller(robotCurrentPose);
        else
            if AX_per(count,i) > 0.001      % Simulating packet drop due to high PER
                omega = randi([0,180],1,1);
                v = randi([0,10],1,1);     
            end
        end
    else
        if STASnr_rndd(count-1,i) < 20   % Re-association
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
            
            % Classical .11ax MU UL frame duration calculation 
            AXrate = AXGetMyRate(best_mcs_classical(count,i),bandwidth,Nss,GI);
            AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration in usecs
    
            % Packet error rate with AX
            AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
            
            % Updating the assigned AP load
            AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);
    
            if AX_per(count,i) <= 0.001
            % Compute the controller outputs, i.e., the inputs to the robot
                [v, omega] = controller(robotCurrentPose);
            else
                if AX_per(count,i) > 0.001        % Simulating packet drop due to high PER
                    omega = randi([0,180],1,1);
                    v = randi([0,10],1,1);
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