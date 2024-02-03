function [robotCurrentPose,distanceToGoal,vel,closestIndex_xRobot,closestIndex_yRobot,STASnr_rndd]=PurePursuitPos_path1_mrc(i,Params,robot,robotCurrentPose,count,snr_per_mcs_5GHz_indoor_channelB,controller,AP_x,AP_y,STASnr_rndd,closestIndex_xRobot,closestIndex_yRobot)

%% Define a set of waypoints for path #2

x2 = [-25 -22 -18 -15 -13 -10 -8 -5 -3 0 2 5 8 10];
y2 = [11 15 21 13 0 -15 -20 -25 -24 -22 -20 -15 -10 -6];
path_x2 = -25:0.5:10;
path_y2 = interp1(x2,y2,path_x2,'spline');
path_x2 = path_x2.';
path_y2 = path_y2.';
path = [path_x2 path_y2];

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
    
        % Packet error rate with AX
        AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
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
    
[minValue_x,closestIndex_x(count,i)] = min(abs(robotCurrentPose(1)-path_x2));
[minValue_y,closestIndex_y(count,i)] = min(abs(robotCurrentPose(2)-path_y2));
% deviation_controller_method(count) = sqrt(((robotCurrentPose(1)- path_x1(closestIndex_x(count)))^2)+(robotCurrentPose(2)- path_y1(closestIndex_y(count)))^2);
                
% Re-compute the distance to the goal
distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));