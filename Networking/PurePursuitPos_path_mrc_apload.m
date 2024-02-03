function [robotCurrentPose,distanceToGoal,vel,closestIndex_xRobot,closestIndex_yRobot,STASnr_rndd,closestIndex_xRobot_mrc,closestIndex_yRobot_mrc,best_mcs_classical,AP_label]=PurePursuitPos_path_mrc_apload(i,Params,robot,robotCurrentPose,count,snr_per_mcs_5GHz_indoor_channelB,controller,STASnr_rndd,closestIndex_xRobot,closestIndex_yRobot,closestIndex_xRobot_mrc,closestIndex_yRobot_mrc,AP_x,AP_y,AP_label)

%% Classical IEEE 802.11ax parameters
Nss = 1;                                % Number of spatial streams
max_num_sta_in_max_packed_ppdus = 9;    % If 2 MHz RU in 20 MHz BW
GI = 0.8;                               % Guard interval duration in us
per_target = 0.001;                     % PER threshold 10^(-3)
packet_size = 1000;                      % Bytes
bandwidth = 20;                         % RU -- 2/4/8/20 MHz

%% Define a set of waypoints for path #1

x1 = [-17 -15 -10 -6 -3 0 5 10 16];
y1 = [0 10 19 10 -10 -17 -22 -20 -17];
path_x1 = -17:0.5:16;
path_y1 = interp1(x1,y1,path_x1,'spline');
path_x1 = path_x1.';
path_y1 = path_y1.';
path = [path_x1 path_y1];

% Input parameters %
txPower = 25;                                  % In mW (dB)
opBW = 2e7;                                    % Operational BW in MHz
NoisePower = (10.^(7/10)*1.3803e-23*290*opBW); % noisefigure*thermal noise*290*bw

% Defining SNR range
snr_range = (-35:1:45);
sampleTime = 0.1;
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

%% Controller based path correction
% if count ==75
%     disp('Debugging needed');
% end

if count == 1
%     [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x));
%     [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y));
    current_ap_load = 1;
    AX_per(count,i) = 1;
    AP_x_temp = AP_x;
    AP_y_temp = AP_y;
    iter = 0;
    while current_ap_load > 0 || AX_per(count,i) >= 0.001
        if iter == 0
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

            for j=1:length(AP_label)
                if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
                    if AP_label{j}.load == 1                                    % AP overloaded
                        current_ap_load = AP_label{j}.load;
                    else
                        if AP_label{j}.load == 0 && AX_per(count,i) < 0.001    % AP underloaded, PER < threshold
                            current_ap_load = AP_label{j}.load;
                            current_ap_num_sta = AP_label{j}.num_sta;
                            % Updating the assigned AP load
                            AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);
                            % Compute the controller outputs, i.e., the inputs to the robot actuator
                            [v, omega] = controller(robotCurrentPose);
                        else
                            if AP_label{j}.load == 0 && AX_per(count,i) >= 0.001 % AP underloaded, PER > threshold
                                current_ap_load = AP_label{j}.load;
    %                             omega = randi([0,180],1,1);
    %                             v = randi([0,10],1,1);    
                            end
                        end
                    end
                end
            end
            iter = iter+1;
        else
            AP_x_temp(closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again
            AP_y_temp(closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again
            for ap_list = 1:length(AP_x)
                dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x_temp(ap_list))^2)+(robotCurrentPose(2)- AP_y_temp(ap_list))^2);
            end
            [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
            [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);                    
            distance_closestAP = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
            Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
            SNR = txPower-Ploss-30-10*log10(NoisePower);
            STASnr_rndd = ceil(str2num(sprintf('%.1f',SNR)));
            if STASnr_rndd > 45
                STASnr_rndd = 45;
            end

            % MCS calculation for AX classical
            locate_snr_classical = find(snr_range==STASnr_rndd);
            best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
            if isempty(best_mcs_classical_temp)
                best_mcs_classical_temp = 1;
            else
                best_mcs_classical_temp = best_mcs_classical_temp(end);
            end
            best_mcs_classical = best_mcs_classical_temp -1;  

            % dot11ax MU UL frame duration calculation 
            AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
            AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration 
            
            % Packet error rate with AX
            AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

            for j=1:length(AP_label)
                if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
                    if AP_label{j}.load == 1                                    % AP overloaded
                        current_ap_load = AP_label{j}.load;
                    else
                        if AP_label{j}.load == 0 && AX_per(count,i) < 0.001    % AP underloaded, PER < threshold
                            current_ap_load = AP_label{j}.load;
                            current_ap_num_sta = AP_label{j}.num_sta;
                            % Updating the assigned AP load
                            AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);
                            % Compute the controller outputs, i.e., the inputs to the robot actuator
                            [v, omega] = controller(robotCurrentPose);
                        else
                            if AP_label{j}.load == 0 && AX_per(count,i) >= 0.001 % AP underloaded, PER > threshold
                                current_ap_load = AP_label{j}.load;
    %                             omega = randi([0,180],1,1);
    %                             v = randi([0,10],1,1);    
                            end
                        end
                    end
                end
            end            
            iter = iter+1;
        end
    end
else
    if STASnr_rndd(count-1,i) >= 20
        AP_x_temp = AP_x;
        AP_y_temp = AP_y;
        for ap_list = 1:length(AP_x)
            dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x(ap_list))^2)+(robotCurrentPose(2)- AP_y(ap_list))^2);
        end
        [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
        [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);
        distance_closestAP(count,i) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2) + (robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
        Ploss(count,i) = propagation_loss (distance_closestAP(count,i),Params);     % pathloss vs. distance + fixed shadow fading
        
        % SNR calculation
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
 
%         AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y); 
    
        % Packet error rate with AX
        AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
             
        for j=1:length(AP_label)
            if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
                if AP_label{j}.load == 1                                    % AP overloaded
                    current_ap_load = AP_label{j}.load;
                    while current_ap_load > 0
                        AP_x_temp(closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again
                        AP_y_temp(closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again
                        for ap_list = 1:length(AP_x)
                            dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x_temp(ap_list))^2)+(robotCurrentPose(2)- AP_y_temp(ap_list))^2);
                        end
                        [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
                        [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);                    
                        distance_closestAP = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
                        Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
                        SNR = txPower-Ploss-30-10*log10(NoisePower);
                        STASnr_rndd = ceil(str2num(sprintf('%.1f',SNR)));
                        if STASnr_rndd > 45
                            STASnr_rndd = 45;
                        end

                        % MCS calculation for AX classical
                        locate_snr_classical = find(snr_range==STASnr_rndd);
                        best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                        if isempty(best_mcs_classical_temp)
                            best_mcs_classical_temp = 1;
                        else
                            best_mcs_classical_temp = best_mcs_classical_temp(end);
                        end
                        best_mcs_classical = best_mcs_classical_temp -1;  

                        % dot11ax MU UL frame duration calculation 
                        AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
                        AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration 

                        % Packet error rate with AX
                        AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

                        for m=1:length(AP_label)
                            if AP_label{m}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{m}.Location(2) == AP_y(closestIndex_yRobot)
                                if AP_label{m}.load == 1                                    % AP overloaded
                                    current_ap_load = AP_label{m}.load;
                                else
                                    current_ap_load = 0;
                                end
                            end
                        end                      
                    end
                end
            end
        end

        for j=1:length(AP_label)
            if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
                if AP_label{j}.load == 0 && AX_per(count,i) < 0.001    % AP underloaded, PER < threshold, MRC not needed
                        current_ap_load = AP_label{j}.load;
                        current_ap_num_sta = AP_label{j}.num_sta;
                        closestIndex_xRobot_mrc = [];
                        closestIndex_yRobot_mrc = [];
                        % Updating the assigned AP load
                        AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);
                        % Compute the controller outputs, i.e., the inputs to the robot actuator
                        [v, omega] = controller(robotCurrentPose);
                else
                    if AP_label{j}.load == 0 && AX_per(count,i) >= 0.001 % AP underloaded, PER > threshold, MRC needed
                        current_ap_load = AP_label{j}.load;
%                             omega = randi([0,180],1,1);
%                             v = randi([0,10],1,1);
                        AP_x_temp = AP_x;
                        AP_y_temp = AP_y;
                        closestIndex_xRobot_old = []; % Flushing out old values from temporary AP indices
                        closestIndex_yRobot_old = []; % Flushing out old values from temporary AP indices
                        STASnr_rndd_old = [];
                        STASnr_rndd_old = [STASnr_rndd_old;STASnr_rndd(count,i)];
                        closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
                        closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];

                        % MCS calculation for AX classical
                        locate_snr_classical = find(snr_range==STASnr_rndd(count,i));
                        best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                        if isempty(best_mcs_classical_temp)
                            best_mcs_classical_temp = 1;
                        else
                            best_mcs_classical_temp = best_mcs_classical_temp(end);
                        end
                        best_mcs_classical = best_mcs_classical_temp -1;                  

%                         % dot11ax MU UL frame duration calculation 
%                         AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
%                         AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration  
                        
                        % Packet error rate with AX
                        AX_per(count,i) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

%                         AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);    

                        for k=1:length(AP_label)
                            if AP_label{k}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{k}.Location(2) == AP_y(closestIndex_yRobot)
                                ap_load_factor = AP_label{k}.load; 
                                AX_per_temp = AX_per(count,i);
                                while AX_per_temp >= 0.001 || ap_load_factor ~= 0              
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

                                    % MCS calculation for AX classical
                                    locate_snr_classical = find(snr_range==STASnr);
                                    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                                    if isempty(best_mcs_classical_temp)
                                        best_mcs_classical_temp = 1;
                                    else
                                        best_mcs_classical_temp = best_mcs_classical_temp(end);
                                    end
                                    best_mcs_classical = best_mcs_classical_temp -1;  

%                                     % dot11ax MU UL frame duration calculation 
%                                     AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
%                                     AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration                    

%                                     AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);   

                                    for m=1:length(AP_label)
                                        if AP_label{m}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{m}.Location(2) == AP_y(closestIndex_yRobot)
                                            if AP_label{m}.load == 0
                                                ap_load_factor = AP_label{m}.load;
                                                closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
                                                closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];                    
                                                % MRC-SNR calculation
                                                STASnr_rndd_old = [STASnr_rndd_old;STASnr];
                                                for mrc_calc = 1:length(STASnr_rndd_old)
                                                    mrc_weights(mrc_calc) = 10^(STASnr_rndd_old(mrc_calc)/10); % Weight by SNR (but not in dB!)
                                                end
                                                SNR_after_MRC_temp = 10*log10(sum(mrc_weights));
                                                SNR_after_MRC = ceil(str2num(sprintf('%.1f',SNR_after_MRC_temp)));

                                                % MCS calculation for AX with MRC
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
                                        end
                                    end
                                end
                            end
                        end
                        AX_per(count,i) = AX_per_temp;
                        STASnr_rndd(count,i) = SNR_after_MRC;
                        closestIndex_xRobot_mrc = closestIndex_xRobot_old;
                        closestIndex_yRobot_mrc = closestIndex_yRobot_old;
                        for mrc_elements=1:length(closestIndex_xRobot_mrc)
                            distance_closestAP = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot_mrc(mrc_elements)))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot_mrc(mrc_elements)))^2);
                            Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
                            SNR = txPower-Ploss-30-10*log10(NoisePower);
                            STASnr = ceil(str2num(sprintf('%.1f',SNR)));
                            if STASnr > 45
                                STASnr = 45;
                            end

                            % MCS calculation for AX classical
                            locate_snr_classical = find(snr_range==STASnr);
                            best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                            if isempty(best_mcs_classical_temp)
                                best_mcs_classical_temp = 1;
                            else
                                best_mcs_classical_temp = best_mcs_classical_temp(end);
                            end
                            best_mcs_classical = best_mcs_classical_temp -1;  

                            % dot11ax MU UL frame duration calculation 
                            AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
                            AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration 
                            AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot_mrc(mrc_elements),closestIndex_yRobot_mrc(mrc_elements),AP_label,AP_x,AP_y);
                        end
                        [v, omega] = controller(robotCurrentPose);
                    end
                end
            end
        end
    
%         if AX_per(count,i) < 0.001
%             for j=1:length(AP_label)
%                 if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
%                     if AP_label{j}.load == 0
%                         % Compute the controller outputs, i.e., the inputs to the robot
%                         [v, omega] = controller(robotCurrentPose);
%                     else
%                         disp('AP overloaded');                     % AP overloaded
%                         [v, omega] = controller(robotCurrentPose); % Will update this section with underloaded AP selection
%                     end                   
%                 end
%             end
% 
%         else
%             if AX_per(count,i) >= 0.001    % Simulating packet drop due to high PER
% %                 omega = randi([0,180],1,1);
% %                 v = randi([0,10],1,1);
%                 AX_per_temp = AX_per(count,i);
%                 AP_x_temp = AP_x;
%                 AP_y_temp = AP_y;
%                 closestIndex_xRobot_old = []; % Flushing out old values from temporary AP indices
%                 closestIndex_yRobot_old = []; % Flushing out old values from temporary AP indices
%                 STASnr_rndd_old = [];
%                 STASnr_rndd_old = [STASnr_rndd_old;STASnr_rndd(count,i)];
%                 closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
%                 closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];
%                 
%                 % MCS calculation for AX classical
%                 locate_snr_classical = find(snr_range==STASnr_rndd(count,i));
%                 best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
%                 if isempty(best_mcs_classical_temp)
%                     best_mcs_classical_temp = 1;
%                 else
%                     best_mcs_classical_temp = best_mcs_classical_temp(end);
%                 end
%                 best_mcs_classical = best_mcs_classical_temp -1;                  
%                 
%                 % dot11ax MU UL frame duration calculation 
%                 AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
%                 AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration                    
% 
%                 AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);    
%                 
%                 for k=1:length(AP_label)
%                     if AP_label{k}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{k}.Location(2) == AP_y(closestIndex_yRobot)
%                         ap_load_factor = AP_label{k}.load; 
%                 
%                         while AX_per_temp >= 0.001 || ap_load_factor ~= 0              
%                             AP_x_temp(closestIndex_xRobot) = 1000;  % Making sure the same APs are not associated again for MRC
%                             AP_y_temp(closestIndex_yRobot) = 1000;  % Making sure the same APs are not associated again for MRC
%         %                     [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x_temp));
%         %                     [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y_temp));
%                             for ap_list = 1:length(AP_x)
%                                 dist_ap(ap_list) = sqrt(((robotCurrentPose(1)- AP_x_temp(ap_list))^2)+(robotCurrentPose(2)- AP_y_temp(ap_list))^2);
%                             end
%                             [minValue_xRobot,closestIndex_xRobot] = min(dist_ap);
%                             [minValue_yRobot,closestIndex_yRobot] = min(dist_ap);                    
%                             distance_closestAP = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
%                             Ploss = propagation_loss (distance_closestAP,Params);     % pathloss vs. distance + fixed shadow fading
%                             SNR = txPower-Ploss-30-10*log10(NoisePower);
%                             STASnr = ceil(str2num(sprintf('%.1f',SNR)));
%                             if STASnr > 45
%                                 STASnr = 45;
%                             end
% 
%                             % MCS calculation for AX classical
%                             locate_snr_classical = find(snr_range==STASnr);
%                             best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
%                             if isempty(best_mcs_classical_temp)
%                                 best_mcs_classical_temp = 1;
%                             else
%                                 best_mcs_classical_temp = best_mcs_classical_temp(end);
%                             end
%                             best_mcs_classical = best_mcs_classical_temp -1;  
% 
%                             % dot11ax MU UL frame duration calculation 
%                             AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
%                             AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration                    
% 
%                             AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);   
% 
%                             for j=1:length(AP_label)
%                                 if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
%                                     if AP_label{j}.load == 0
%                                         ap_load_factor = AP_label{j}.load;
%                                         closestIndex_xRobot_old = [closestIndex_xRobot_old;closestIndex_xRobot];
%                                         closestIndex_yRobot_old = [closestIndex_yRobot_old;closestIndex_yRobot];                    
%                                         % MRC-SNR calculation
%                                         STASnr_rndd_old = [STASnr_rndd_old;STASnr];
%                                         for mrc_calc = 1:length(STASnr_rndd_old)
%                                             mrc_weights(mrc_calc) = 10^(STASnr_rndd_old(mrc_calc)/10); % Weight by SNR (but not in dB!)
%                                         end
%                                         SNR_after_MRC_temp = 10*log10(sum(mrc_weights));
%                                         SNR_after_MRC = ceil(str2num(sprintf('%.1f',SNR_after_MRC_temp)));
% 
%                                         % MCS calculation for AX with MRC
%                                         locate_snr_classical = find(snr_range==SNR_after_MRC);
%                                         best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
%                                         if isempty(best_mcs_classical_temp)
%                                             best_mcs_classical_temp = 1;
%                                         else
%                                             best_mcs_classical_temp = best_mcs_classical_temp(end);
%                                         end
%                                         best_mcs_classical = best_mcs_classical_temp -1;
% 
%                                         % Packet error rate with AX
%                                         AX_per_temp = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
%                                     end
%                                 end
%                             end
%                         end
%                     end
%                 end
%                 AX_per(count,i) = AX_per_temp;
%                 STASnr_rndd(count,i) = SNR_after_MRC;
%                 closestIndex_xRobot_mrc = closestIndex_xRobot_old;
%                 closestIndex_yRobot_mrc = closestIndex_yRobot_old;
%                 [v, omega] = controller(robotCurrentPose);
%             end
%         end
    else
        if STASnr_rndd(count-1,i) < 20   % Re-association   % this condition never happens because of mrc, so this part of the code is not updated
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
                        
                        % MCS calculation for AX classical
                        locate_snr_classical = find(snr_range==STASnr);
                        best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                        if isempty(best_mcs_classical_temp)
                            best_mcs_classical_temp = 1;
                        else
                            best_mcs_classical_temp = best_mcs_classical_temp(end);
                        end
                        best_mcs_classical = best_mcs_classical_temp -1;  

                        % dot11ax MU UL frame duration calculation 
                        AXrate = AXGetMyRate(best_mcs_classical,bandwidth,Nss,GI);
                        AXpyDuration = AXPayloadDuration(packet_size,AXrate,GI);   %Payload duration                    

                        AP_label = APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y);   

                        for j=1:length(AP_label)
                            if AP_label{j}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{j}.Location(2) == AP_y(closestIndex_yRobot)
                                if AP_label{j}.load == 0

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
                            end
                        end
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
    
disp("iteration# robot#")
disp([count,i])
disp("SNR:")
disp(STASnr_rndd(count,i))
                
% Get the robot's velocity using controller inputs
vel = derivative(robot, robotCurrentPose, [v omega]);
    
% Update the current pose
robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
% [minValue_x,closestIndex_x(count,i)] = min(abs(robotCurrentPose(1)-path_x1));
% [minValue_y,closestIndex_y(count,i)] = min(abs(robotCurrentPose(2)-path_y1));
% deviation_controller_method(count) = sqrt(((robotCurrentPose(1)- path_x1(closestIndex_x(count)))^2)+(robotCurrentPose(2)- path_y1(closestIndex_y(count)))^2);
                
% Re-compute the distance to the goal
distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));