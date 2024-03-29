function AP_label=APLoadCalc(count,i,AXpyDuration,closestIndex_xRobot,closestIndex_yRobot,AP_label,AP_x,AP_y)    

%% Classical IEEE 802.11ax parameters
Nss = 1;                                % Number of spatial streams
max_num_sta_in_max_packed_ppdus = 9;    % If 2 MHz RU in 20 MHz BW
GI = 0.8;                               % Guard interval duration in us
per_target = 0.001;                     % PER threshold 10^(-3)
packet_size = 100;                      % Bytes
bandwidth = 20;                         % RU -- 2/4/8/20 MHz
latency = 5000;                         % us (microseconds)

%% Classical .11ax MU UL frame duration calculation 
% Preamble duration
AXprDuration = AXPreambleDuration(Nss,'ax');

% if count == 149
%     if i==10
%         disp("Robot");
%         disp(i);
%         disp("Debugging needed : num_sta=2,rttd=[]");
%     end
% end

% AP load calculation
for m=1:length(AP_label)
%     if m == 49
%         disp(m);
%     end
    if AP_label{m}.Location(1) == AP_x(closestIndex_xRobot) && AP_label{m}.Location(2) == AP_y(closestIndex_yRobot)
%         if i > length(AP_label{m}.payloadDuration)
%             AP_label{m}.payloadDuration = cat(1,AXpyDuration,AXpyDuration);
%         else
%             AP_label{m}.payloadDuration(i) = AXpyDuration;
%         end
%         if length(AXpyDuration)>1
%             AP_label{m}.payloadDuration(i) = AXpyDuration(i);
%         else
        AP_label{m}.payloadDuration(i) = AXpyDuration;
%         end
% %         AP_label{m}.num_sta = AP_label{m}.num_sta + 1;
%         if AP_label{m}.num_sta < i
%             AP_label{m}.num_sta = i;
%         end
        AP_label{m}.num_sta = nnz(AP_label{m}.payloadDuration);             %Number of non-zero elements in AP_label{m}.payloadDuration
        if AP_label{m}.num_sta <= max_num_sta_in_max_packed_ppdus 
            AP_label{m}.num_of_ppdus = 1;
            AP_label{m}.PPDUduration = max(AP_label{m}.payloadDuration)+AXprDuration;
            AP_label{m}.ppdu_overhead = 48;% + AXFrameDurationWCLax('Trigger',AP_label{m}.num_sta) + AXFrameDurationWCLax('BA',AP_label{m}.num_sta); %SIFS = 16us, 3*SIFS = 48
            AP_label{m}.frame_duration = AP_label{m}.PPDUduration + AP_label{m}.ppdu_overhead;
        else
            AP_label{m}.num_of_ppdus = ceil((AP_label{m}.num_sta)/max_num_sta_in_max_packed_ppdus);
            k=1;
            PPDUduration=zeros(1:AP_label{m}.num_of_ppdus);
            ppdu_overhead = zeros(1:AP_label{m}.num_of_ppdus);
            [packed_ppdu,remaining_stas]= quorem(sym(length(AP_label{m}.payloadDuration)),sym(max_num_sta_in_max_packed_ppdus));
            AP_label{m}.payloadDuration = sort(AP_label{m}.payloadDuration);
            for j=1:AP_label{m}.num_of_ppdus
                if j==1
                    PPDUduration(j) = max(AP_label{m}.payloadDuration(1:j*max_num_sta_in_max_packed_ppdus))+AXprDuration;
                    ppdu_overhead(j) = 48;% + AXFrameDurationWCLax('Trigger',max_num_sta_in_max_packed_ppdus) + AXFrameDurationWCLax('BA',max_num_sta_in_max_packed_ppdus);
                    packed_ppdu = packed_ppdu-1;
                else
                    if packed_ppdu > 0
                        k=k+9;
                        PPDUduration(j) = max(AP_label{m}.payloadDuration(k:k+max_num_sta_in_max_packed_ppdus-1))+AXprDuration;
                        ppdu_overhead(j) = 48;% + AXFrameDurationWCLax('Trigger',max_num_sta_in_max_packed_ppdus) + AXFrameDurationWCLax('BA',max_num_sta_in_max_packed_ppdus);
                        packed_ppdu = packed_ppdu-1;
                    else
                        k=k+9;
                        PPDUduration(j) = max(AP_label{m}.payloadDuration(k:k+remaining_stas-1))+AXprDuration;
                        ppdu_overhead(j) = 48;% + AXFrameDurationWCLax('Trigger',remaining_stas) + AXFrameDurationWCLax('BA',remaining_stas);
                    end
                end
            end
            AP_label{m}.PPDUduration = sum(PPDUduration);
%             AP_label{m}.ppdu_overhead = AP_label{m}.num_of_ppdus(48 + AXFrameDurationWCLax('Trigger',AP_label{m}.num_sta) + AXFrameDurationWCLax('BA',AP_label{m}.num_sta));
             AP_label{m}.ppdu_overhead = sum(ppdu_overhead);
            AP_label{m}.frame_duration = AP_label{m}.PPDUduration + AP_label{m}.ppdu_overhead;
        end
        if AP_label{m}.frame_duration > latency
            AP_label{m}.load = 1;
            AP_label{m}.rttd = latency - AP_label{m}.frame_duration;
        else
            AP_label{m}.rttd = latency - AP_label{m}.frame_duration;
        end
    end
end

% % Preamble + Payload duration
% if sta_per_ap_x(closestIndex_xRobot) <= 9 && sta_per_ap_y(closestIndex_yRobot) <= 9
%     AP_x_PPDUduration(closestIndex_xRobot) = max(APx_payload_duration{closestIndex_xRobot});
%     AP_y_PPDUduration(closestIndex_yRobot) = max(APy_payload_duration{closestIndex_yRobot});
%     if AP_x_PPDUduration(closestIndex_xRobot) == AP_y_PPDUduration(closestIndex_yRobot)
%         AXPPDUDuration(count,i) = AP_x_PPDUduration(closestIndex_xRobot) + AXprDuration(count,i);
%     else
%         disp("mismatch in Payload duration for the same AP. Debugging needed");
%     end
% else
%     num_of_ppdus(closestIndex_xRobot) = ceil(sta_per_ap_x(closestIndex_xRobot)/9);
%     AP_x_PPDUduration(closestIndex_xRobot) = max(APx_payload_duration{closestIndex_xRobot});
% end
% 
% AXppdu_overhead(count,i)= 48 + AXFrameDurationWCLax('Trigger',num_sta) + AXFrameDurationWCLax('BA',num_sta); %SIFS = 16us, 3*SIFS = 48
% AXduration_uplink(count,i) = AXPPDUDuration(count,i) + AXppdu_overhead(count,i);


% % Time remaining till deadline in AP (AP Load)
% if AP_x_load(closestIndex_xRobot) && AP_y_load(closestIndex_yRobot) == 0
%     if AXduration_uplink(count,i) >= AP_x_rttd(closestIndex_xRobot) || AXduration_uplink(count,i) >= AP_y_rttd(closestIndex_yRobot) || AXduration_uplink(count,i) >= latency
%         AP_x_load(closestIndex_xRobot) = 1;
%         AP_y_load(closestIndex_yRobot) = 1;
%     else
%         if sta_per_ap_x(closestIndex_xRobot) >= max_num_sta_in_max_packed_ppdus && sta_per_ap_y(closestIndex_yRobot) >= max_num_sta_in_max_packed_ppdus
%             AP_x_rttd(closestIndex_xRobot) = latency - (AP_x_rttd(closestIndex_xRobot)+AXduration_uplink(count,i));
%             AP_y_rttd(closestIndex_yRobot) = latency - (AP_y_rttd(closestIndex_yRobot)+AXduration_uplink(count,i));
% end
