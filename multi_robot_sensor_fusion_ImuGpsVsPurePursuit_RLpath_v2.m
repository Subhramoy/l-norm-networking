clc; close all; clear all;

%% Input number of robots for each navigation method

robot_PP_numbers = 2;   % Number of robots executing Pure Pursuit algorithm
robot_SF_numbers = 0;   % Number of robots executing IMU+GPS sensor fusion algorithm

%% Wireless Network aspects
% Load snr-per-mcs table
load snr_per_mcs_5GHz_indoor_channelB

% Initialize the random number generator used to simulate sensor noise.
rng('default');

% Initializing array for robot actual path tracing
pure_pursuit_pos = [];
fusion_pos = [];
STASnr_rndd = [];
imu_STASnr_rndd = [];

% Defining SNR range
snr_range = (-35:1:45);

% Classical IEEE 802.11ax parameters
Nss = 1;                                % Number of spatial streams
max_num_sta_in_max_packed_ppdus = 9;    % If 2 MHz RU in 20 MHz BW
GI = 0.8;                               % Guard interval duration in us
per_target = 0.001;                     % PER threshold 10^(-3)
packet_size = 100;                      % Bytes
bandwidth = 20;                         % RU -- 2/4/8/20 MHz
num_sta = 1;                            % Using 1 robot now for testing code

% Initialize the random number generator used to simulate sensor noise.
rng('default');

% % Location of APs in the environment
% AP_x = [-30 -20 -10 -100 10 20 30 30 20 10 0 -20 -30 -10 0 -100 -100 -100 -100 -100 -20 -300 10 20 30 -200 -200 20 20 -300 -30 30 30 10 20 -100 -200 -200 -100 10 20 -100 -100 10 10 -30 -300 30 30];
% % AP_x = [-30 -20 -10 0 10 20 30 30 20 10 -10 -20 -30 0 0 0 0 0 0 -10 -20 -30 10 20 30 -20 -20 20 20 -30 -30 30 30 10 20 -10 -20 -20 -10 10 20 -10 -10 10 10 -30 -30 30 30];
% AP_y = [-30 -20 -10 0 10 20 30 -30 -20 -10 10 20 30 -10 -20 -30 10 20 30 0 0 0 0 0 0 -10 10 -10 10 -10 10 -10 10 -30 -30 -30 -30 30 30 30 30 -20 20 20 -20 -20 20 -20 20];

% Location of APs in the environment
AP_x = [-30 -20 -10 -100 100 20 30 30 20 10 0 -20 -30 -10 0 -100 -100 -100 -100 -100 -20 -300 100 20 30 -200 -200 20 20 -300 -30 30 30 10 20 -100 -200 -200 -100 10 20 -100 -100 10 10 -30 -300 30 30];
% AP_x = [-30 -20 -10 0 10 20 30 30 20 10 -10 -20 -30 0 0 0 0 0 0 -10 -20 -30 10 20 30 -20 -20 20 20 -30 -30 30 30 10 20 -10 -20 -20 -10 10 20 -10 -10 10 10 -30 -30 30 30];
AP_y = [-30 -20 -10 0 10 20 30 -30 -20 -10 10 20 30 -10 -20 -30 10 20 30 0 0 0 0 0 0 -10 10 -10 10 -10 10 -10 10 -30 -30 -30 -30 30 30 30 30 -20 20 20 -20 -20 20 -20 20];

% Labeling APs based on their X,Y coordinates and tracking each AP load factor
for i=1:length(AP_x)
    AP_label{i}.Location = [AP_x(i);AP_y(i)]; % X,Y coordinates of AP location
    AP_label{i}.Location_x = AP_x(i);
    AP_label{i}.Location_y = AP_y(i);
    AP_label{i}.num_sta = 0;                  % Number of STAs associated with this AP
    AP_label{i}.load = 0;                     % AP overloaded (crossed latency threshold) = 1
    AP_label{i}.payloadDuration = [0 0 0 0];      % Duration of PP uplink frame at this time instant for this AP
    AP_label{i}.payloadDuration_sf = [0 0];   % Duration of SF uplink frame at this time instant for this AP
end

% Input parameters %
txPower = 25;               %% In mW (dB)
opBW = 2e7;                 %% Operational BW in MHz
Params=struct();
Params.fixedshadowfadingtmp = 18;
Params.randomShadowFading = 0;
Params.Frequency = 5;       %% Operating frequency in GHz
Params.ch.nFloor = 0;       %% This is set to 0 since the number of floor will be calculated based on real floor plan
Params.ch.nWall = 0;        %% This is set to 0 since the number of floor will be calculated based on real floor plan
nChannelRealizations = 1;   %% Number of channel realizations to study
% ----------------------------------------------------------------------- %

% Channel Model %
Params.ch.type = 'B';       % Indoor channel model B
Params.ch.Liw = 7;          % Penetration loss for a single wall
Params.ch = path_loss(Params.ch);
% ----------------------------------------------------------------------- %

NoisePower = (10.^(7/10)*1.3803e-23*290*opBW); % noisefigure*thermal noise*290*bw

%% Define a set of waypoints for path #1

x1 = [-17 -15 -10 -6 -3 0 5 10 16];
y1 = [0 10 19 10 -10 -17 -22 -20 -17];
path_x1 = -17:0.5:16;
path_y1 = interp1(x1,y1,path_x1,'spline');
path_x1 = path_x1.';
path_y1 = path_y1.';
path = [path_x1 path_y1];

path_z1 = zeros(size(path_x1));

%% Define a set of waypoints for path #2

x2 = [-25 -22 -18 -15 -13 -10 -8 -5 -3 0 2 5 8 10];
y2 = [11 15 21 13 0 -15 -20 -25 -24 -22 -20 -15 -10 -6];
path_x2 = -25:0.5:10;
path_y2 = interp1(x2,y2,path_x2,'spline');
path_x2 = path_x2.';
path_y2 = path_y2.';
path1 = [path_x2 path_y2];

path_z2 = zeros(size(path_x2));

%% Sensor fusion integration

imuFs = 10;
gpsFs = 10;
localOrigin = [42.2825 -71.343 53.0352];

for i=1:robot_SF_numbers
    gndFusion{i} = insfilterNonholonomic('ReferenceFrame', 'ENU', ...
        'IMUSampleRate', imuFs, ...
        'ReferenceLocation', localOrigin, ...
        'DecimationFactor', 2);
    
    gps{i} = gpsSensor('UpdateRate', gpsFs, 'ReferenceFrame', 'ENU');
    gps{i}.ReferenceLocation = localOrigin;  % 3-element row vector in geodetic coordinates (latitude, longitude, and altitude) [degrees degrees meters]
    gps{i}.DecayFactor = 0.5;                % [0,1] 0 models the global position noise as a white noise process. 1 models the global position noise as a random walk process. 
    gps{i}.HorizontalPositionAccuracy = 1.0; % standard deviation of the noise in the horizontal position measurement  
    gps{i}.VerticalPositionAccuracy =  1.0;  % standard deviation of the noise in the vertical position measurement
    gps{i}.VelocityAccuracy = 0.1;           % standard deviation of the noise in the velocity measurement

    imu{i} = imuSensor('accel-gyro', ...
        'ReferenceFrame', 'ENU', 'SampleRate', imuFs);
    imu{i}.Accelerometer.MeasurementRange =  19.6133; % Max measurable accleration in m/s^2
    imu{i}.Accelerometer.Resolution = 0.0023928;      % Resolution of sensor measurements in (m/s^2)/LSB
    imu{i}.Accelerometer.NoiseDensity = 0.0012356;    % Power spectral density of sensor noise in (m/s^2/√Hz)
    imu{i}.Gyroscope.MeasurementRange = deg2rad(250);
    imu{i}.Gyroscope.Resolution = deg2rad(0.0625);
    imu{i}.Gyroscope.NoiseDensity = deg2rad(0.025);
%     theta = zeros(size(path_x2));
    initialYaw(i) = 0; % (degrees)
    
    % Measurement noises
    Rvel(i) = gps{i}.VelocityAccuracy.^2;
    Rpos(i) = gps{i}.HorizontalPositionAccuracy.^2;

    gndFusion{i}.ZeroVelocityConstraintNoise = 1e-2;

    % Process noises
    gndFusion{i}.GyroscopeNoise = 4e-6;
    gndFusion{i}.GyroscopeBiasNoise = 4e-14;
    gndFusion{i}.AccelerometerNoise = 4.8e-2;
    gndFusion{i}.AccelerometerBiasNoise = 4e-14;

    % Initial error covariance
    gndFusion{i}.StateCovariance = 1e-9*ones(16);

    if mod(i,2) == 0
        position{i} = [path_x2, path_y2, path_z2];

        for j=1:length(path_x2)
            if path_y2(j) >=11 && path_y2(j) <=21 && path_x2(j) >= -25 && path_x2(j) <=-18
                yaw_temp(j) = 60;
            else
                if path_y2(j) <= 21 && path_y2(j) >=-25 && path_x2(j) > -18 && path_x2(j) < -4.5
                    yaw_temp(i) = 0;
                else
                    if path_y2(j) > -25 && path_y2(j) <=-6 && path_x2(j) >= -4.5 && path_x2(j) <=10
                        yaw_temp(j) = 60;
                    end
                end
            end
        end

        % Define orientation.
        yaw = yaw_temp.';
        yaw = deg2rad(yaw);
        yaw = mod(yaw, 2*pi);
        pitch = zeros(size(yaw));
        roll = zeros(size(yaw));
        orientation = quaternion([yaw, pitch, roll], 'euler', ...
            'ZYX', 'frame');

        
        % Added speed for debugging
        speed(i) = 0.8 -(i/10);   % Giving unique speeds to each robot 

        for j=1:length(path_x2)
            if j==length(path_x2)
                break;
            end
            dist(j) = sqrt(((path_x2(j+1)- path_x2(j))^2)+(path_y2(j+1)- path_y2(j))^2); 
        end
        distance_traversed = sum(dist);

        % Added rev time for debugging
        revTime = distance_traversed / speed(i);
        time_of_arrival = linspace(0, revTime, numel(path_x2)).';
        t = time_of_arrival;

        % Generate trajectory.
        groundTruth{i} = waypointTrajectory('SampleRate', imuFs, ...
            'Waypoints', position{i}, ...
            'TimeOfArrival', t, ...
            'Orientation', orientation);

        [initialPos, initialAtt, initialVel] = groundTruth{i}();
        reset(groundTruth{i});

        % Initialize the states of the filter
        gndFusion{i}.State(1:4) = compact(initialAtt).';
        gndFusion{i}.State(5:7) = imu{i}.Gyroscope.ConstantBias;
        gndFusion{i}.State(8:10) = initialPos.';
        gndFusion{i}.State(11:13) = initialVel.';
        gndFusion{i}.State(14:16) = imu{i}.Accelerometer.ConstantBias;
    else
        if mod(i,2) ~= 0
            position{i} = [path_x1, path_y1, path_z1];

            for j=1:length(path_x1)
                if path_y1(j) >=0 && path_y1(j) <18.94 && path_x1(j) >= -17 && path_x1(j) <-9.5
                    yaw_temp(j) = 60;
                else
                    if path_y1(j) <=18.93  && path_y1(j) >=-22.14 && path_x1(j) >= -9.5 && path_x1(j) < 5.5
                        yaw_temp(i) = 0;
                    else
                        if path_y1(j) > -22.14 && path_y1(j) <=-17 && path_x1(j) >= 5.5 && path_x1(j) <=16
                            yaw_temp(j) = 60;
                        end
                    end
                end
            end

            % Define orientation.
            yaw = yaw_temp.';
            yaw = deg2rad(yaw);
            yaw = mod(yaw, 2*pi);
            pitch = zeros(size(yaw));
            roll = zeros(size(yaw));
            orientation = quaternion([yaw, pitch, roll], 'euler', ...
                'ZYX', 'frame');
            
            %Added speed for debugging
            speed(i) = 0.8 -(i/10);   % Giving unique speeds to each robot 

            for j=1:length(path_x1)
                if j==length(path_x1)
                    break;
                end
                dist(j) = sqrt(((path_x1(j+1)- path_x1(j))^2)+(path_y1(j+1)- path_y1(j))^2); 
            end
            distance_traversed = sum(dist);

            %added rev time for debugging
            revTime = distance_traversed / speed(i);
            time_of_arrival = linspace(0, revTime, numel(path_x1)).';
            t = time_of_arrival;

            % Generate trajectory.
            groundTruth{i} = waypointTrajectory('SampleRate', imuFs, ...
                'Waypoints', position{i}, ...
                'TimeOfArrival', t, ...
                'Orientation', orientation);

            [initialPos, initialAtt, initialVel] = groundTruth{i}();
            reset(groundTruth{i});

            % Initialize the states of the filter
            gndFusion{i}.State(1:4) = compact(initialAtt).';
            gndFusion{i}.State(5:7) = imu{i}.Gyroscope.ConstantBias;
            gndFusion{i}.State(8:10) = initialPos.';
            gndFusion{i}.State(11:13) = initialVel.';
            gndFusion{i}.State(14:16) = imu{i}.Accelerometer.ConstantBias;
        end
    end
end

for i=1:robot_SF_numbers
    robot_sf(i) = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
    frameSize_sf(i) = robot_sf(i).TrackWidth/0.3;
end

%% Controller based path correction metrics

goalRadius = 0.1;
for i=1:robot_PP_numbers
    initialOrientation(i) = 0;
    robot(i) = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
    controller{i} = controllerPurePursuit;
    controller{i}.DesiredLinearVelocity = 0.8; % Each robot gets a unique linear velocity
    controller{i}.MaxAngularVelocity = 2;      % Each robot gets a unique angular velocity
    controller{i}.LookaheadDistance = 0.4;
    % Determine vehicle frame size to most closely represent vehicle with plotTransforms
    frameSize(i) = robot(i).TrackWidth/0.3;
    if mod(i,2) ~= 0
        robotInitialLocation(i,:) = path(1,:);
        robotGoal(i,:) = path(end,:);
        robotCurrentPose{i} = [robotInitialLocation(i,:) initialOrientation(i)]';       
        distanceToGoal(i) = norm(robotInitialLocation(i,:) - robotGoal(i,:));       
        controller{i}.Waypoints = path;
    else
        if mod(i,2) == 0
            robotInitialLocation(i,:) = path1(1,:);
            robotGoal(i,:) = path1(end,:);
            robotCurrentPose{i} = [robotInitialLocation(i,:) initialOrientation(i)]';
            distanceToGoal(i) = norm(robotInitialLocation(i,:) - robotGoal(i,:));
            controller{i}.Waypoints = path1;
        end
    end
end

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure


%% Simulation Loop

count = 0;
for i=1:robot_PP_numbers
    [minValue_xRobot,closestIndex_xRobot(i)] = min(abs(robotCurrentPose{i}(1))-AP_x);
    [minValue_yRobot,closestIndex_yRobot(i)] = min(abs(robotCurrentPose{i}(2))-AP_y);
end

for i=1:robot_SF_numbers
    % Simulate the IMU data from the current pose.
    [truePosition, trueOrientation, trueVel, trueAcc, trueAngVel] = groundTruth{i}();
    [accelData, gyroData] = imu{i}(trueAcc, trueAngVel, trueOrientation);

    % Use the predict method to estimate the filter state based on the accelData and gyroData arrays.
    if isnan(accelData)
        accelData = [0,0,0];
    end
    if isnan(gyroData)
        gyroData = [0,0,0];
    end
            
    predict(gndFusion{i}, accelData, gyroData);
        
    % Log the estimated orientation and position.
    [estPosition{i}, estOrientation{i}] = pose(gndFusion{i});
    [minValue_xRobot,imu_closestIndex_xRobot(i)] = min(abs((estPosition{i}(1))-AP_x));
    [minValue_yRobot,imu_closestIndex_yRobot(i)] = min(abs(estPosition{i}(2)-AP_y));
end

while( distanceToGoal >= goalRadius )
    count = count + 1; 

    AP_label_final(count,:) = AP_label;
    
    for i=1:length(AP_x)
        AP_label{i}.Location = [AP_x(i);AP_y(i)]; % X,Y coordinates of AP location
        AP_label{i}.Location_x = AP_x(i);
        AP_label{i}.Location_y = AP_y(i);
        AP_label{i}.num_sta = 0;                  % Number of STAs associated with this AP
        AP_label{i}.load = 0;                     % AP overloaded (crossed latency threshold) = 1
        AP_label{i}.payloadDuration = [0 0 0 0];      % Duration of PP uplink frame at this time instant for this AP
        AP_label{i}.payloadDuration_sf = [0 0];   % Duration of SF uplink frame at this time instant for this AP
    end
    
    % Update the plot
    hold off

    % Plot path each instance so that it stays persistent while robot mesh moves
    plot(path(:,1), path(:,2),"b--",path1(:,1), path1(:,2),"g--")
    hold on
    light;
    xlim([-30 30])
    ylim([-30 30])
    grid on
    % Plot the location of the APs
    plot(AP_x,AP_y,'r*');
    
    hold all
    
    % Pure Pursuit based path following
    for i = 1:robot_PP_numbers
        if mod(i,2) ~= 0
        % Update the current pose
            [robotCurrentPose{i},distanceToGoal,vel,closestIndex_xRobot(i),closestIndex_yRobot(i),STASnr_rndd,AP_label] = PurePursuitPos_path(i,Params,robot(i),robotCurrentPose{i}, ...
                count,snr_per_mcs_5GHz_indoor_channelB,controller{i},AP_x,AP_y,STASnr_rndd,closestIndex_xRobot(i),closestIndex_yRobot(i),AP_label);
            robotCurrentPose{i} = robotCurrentPose{i} + vel*sampleTime; 
            
            for waypoints = 1:length(path_x1)
                dist_dev(waypoints) = sqrt(((robotCurrentPose{i}(1)- path_x1(waypoints))^2)+(robotCurrentPose{i}(2)- path_y1(waypoints))^2);
            end 
            robot_deviation(count,i) = min(dist_dev);
    %         pure_pursuit_pos(:,:,i) = [pure_pursuit_pos; robotCurrentPose(1) robotCurrentPose(2)];

            % Plot the path of the robot as a set of transforms
            plotTrVec = [robotCurrentPose{i}(1:2); 0];
            plotRot = axang2quat([0 0 1 robotCurrentPose{i}(3)]);
            plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize(i), "MeshColor", "blue");
            plot(AP_x(closestIndex_xRobot(i)),AP_y(closestIndex_yRobot(i)),'k*');
    %         plot(pure_pursuit_pos(:,1,i), pure_pursuit_pos(:,2,i), '-b', 'LineWidth', 2);
        else 
            if mod(i,2) == 0
            % Update the current pose
            [robotCurrentPose{i},distanceToGoal,vel,closestIndex_xRobot(i),closestIndex_yRobot(i),STASnr_rndd,AP_label] = PurePursuitPos_path1(i,Params,robot(i),robotCurrentPose{i}, ...
                count,snr_per_mcs_5GHz_indoor_channelB,controller{i},AP_x,AP_y,STASnr_rndd,closestIndex_xRobot(i),closestIndex_yRobot(i),AP_label);
            robotCurrentPose{i} = robotCurrentPose{i} + vel*sampleTime; 
            
            for waypoints = 1:length(path_x2)
                dist_dev(waypoints) = sqrt(((robotCurrentPose{i}(1)- path_x2(waypoints))^2)+(robotCurrentPose{i}(2)- path_y2(waypoints))^2);
            end 
            robot_deviation(count,i) = min(dist_dev);

    %         pure_pursuit_pos(:,:,i) = [pure_pursuit_pos; robotCurrentPose(1) robotCurrentPose(2)];

            % Plot the path of the robot as a set of transforms
            plotTrVec = [robotCurrentPose{i}(1:2); 0];
            plotRot = axang2quat([0 0 1 robotCurrentPose{i}(3)]);
            plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize(i), "MeshColor", "blue");
            plot(AP_x(closestIndex_xRobot(i)),AP_y(closestIndex_yRobot(i)),'b*');
    %         plot(pure_pursuit_pos(:,1,i), pure_pursuit_pos(:,2,i), '-b', 'LineWidth', 2);
            end
        end
    end
    
    % IMU+GPS sensor fusion based path following
    for i=1:robot_SF_numbers
        % Simulate the IMU data from the current pose.
        [truePosition, trueOrientation, trueVel, trueAcc, trueAngVel] = groundTruth{i}();
        [accelData, gyroData] = imu{i}(trueAcc, trueAngVel, trueOrientation);

        % Use the predict method to estimate the filter state based on the accelData and gyroData arrays.
        if isnan(accelData)
            accelData = [0,0,0];
        end
        if isnan(gyroData)
            gyroData = [0,0,0];
        end
            
        predict(gndFusion{i}, accelData, gyroData);
        
        % Log the estimated orientation and position.
        [estPosition{i}, estOrientation{i}] = pose(gndFusion{i});
        if mod(i,2) ~= 0
            [estPosition{i},eul_angle,imu_closestIndex_xRobot(i),imu_closestIndex_yRobot(i),imu_STASnr_rndd,distanceToGoal]=ImuGpsFusion_path(i, ... 
                Params,truePosition,trueVel,Rpos(i),Rvel(i),estPosition{i},count,snr_per_mcs_5GHz_indoor_channelB, ... 
                AP_x,AP_y,imu_STASnr_rndd,gndFusion{i},accelData,gyroData,gps{i},imu_closestIndex_xRobot(i),imu_closestIndex_yRobot(i));
            
                % Plot the path of the robot as a set of transforms
                plotTrVec1 = [estPosition{i}(1:2); 0];
                plotRot1 = axang2quat([0 0 1 eul_angle]);
                plotTransforms(plotTrVec1', plotRot1, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize_sf(i), "MeshColor", "green");
                plot(AP_x(imu_closestIndex_xRobot),AP_y(imu_closestIndex_yRobot),'g*');
%                 plot(fusion_pos(:,1), fusion_pos(:,2), '-g', 'LineWidth', 2);
        else
            if mod(i,2) == 0
                [estPosition{i},eul_angle,imu_closestIndex_xRobot(i),imu_closestIndex_yRobot(i),imu_STASnr_rndd,distanceToGoal]=ImuGpsFusion_path1(i, ... 
                Params,truePosition,trueVel,Rpos(i),Rvel(i),estPosition{i},count,snr_per_mcs_5GHz_indoor_channelB, ...
                AP_x,AP_y,imu_STASnr_rndd,gndFusion{i},accelData,gyroData,gps{i},imu_closestIndex_xRobot(i),imu_closestIndex_yRobot(i));
            
                % Plot the path of the robot as a set of transforms
                plotTrVec1 = [estPosition{i}(1:2); 0];
                plotRot1 = axang2quat([0 0 1 eul_angle]);
                plotTransforms(plotTrVec1', plotRot1, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize_sf(i), "MeshColor", "green");
                plot(AP_x(imu_closestIndex_xRobot),AP_y(imu_closestIndex_yRobot),'g*');
%                 plot(fusion_pos(:,1), fusion_pos(:,2), '-g', 'LineWidth', 2); 
            end
        end     
    end
    
    waitfor(vizRate);
end

%% End
