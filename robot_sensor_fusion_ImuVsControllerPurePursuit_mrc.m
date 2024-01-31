clc; close all; clear all;

%% Networking aspects
% Load snr-per-mcs table
load snr_per_mcs_5GHz_indoor_channelB

snr_range = (-35:1:45);

% Classical .11ax parameters
Nss = 1;                                %Number of spatial streams
max_num_sta_in_max_packed_ppdus = 9;    %If 2 MHz RU in 20 MHz BW
GI = 0.8;                               %Guard interval duration in us
per_target = 0.001;                     %PER threshold 10^(-3)
packet_size = 100;                      %Bytes
bandwidth = 20;                         %RU -- 2/4/8/20 MHz
num_sta = 1;                            %Using 1 robot now for testing code

% Initialize the random number generator used to simulate sensor noise.
rng('default');

% awgnNoise = randi([-5,40],1,1);

% Location of AP in the environment
AP_x = [-30 -20 -10 0 10 20 30 30 20 10 -10 -20 -30 0 0 0 0 0 0 -10 -20 -30 10 20 30 -20 -20 20 20 -30 -30 30 30 10 20 -10 -20 -20 -10 10 20 -10 -10 10 10 -30 -30 30 30];
AP_y = [-30 -20 -10 0 10 20 30 -30 -20 -10 10 20 30 -10 -20 -30 10 20 30 0 0 0 0 0 0 -10 10 -10 10 -10 10 -10 10 -30 -30 -30 -30 30 30 30 30 -20 20 20 -20 -20 20 -20 20];

% Robot spawning location in the environment
spawn_x = -25;
spawn_y = -25;

% Robot speed
robot_speed = 0.5; %meters/sec

% Input parameters %
txPower = 25;               % In mW (dB)
opBW = 2e7;                 % Operational BW in MHz
Params=struct();
Params.fixedshadowfadingtmp = 18;
Params.randomShadowFading = 0;
Params.Frequency = 5;       %% Operating frequency in GHz
Params.ch.nFloor = 0;       %% This is set to 0 since the number of floor will be calculated based on real floor plan
Params.ch.nWall = 0;        %% This is set to 0 since the number of floor will be calculated based on real floor plan
nChannelRealizations = 1;   %  Number of channel realizations to study
% ----------------------------------------------------------------------- %

% channel model %
Params.ch.type = 'B';       %Indoor channel model B
Params.ch.Liw = 7;          %Penetration loss for a single wall
Params.ch = path_loss(Params.ch);
% ------------------------------------------------------------------------------------------- %

NoisePower = (10.^(7/10)*1.3803e-23*290*opBW); %noisefigure*thermal noise*290*bw

%Theoretical path traversed by the robot in the environment
for i=1:402
    if spawn_x >= -25 && spawn_x <= -15 && spawn_y == -25
        dynamic_x(i) = spawn_x + robot_speed;
        dynamic_y(i) = spawn_y;
        spawn_x = dynamic_x(i);
        spawn_y = dynamic_y(i);
    else
        if spawn_x > -15 && spawn_x < 5 && spawn_y >= -25 && spawn_y <= 25
            dynamic_y(i) = spawn_y + robot_speed;
            dynamic_x(i) = spawn_x;
            spawn_x = dynamic_x(i);
            spawn_y = dynamic_y(i);
        else
            if spawn_y == 25.5 && spawn_x >= -14.5 && spawn_x <= 5
                dynamic_x(i) = spawn_x + robot_speed;
                dynamic_y(i) = spawn_y;
                spawn_x = dynamic_x(i);
                spawn_y = dynamic_y(i); 
            else
                if spawn_x == 5.5 && spawn_y <=25.5 && spawn_y >= -25
                    dynamic_y(i) = spawn_y - robot_speed;
                    dynamic_x(i) = spawn_x;
                    spawn_x = dynamic_x(i);
                    spawn_y = dynamic_y(i);
                else
                    if spawn_y == -25.5 && spawn_x >= 5.5 && spawn_x <= 25
                        dynamic_y(i) = spawn_y;
                        dynamic_x(i) = spawn_x + robot_speed;
                        spawn_x = dynamic_x(i);
                        spawn_y = dynamic_y(i);   
                    else
                        if spawn_x == 25.5 && spawn_y >=-25.5 && spawn_y <= 25
                            dynamic_y(i) = spawn_y + robot_speed;
                            dynamic_x(i) = spawn_x;
                            spawn_x = dynamic_x(i);
                            spawn_y = dynamic_y(i);  
                        end
                    end
                end
            end
        end
    end            
end

%% Controller based path correction
% Define a set of waypoints for the desired path for the robot

path  = [-24.5 -25; -20 -25; -14.5 -25; -14.5 -20.5; -14.5 -15.5; -14.5 -10; -14.5 -5; -14.5 -0.5; -14.5 5; 

-14.5 10; -14.5 15; -14.5 20; -14.5 25.5; -10 25.5; -4.5 25.5; 0 25.5; 5.5 25.5; 5.5 20; 5.5 15; 5.5 10; 5.5 5; 

5.5 0; 5.5 -5; 5.5 -10; 5.5 -15; 5.5 -20; 5.5 -25.5; 10 -25.5; 15 -25.5; 20 -25.5; 20 -25.5; 25.5 -25.5; 25.5 -20; 

25.5 -15; 25.5 -10; 25.5 -5; 25.5 0; 25.5 5; 25.5 10; 25.5 15; 25.5 20; 25.5 25];

% Set the current location and the goal location of the robot as defined by the path
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

% Assume an initial robot orientation 
% (the robot orientation is the angle between the robot heading and the positive X-axis, measured counterclockwise)
initialOrientation = 0;

% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

% Initialize the robot model and assign an initial pose 
% The simulated robot has kinematic equations for the motion of a two-wheeled differential drive robot
% The inputs to this simulated robot are linear and angular velocities
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Based on the path defined above and a robot motion model, you need a path following controller to drive the robot along the path. 
% Create the path following controller using the controllerPurePursuit object.
controller = controllerPurePursuit;

% Use the path defined above to set the desired waypoints for the controller
controller.Waypoints = path;

% Set the path following controller parameters. 
% The desired linear velocity is set to 0.6 meters/second for this example.
controller.DesiredLinearVelocity = 0.8;

% The maximum angular velocity acts as a saturation limit for rotational velocity. 
% This is set at 2 radians/second for this example.
controller.MaxAngularVelocity = 2;

% The lookahead distance should be larger than the desired linear velocity for a smooth path.
% The robot might cut corners when the lookahead distance is large.
% A small lookahead distance can result in an unstable path following behavior.
controller.LookaheadDistance = 0.4;

% The path following controller provides input control signals for the robot. 
% The robot uses to drive itself along the desired path.
% Too small value of the goal radius may cause the robot to miss the goal, which may result in an unexpected behavior near the goal.
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.3;


%% Sensor fusion integration

% To simulate this configuration, the IMU (accelerometer and gyroscope) is
% sampled at 100 Hz, and the GPS is sampled at 10 Hz.

imuFs = 10;
gpsFs = 10;

% Define where on the Earth this simulation takes place using latitude, 
% longitude, and altitude (LLA) coordinates.
localOrigin = [42.2825 -71.343 53.0352];

% Validate that the |gpsFs| divides |imuFs|. This allows the sensor sample
% rates to be simulated using a nested for loop without complex sample rate
% matching.

% imuSamplesPerGPS = (imuFs/gpsFs);
% assert(imuSamplesPerGPS == fix(imuSamplesPerGPS), ...
%     'GPS sampling rate must be an integer factor of IMU sampling rate.');

gndFusion = insfilterNonholonomic('ReferenceFrame', 'ENU', ...
    'IMUSampleRate', imuFs, ...
    'ReferenceLocation', localOrigin, ...
    'DecimationFactor', 2);

% Trajectory parameters
path_x  = [-24.5; -20; -14.5; -14.5; -14.5; -14.5; -14.5; -14.5; -14.5; 

-14.5; -14.5; -14.5; -14.5; -10; -4.5; 0; 5.5; 5.5; 5.5; 5.5; 5.5; 

5.5; 5.5; 5.5; 5.5; 5.5; 5.5; 10; 15; 20; 20; 25.5; 25.5; 

25.5; 25.5; 25.5; 25.5; 25.5; 25.5; 25.5; 25.5; 25.5];

path_y  = [-25; -25; -25; -20.5; -15.5; -10; -5; -0.5; 5; 

10; 15; 20; 25.5; 25.5; 25.5; 25.5; 25.5; 20; 15; 10; 5; 

0; -5; -10; -15; -20; -25.5; -25.5; -25.5; -25.5; -25.5; -25.5; -20; 

-15; -10; -5; 0; 5; 10; 15; 20; 25];

path_z = zeros(size(path_x));
position = [path_x, path_y, path_z];
%Added speed for debugging
speed = 0.8;

for i=1:length(path_x)
    if i==length(path_x)
        break;
    end
    dist(i) = sqrt(((path_x(i+1)- path_x(i))^2)+(path_y(i+1)- path_y(i))^2); 
end
distance_traversed = sum(dist);
%added rev time for debugging
revTime = distance_traversed / speed;

time_of_arrival = linspace(0, revTime, numel(path_x)).';
t = time_of_arrival;

theta = zeros(size(path_x));
initialYaw = 0; % (degrees)
for i=1:length(path_x)
    if path_y(i) == -25 && path_x(i) >= -24.5 && path_x(i) <= -14.5
        yaw_temp(i) = 0;
    else 
        if path_x(i) == -14.5 && path_y(i) >= -20.5 && path_y(i) <= 25.5
            yaw_temp(i) = 90;
        else
            if path_x(i) >= -10 && path_x(i) <= 5.5 && path_y(i) == 25.5
                yaw_temp(i) = 0;
            else
                if path_x(i) == 5.5 && path_y(i) <=20 && path_y(i) >= -25.5
                    yaw_temp(i) = -90;
                else
                    if path_x(i) >= 10 && path_x(i) <= 25.5 && path_y(i) == -25.5
                       yaw_temp(i) = 0; 
                    else
                        if path_x(i) == 25.5 && path_y(i) >= -20 && path_y(i) <= 25
                            yaw_temp(i) = 90; 
                        end
                    end
                end
            end
        end
    end
end

% Define orientation.
yaw=yaw_temp.';
yaw = deg2rad(yaw);
yaw = mod(yaw, 2*pi);
pitch = zeros(size(yaw));
roll = zeros(size(yaw));
orientation = quaternion([yaw, pitch, roll], 'euler', ...
    'ZYX', 'frame');

% Generate trajectory.
groundTruth = waypointTrajectory('SampleRate', imuFs, ...
    'Waypoints', position, ...
    'TimeOfArrival', t, ...
    'Orientation', orientation);

% Initialize the random number generator used to simulate sensor noise.
rng('default');

%% GPS Receiver
% Set up the GPS at the specified sample rate and reference location. The
% other parameters control the nature of the noise in the output signal.

gps = gpsSensor('UpdateRate', gpsFs, 'ReferenceFrame', 'ENU');
gps.ReferenceLocation = localOrigin;  % 3-element row vector in geodetic coordinates (latitude, longitude, and altitude) [degrees degrees meters]
gps.DecayFactor = 0.5;                % [0,1] 0 models the global position noise as a white noise process. 1 models the global position noise as a random walk process. 
gps.HorizontalPositionAccuracy = 1.0; % standard deviation of the noise in the horizontal position measurement  
gps.VerticalPositionAccuracy =  1.0;  % standard deviation of the noise in the vertical position measurement
gps.VelocityAccuracy = 0.1;           % standard deviation of the noise in the velocity measurement

%% IMU Sensors

imu = imuSensor('accel-gyro', ...
    'ReferenceFrame', 'ENU', 'SampleRate', imuFs);

% Accelerometer
imu.Accelerometer.MeasurementRange =  19.6133; % Max measurable accleration in m/s^2
imu.Accelerometer.Resolution = 0.0023928;      % Resolution of sensor measurements in (m/s^2)/LSB
imu.Accelerometer.NoiseDensity = 0.0012356;    % Power spectral density of sensor noise in (m/s^2/√Hz)

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.0625);
imu.Gyroscope.NoiseDensity = deg2rad(0.025);

%% Initialize the States of the |insfilterNonholonomic|
% The states are:
% 
%  States                            Units    Index
%  Orientation (quaternion parts)             1:4  
%  Gyroscope Bias (XYZ)              rad/s    5:7  
%  Position (NED)                    m        8:10 
%  Velocity (NED)                    m/s      11:13
%  Accelerometer Bias (XYZ)          m/s^2    14:16
%
% Ground truth is used to help initialize the filter states, so the filter
% converges to good answers quickly.

% Get the initial ground truth pose from the first sample of the trajectory
% and release the ground truth trajectory to ensure the first sample is not 
% skipped during simulation.
[initialPos, initialAtt, initialVel] = groundTruth();
reset(groundTruth);

% Initialize the states of the filter
gndFusion.State(1:4) = compact(initialAtt).';
gndFusion.State(5:7) = imu.Gyroscope.ConstantBias;
gndFusion.State(8:10) = initialPos.';
gndFusion.State(11:13) = initialVel.';
gndFusion.State(14:16) = imu.Accelerometer.ConstantBias;

%% Initialize the Variances of the |insfilterNonholonomic|
% The measurement noises describe how much noise is corrupting the GPS 
% reading based on the |gpsSensor| parameters and how much uncertainty is 
% in the vehicle dynamic model.

% Measurement noises
Rvel = gps.VelocityAccuracy.^2;
Rpos = gps.HorizontalPositionAccuracy.^2;

% The dynamic model of the ground vehicle for this filter assumes there is
% no side slip or skid during movement. This means that the velocity is 
% constrained to only the forward body axis. The other two velocity axis 
% readings are corrected with a zero measurement weighted by the 
% |ZeroVelocityConstraintNoise| parameter.
gndFusion.ZeroVelocityConstraintNoise = 1e-2;

% Process noises
gndFusion.GyroscopeNoise = 4e-6;
gndFusion.GyroscopeBiasNoise = 4e-14;
gndFusion.AccelerometerNoise = 4.8e-2;
gndFusion.AccelerometerBiasNoise = 4e-14;

% Initial error covariance
gndFusion.StateCovariance = 1e-9*ones(16);

%% Simulation Loop
% The main simulation loop is a while loop with a nested for loop. The
% while loop executes at the |gpsFs|, which is the GPS measurement rate.
% The nested for loop executes at the |imuFs|, which is the IMU sample
% rate. The scopes are updated at the IMU sample rate.

totalSimTime = 100; % seconds

% Log data for final metric computation.
numsamples = floor(min(t(end), totalSimTime) * gpsFs);
truePosition = zeros(numsamples,3);
trueOrientation = quaternion.zeros(numsamples,1);
estPosition = zeros(numsamples,3);
estOrientation = quaternion.zeros(numsamples,1);

count = 0;

while( distanceToGoal > goalRadius )
    count = count + 1;
    % Controller Pure Pursuit based robot motion with .11ax AP allocation
    % At starting associate with AP closest to robot
    if count == 1
        [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x));
        [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y));
        distance_closestAP(count) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
        Ploss(count) = propagation_loss (distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
        SNR(count) = txPower-Ploss(count)-30-10*log10(NoisePower);
        STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',SNR(count))));
        if STASnr_rndd(count) > 45
            STASnr_rndd(count) = 45;
        end
        % MCS calculation for AX
        locate_snr_classical = find(snr_range==STASnr_rndd(count));
        best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
        if isempty(best_mcs_classical_temp)
            best_mcs_classical_temp = 1;
        else
            best_mcs_classical_temp = best_mcs_classical_temp(end);
        end
        best_mcs_classical(count) = best_mcs_classical_temp -1;
    
        % Packet error rate with AX
        AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
        % .11ax MU UL frame duration calculation to be added later
        
        if AX_per(count) <= 0.001
            % Compute the controller outputs, i.e., the inputs to the robot
            [v, omega] = controller(robotCurrentPose);
        else
            if AX_per(count) > 0.001       % Simulating packet drop due to high PER
                omega = randi([0,180],1,1);
                v = randi([0,10],1,1);    
            end
        end
    else
        if STASnr_rndd(count-1) >= 20
            distance_closestAP(count) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
            Ploss(count) = propagation_loss (distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
            SNR(count) = txPower-Ploss(count)-30-10*log10(NoisePower);
            STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',SNR(count))));
            if STASnr_rndd(count) > 45
                STASnr_rndd(count) = 45;
            end
            % MCS calculation for AX
            locate_snr_classical = find(snr_range==STASnr_rndd(count));
            best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
            if isempty(best_mcs_classical_temp)
                best_mcs_classical_temp = 1;
            else
                best_mcs_classical_temp = best_mcs_classical_temp(end);
            end
            best_mcs_classical(count) = best_mcs_classical_temp -1;
    
            % Packet error rate with AX
            AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
            if AX_per(count) <= 0.001
                % Compute the controller outputs, i.e., the inputs to the robot
                [v, omega] = controller(robotCurrentPose);
            else
                if AX_per(count) > 0.001      % Simulating packet drop due to high PER
%                     omega = randi([0,180],1,1);
%                     v = randi([0,10],1,1);     
                    [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x));
                    [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y));
                    distance_closestAP(count) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
                    Ploss(count) = propagation_loss (distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
                    SNR(count) = txPower-Ploss(count)-30-10*log10(NoisePower);
                    STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',SNR(count))));
                    if STASnr_rndd(count) > 45
                        STASnr_rndd(count) = 45;
                    end
                    % MCS calculation for AX
                    locate_snr_classical = find(snr_range==STASnr_rndd(count));
                    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                    if isempty(best_mcs_classical_temp)
                        best_mcs_classical_temp = 1;
                    else
                        best_mcs_classical_temp = best_mcs_classical_temp(end);
                    end
                    best_mcs_classical(count) = best_mcs_classical_temp -1;

                    % Packet error rate with AX
                    AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
                    [v, omega] = controller(robotCurrentPose);
                end
            end
        else
            if STASnr_rndd(count-1) < 20   % Re-association
                [minValue_xRobot,closestIndex_xRobot] = min(abs(robotCurrentPose(1)-AP_x));
                [minValue_yRobot,closestIndex_yRobot] = min(abs(robotCurrentPose(2)-AP_y));
                distance_closestAP(count) = sqrt(((robotCurrentPose(1)- AP_x(closestIndex_xRobot))^2)+(robotCurrentPose(2)- AP_y(closestIndex_yRobot))^2);
                Ploss(count) = propagation_loss (distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
                SNR(count) = txPower-Ploss(count)-30-10*log10(NoisePower);
                STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',SNR(count))));
                if STASnr_rndd(count) > 45
                    STASnr_rndd(count) = 45;
                end
                % MCS calculation for AX
                locate_snr_classical = find(snr_range==STASnr_rndd(count));
                best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                if isempty(best_mcs_classical_temp)
                    best_mcs_classical_temp = 1;
                else
                    best_mcs_classical_temp = best_mcs_classical_temp(end);
                end
                best_mcs_classical(count) = best_mcs_classical_temp -1;
    
                % Packet error rate with AX
                AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
    
                if AX_per(count) <= 0.001
                    % Compute the controller outputs, i.e., the inputs to the robot
                    [v, omega] = controller(robotCurrentPose);
                else
                    if AX_per(count) > 0.001        % Simulating packet drop due to high PER
                        omega = randi([0,180],1,1);
                        v = randi([0,10],1,1);
                    end
                end
            end
        end
    end
    
    disp("Step:")
    disp(count)
    disp("SNR:")
    disp(STASnr_rndd(count))
                
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Deviation from actual path with controller pure pursuit method
    [minValue_x,closestIndex_x(count)] = min(abs(robotCurrentPose(1)-dynamic_x));
    [minValue_y,closestIndex_y(count)] = min(abs(robotCurrentPose(2)-dynamic_y));
    deviation_controller_method(count) = sqrt(((robotCurrentPose(1)- dynamic_x(closestIndex_x(count)))^2)+(robotCurrentPose(2)- dynamic_y(closestIndex_y(count)))^2);
                
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Sensor Fusion (IMU+GPS) based robot motion with .11ax AP allocation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        
        % Simulate the IMU data from the current pose.
        [truePosition, trueOrientation, ...
        trueVel, trueAcc, trueAngVel] = groundTruth();
        [accelData, gyroData] = imu(trueAcc, trueAngVel, ...
        trueOrientation);

        % Use the predict method to estimate the filter state based
        % on the accelData and gyroData arrays.
        if isnan(accelData)
            accelData = [0,0,0];
        end
        if isnan(gyroData)
            gyroData = [0,0,0];
        end
            
        predict(gndFusion, accelData, gyroData);

        % Log the estimated orientation and position.
        [estPosition, estOrientation] = pose(gndFusion);

        if count == 1
            [minValue_xRobot,imu_closestIndex_xRobot] = min(abs(estPosition(1)-AP_x));
            [minValue_yRobot,imu_closestIndex_yRobot] = min(abs(estPosition(2)-AP_y));
            imu_distance_closestAP(count) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
            imu_Ploss(count) = propagation_loss (imu_distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
            imu_SNR(count) = txPower-imu_Ploss(count)-30-10*log10(NoisePower);
            imu_STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',imu_SNR(count))));
            if imu_STASnr_rndd(count) > 45
                imu_STASnr_rndd(count) = 45;
            end
            % MCS calculation for AX
            locate_snr_classical = find(snr_range==imu_STASnr_rndd(count));
            best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
            if isempty(best_mcs_classical_temp)
                best_mcs_classical_temp = 1;
            else
                best_mcs_classical_temp = best_mcs_classical_temp(end);
            end
            imu_best_mcs_classical(count) = best_mcs_classical_temp -1;

            % Packet error rate with AX
            imu_AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

            % .11ax MU UL frame duration calculation to be added later

            if imu_AX_per(count) > 0.001       % Simulating packet drop due to high PER
                accelData = accelData*rand; 
                gyroData = gyroData*rand; 
            end
        else
            if imu_STASnr_rndd(count-1) >= 20
                imu_distance_closestAP(count) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
                imu_Ploss(count) = propagation_loss (imu_distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
                imu_SNR(count) = txPower-imu_Ploss(count)-30-10*log10(NoisePower);
                imu_STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',imu_SNR(count))));
                if imu_STASnr_rndd(count) > 45
                    imu_STASnr_rndd(count) = 45;
                end
                if imu_STASnr_rndd(count) < -35
                    imu_STASnr_rndd(count) = -35;
                end               
                % MCS calculation for AX
                locate_snr_classical = find(snr_range==imu_STASnr_rndd(count));
                best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                if isempty(best_mcs_classical_temp)
                    best_mcs_classical_temp = 1;
                else
                    best_mcs_classical_temp = best_mcs_classical_temp(end);
                end
                imu_best_mcs_classical(count) = best_mcs_classical_temp -1;

                % Packet error rate with AX
                imu_AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

                if imu_AX_per(count) > 0.001      % Simulating packet drop due to high PER
%                     accelData = accelData*rand; 
%                     gyroData = gyroData*rand;    
                    [minValue_xRobot,imu_closestIndex_xRobot] = min(abs(estPosition(1)-AP_x));
                    [minValue_yRobot,imu_closestIndex_yRobot] = min(abs(estPosition(2)-AP_y));
                    imu_distance_closestAP(count) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
                    imu_Ploss(count) = propagation_loss (imu_distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
                    imu_SNR(count) = txPower-imu_Ploss(count)-30-10*log10(NoisePower);
                    imu_STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',imu_SNR(count))));
                    if imu_STASnr_rndd(count) > 45
                        imu_STASnr_rndd(count) = 45;
                    end
                    if imu_STASnr_rndd(count) < -35
                        imu_STASnr_rndd(count) = -35;
                    end  
                    % MCS calculation for AX
                    locate_snr_classical = find(snr_range==imu_STASnr_rndd(count));
                    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                    if isempty(best_mcs_classical_temp)
                        best_mcs_classical_temp = 1;
                    else
                        best_mcs_classical_temp = best_mcs_classical_temp(end);
                    end
                    imu_best_mcs_classical(count) = best_mcs_classical_temp -1;

                    % Packet error rate with AX
                    imu_AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);
                end
            else
                if imu_STASnr_rndd(count-1) < 20   % Re-association
                    [minValue_xRobot,imu_closestIndex_xRobot] = min(abs(estPosition(1)-AP_x));
                    [minValue_yRobot,imu_closestIndex_yRobot] = min(abs(estPosition(2)-AP_y));
                    imu_distance_closestAP(count) = sqrt(((estPosition(1)- AP_x(imu_closestIndex_xRobot))^2)+(estPosition(2)- AP_y(imu_closestIndex_yRobot))^2);
                    imu_Ploss(count) = propagation_loss (imu_distance_closestAP(count),Params);     % pathloss vs. distance + fixed shadow fading
                    imu_SNR(count) = txPower-imu_Ploss(count)-30-10*log10(NoisePower);
                    imu_STASnr_rndd(count) = ceil(str2num(sprintf('%.1f',imu_SNR(count))));
                    if imu_STASnr_rndd(count) > 45
                        imu_STASnr_rndd(count) = 45;
                    end
                    if imu_STASnr_rndd(count) < -35
                        imu_STASnr_rndd(count) = -35;
                    end  
                    % MCS calculation for AX
                    locate_snr_classical = find(snr_range==imu_STASnr_rndd(count));
                    best_mcs_classical_temp = find(snr_per_mcs_5GHz_indoor_channelB(:,locate_snr_classical) <= 0.001);
                    if isempty(best_mcs_classical_temp)
                        best_mcs_classical_temp = 1;
                    else
                        best_mcs_classical_temp = best_mcs_classical_temp(end);
                    end
                    imu_best_mcs_classical(count) = best_mcs_classical_temp -1;

                    % Packet error rate with AX
                    imu_AX_per(count) = snr_per_mcs_5GHz_indoor_channelB(best_mcs_classical_temp,locate_snr_classical);

                    if imu_AX_per(count) > 0.001        % Simulating packet drop due to high PER
                        accelData = accelData*rand; 
                        gyroData = gyroData*rand;   
                    end
                end
            end
        end

        % Use the predict method to estimate the filter state based
        % on the accelData and gyroData arrays.
        predict(gndFusion, accelData, gyroData);

        % Log the estimated orientation and position.
        [estPosition, estOrientation] = pose(gndFusion);

        estPosition = estPosition.'; 
        disp("estPosition")
        disp(estPosition)

        eul_angle = max(quat2eul(estOrientation));

        if eul_angle == 0
            eul_angle = -max(abs(quat2eul(estOrientation)));
        end

        if estPosition(1) > 3 && estPosition(1) < 7 && estPosition(2) > -25.5 && estPosition(2) < 25
            eul_angle = -1.5708;
        end

        % This next step happens at the GPS sample rate.
        % Simulate the GPS output based on the current pose.
%         [lla, gpsVel] = gps(truePosition, trueVel);
%         
%         if isnan(lla)
%             lla = [0,0,0];
%         end
%         
%         if isnan(gpsVel)
%             gpsVel = [0,0,0];
%         end
%         
% 
%         % Update the filter states based on the GPS data.
%         fusegps(gndFusion, lla, Rpos, gpsVel, Rvel);
    
    % Deviation from actual path with controller pure pursuit method
    [imu_minValue_x,imu_closestIndex_x(count)] = min(abs(estPosition(1)-dynamic_x));
    [imu_minValue_y,imu_closestIndex_y(count)] = min(abs(estPosition(2)-dynamic_y));
    deviation_fusion_method(count) = sqrt(((estPosition(1)- dynamic_x(imu_closestIndex_x(count)))^2)+(estPosition(2)- dynamic_y(imu_closestIndex_y(count)))^2);
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize, "MeshColor", "blue");
    light;
    xlim([-30 30])
    ylim([-30 30])
    grid on
    plotTrVec1 = [estPosition(1:2); 0];
    plotRot1 = axang2quat([0 0 1 eul_angle]);
    plotTransforms(plotTrVec1', plotRot1, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize, "MeshColor", "green");
    plot(AP_x,AP_y,'r*');
    plot(AP_x(imu_closestIndex_xRobot),AP_y(imu_closestIndex_yRobot),'g*');
    plot(AP_x(closestIndex_xRobot),AP_y(closestIndex_yRobot),'b*');
    
    waitfor(vizRate);
end

%% Error Metric Computation
% Position and orientation were logged throughout the simulation. Now 
% compute an end-to-end root mean squared error for both position and 
% orientation.

posd = estPosition - truePosition;

% min_x = min(min(groundTruth.Waypoints(:,1)),min(estPosition(:,1)));
% max_x = max(max(groundTruth.Waypoints(:,1)),max(estPosition(:,1)));
% min_y = min(min(groundTruth.Waypoints(:,2)),min(estPosition(:,2)));
% max_y = max(max(groundTruth.Waypoints(:,2)),max(estPosition(:,2)));
% 
% figure()
% hold on
% grid on
% xlim([-30 30]);
% ylim([-30 30]);
% plot(groundTruth.Waypoints(:,1),groundTruth.Waypoints(:,2))
% plot(estPosition(:,1),estPosition(:,2))
% plot(estPosition(1,1),estPosition(1,2), 'b*')
% hold off


