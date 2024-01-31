clc; close all; clear all;

%% Input number of robots for each navigation method

robot_PP_numbers = 4;   % Number of robots executing Pure Pursuit algorithm
robot_SF_numbers = 2;   % Number of robots executing IMU+GPS sensor fusion algorithm

%% Wireless Network aspects
% Load snr-per-mcs table
load snr_per_mcs_5GHz_indoor_channelB

% Initializing array for robot actual path tracing
pure_pursuit_pos = [];
fusion_pos = [];
STASnr_rndd = [];

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

% Location of APs in the environment
AP_x = [-30 -20 -10 0 10 20 30 30 20 10 -10 -20 -30 0 0 0 0 0 0 -10 -20 -30 10 20 30 -20 -20 20 20 -30 -30 30 30 10 20 -10 -20 -20 -10 10 20 -10 -10 10 10 -30 -30 30 30];
AP_y = [-30 -20 -10 0 10 20 30 -30 -20 -10 10 20 30 -10 -20 -30 10 20 30 0 0 0 0 0 0 -10 10 -10 10 -10 10 -10 10 -30 -30 -30 -30 30 30 30 30 -20 20 20 -20 -20 20 -20 20];

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

%% Define a set of waypoints for path #2

x2 = [-25 -22 -18 -15 -13 -10 -8 -5 -3 0 2 5 8 10];
y2 = [11 15 21 13 0 -15 -20 -25 -24 -22 -20 -15 -10 -6];
path_x2 = -25:0.5:10;
path_y2 = interp1(x2,y2,path_x2,'spline');
path_x2 = path_x2.';
path_y2 = path_y2.';
path1 = [path_x2 path_y2];


%% Controller based path correction

goalRadius = 0.1;
for i=1:robot_PP_numbers
    initialOrientation(i) = 0;
    robot(i) = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
    controller{i} = controllerPurePursuit;
    controller{i}.DesiredLinearVelocity = 0.8-(i/10); % Each robot gets a unique linear velocity
    controller{i}.MaxAngularVelocity = 2-(i/10);      % Each robot gets a unique angular velocity
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

while( distanceToGoal > goalRadius )
    count = count + 1; 
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
    
    for i = 1:robot_PP_numbers
        if mod(i,2) ~= 0
        % Update the current pose
            [robotCurrentPose{i},distanceToGoal,vel,closestIndex_xRobot(i),closestIndex_yRobot(i),STASnr_rndd] = PurePursuitPos_path(i,Params,robot(i),robotCurrentPose{i}, ...
                count,snr_per_mcs_5GHz_indoor_channelB,controller{i},AP_x,AP_y,STASnr_rndd,closestIndex_xRobot(i),closestIndex_yRobot(i));
            robotCurrentPose{i} = robotCurrentPose{i} + vel*sampleTime; 

    %         pure_pursuit_pos(:,:,i) = [pure_pursuit_pos; robotCurrentPose(1) robotCurrentPose(2)];

            % Plot the path of the robot as a set of transforms
            plotTrVec = [robotCurrentPose{i}(1:2); 0];
            plotRot = axang2quat([0 0 1 robotCurrentPose{i}(3)]);
            plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize(i), "MeshColor", "blue");
            plot(AP_x(closestIndex_xRobot(i)),AP_y(closestIndex_yRobot(i)),'b*');
    %         plot(pure_pursuit_pos(:,1,i), pure_pursuit_pos(:,2,i), '-b', 'LineWidth', 2);
        else 
            if mod(i,2) == 0
            % Update the current pose
            [robotCurrentPose{i},distanceToGoal,vel,closestIndex_xRobot(i),closestIndex_yRobot(i),STASnr_rndd] = PurePursuitPos_path1(i,Params,robot(i),robotCurrentPose{i}, ...
                count,snr_per_mcs_5GHz_indoor_channelB,controller{i},AP_x,AP_y,STASnr_rndd,closestIndex_xRobot(i),closestIndex_yRobot(i));
            robotCurrentPose{i} = robotCurrentPose{i} + vel*sampleTime; 

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
    
    waitfor(vizRate);
end

%% End
