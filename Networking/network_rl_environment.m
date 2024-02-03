clc; close all; clear all;

% Location of APs in the environment
AP_x = [-30 -20 -10 0 10 20 30 30 20 10 -10 -20 -30 0 0 0 0 0 0 -10 -20 -30 10 20 30 -20 -20 20 20 -30 -30 30 30 10 20 -10 -20 -20 -10 10 20 -10 -10 10 10 -30 -30 30 30];
AP_y = [-30 -20 -10 0 10 20 30 -30 -20 -10 10 20 30 -10 -20 -30 10 20 30 0 0 0 0 0 0 -10 10 -10 10 -10 10 -10 10 -30 -30 -30 -30 30 30 30 30 -20 20 20 -20 -20 20 -20 20];

% Create a GridWorld object of a 30X30 dimension, using the createGridWorld function.
gw = createGridWorld(20,20,'Kings');                        % if moves = 'Kings', then possible movement directions = ['N';'S';'E';'W';'NE';'NW';'SE';'SW']
                                                            % if moves = 'Standard', then possible movement directions = ['N';'S';'E';'W']

% Set the initial, terminal and obstacle states.
gw.CurrentState = '[20,1]';                                 % starting/spawning grid
gw.TerminalStates = '[1,20]';                               % target grid
% Grid location of obstacles
gw.ObstacleStates = ["[3,3]";"[3,4]";"[3,5]";"[4,3]";"[10,10]";"[2,3]";"[14,13]";"[19,20]";"[1,19]";"[19,1]";"[7,7]";"[8,13]";"[9,5]";"[10,12]"; ...
    "[15,12]";"[12,15]";"[9,16]";"[7,16]";"[2,17]";"[18,9]";"[10,19]";"[11,18]";"[12,17]";"[7,4]";"[8,3]";"[13,3]";"[14,4]";"[15,5]";"[16,6]"; ...
    "[2,11]";"[3,11]";"[4,11]";"[5,11]";"[1,1]"];     

% Update the state transition matrix for the obstacle states and set the jump rule over the obstacle states.
updateStateTranstionForObstacles(gw)
%gw.T(state2idx(gw,"[2,4]"),:,:) = 0;
%gw.T(state2idx(gw,"[2,4]"),state2idx(gw,"[4,4]"),:) = 1;

%  Define the rewards in the reward transition matrix.
nS = numel(gw.States);
nA = numel(gw.Actions);

% All actions result in +1 reward.
gw.R = 1*ones(nS,nS,nA);       

% The environment contains a special jump from cell [2,4] to cell [4,4] with a reward of +5.
%gw.R(state2idx(gw,"[2,4]"),state2idx(gw,"[4,4]"),:) = 5;

% The agent receives a reward +10 if it reaches the terminal state at cell [5,5] (blue).
gw.R(:,state2idx(gw,gw.TerminalStates),:) = 10;

% Initialize Observation settings
ObservationInfo = rlNumericSpec([4 1]);
ObservationInfo.Name = 'state';
ObservationInfo.Description = 'Profit, Distance, x, y';
            
% Initialize Action settings
% Two actions -- distance and angle. Both limited by the provided range
ActionInfo = rlNumericSpec([2 1],'LowerLimit',[0;0],'UpperLimit',[1;1]);
ActionInfo.Name = 'dist;angle';

% Environment constants
envConstants.Gravity = 9.8;                         % Acceleration due to gravity in m/s^2
envConstants.MassCart = 1.0;                        % Mass of the cart
envConstants.MassPole = 0.1;                        % Mass of the pole
envConstants.Length = 0.5;                          % Half the length of the pole
envConstants.MaxForce = 10;                         % Max force the input can apply
envConstants.Ts = 0.02;                             % Sample time
envConstants.ThetaThresholdRadians = 12 * pi/180;   % Angle at which to fail the episode
envConstants.XThreshold = 2.4;                      % Distance at which to fail the episode
envConstants.RewardForNotFalling = 1;               % Reward each time step the cart-pole is balanced
envConstants.PenaltyForFalling = -5;                % Penalty when the cart-pole fails to balance

% % Use rlMDPEnv to create a grid world environment using the GridWorld object GW.
% env = rlMDPEnv(gw);

%Creates a reinforcement learning environment using the provided observation and action specifications, obsInfo and actInfo, respectively. 
%You also set the StepFcn and ResetFcn properties using MATLAB functions.
env = rlFunctionEnv(ObservationInfo,ActionInfo,NetworkResetFunction,NetworkResetFunction);


[InitialObservation,LoggedSignals] = NetworkResetFunction();



plot(env);





