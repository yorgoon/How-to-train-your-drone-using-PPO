function [NextObs, Reward, IsDone, LoggedSignals] = myStepFunction2(Action, LoggedSignals)
% Custom step function to construct quadrotor environment for the function
% name case.
%
% This function applies the given action to the environment and evaluates
% the system dynamics for one simulation step.
% Model parameters
S.mb = 1.477; % Mass of UAV
S.d = 0.263; % Arm Length (Rotor and COM of UAV)
S.c = 8.004e-4; % Drag Factor
S.Ib = [0.01152; 0.01152; 0.0218]; % Moment of Inertia of UAV
% S.Ib = [0.01152 0 0;0 0.01152 0;0 0 0.0218];
S.m1 = 0.05; % Mass of First Link
S.m2 = 0.05; % Mass of Second Link
S.l1 = 0.5; % Length of First Link
S.l2 = 0.5; % Length of Second Link
S.g = 9.807; % Gravity
% Sample time
ts = 0.01;
thrust = (S.mb)*S.g/4;
% Threshold to terminate episode
distance_threshold = 0.5;

% Reward
reward_step = 1;

% Penalty on different thrust - Forces to have a stable rotation
% penalty_action = -var(Action);

% Penalty for being terminated
penalty_terminate = -100;

% Unpack the state vector from the logged signals.
state = LoggedSignals.State;


sdot = droneDynamics(state, Action);

% Perform Euler integration.
LoggedSignals.State = state + ts.*sdot;

% Transform state to observation.
NextObs = LoggedSignals.State;

pos_e = norm(NextObs(1:3));
mapping_R = [cos(state(8)) 0 -cos(state(7))*sin(state(8));...
             0         1            sin(state(7));...
             sin(state(8)) 0  cos(state(7))*cos(state(8))];

% Check terminal condition.
IsDone = pos_e > distance_threshold | det(mapping_R) < 0.000001 | det(mapping_R) > 100000;

% Penalty for being far from the origin
penalty = -min(pos_e, distance_threshold^2);

% Penalty for rotating
% rot_e = norm(NextObs(10:12));
% penalty_rot = -rot_e;
yaw = state(9);
roll = state(7);
pitch = state(8);
R = ROTZ(yaw)*ROTX(roll)*ROTY(pitch); % Current rotation
% eR orientation error
R1 = R - R';

eR = 1/2*vee_optr(R1);
norm_eR = norm(eR)^2;

% Get reward.
if ~IsDone
    Reward = reward_step + 5*penalty - 3*norm_eR;
else
    Reward = penalty_terminate;
end

end
