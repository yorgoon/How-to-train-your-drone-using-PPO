function [NextObs, Reward, IsDone, LoggedSignals] = myStepFunction(Action, LoggedSignals)
% Custom step function to construct quadrotor environment for the function
% name case.
%
% This function applies the given action to the environment and evaluates
% the system dynamics for one simulation step.

% Model parameters
S.dt = 0.01; % Sample time
S.mb = 1.477; % Mass of UAV
S.d = 0.263; % Arm Length (Rotor and COM of UAV)
S.c = 8.004e-4; % Drag Factor
S.Ib = [0.01152; 0.01152; 0.0218]; % Moment of Inertia of UAV
% S.Ib = [0.01152 0 0;0 0.01152 0;0 0 0.0218];
S.m1 = 0.05; % Mass of First Link
S.m2 = 0.05; % Mass of Second Link
S.l1 = 0.5; % Length of First Link
S.l2 = 0.5; % Length of Second Link
S.g = 9.81; % Gravity

% Control Parameters
K.Kp = 11.9;
K.Kv = 4.443;
K.KR = .1;
K.K_omega = 1;
S.K = K;

% Threshold to terminate episode
distance_Threshold = 0.5;

% Check if the given action is valid

% Unpack the state vector from the logged signals.
state = LoggedSignals.State;

% Arm length
L = S.d;

% Drag factor
c_tf = S.c;

% Penalty on different thrust
penalty_action = var(Action);

% Map thrust action into force, moment actions.
mapping_u = [1 1 1 1;0 L 0 -L;-L 0 L 0;c_tf -c_tf c_tf -c_tf];
Action = mapping_u * Action;

% Map actions into xyz rpy q1 q2 actions
R = eulerZYX(state(4:6));
Q_inv = ...
[1, (sin(state(5))*sin(state(4)))/cos(state(5)), (cos(state(4))*sin(state(5)))/cos(state(5));
 0, cos(state(4)), -sin(state(4));
 0, sin(state(4))/cos(state(5)), cos(state(4))/cos(state(5))];
R_bar = blkdiag(R, Q_inv, eye(2));

input = R_bar * [0;0;Action;0;0];

% desired_state.pos = [0,0,0]';
% desired_state.vel = [0,0,0]';
% desired_state.acc = [0,0,0]';
% desired_state.jerk = [0,0,0]';
% desired_state.snap = [0,0,0]';
% desired_state.yaw = 0;
% desired_state.yawdot = 0;
% desired_state.yawddot = 0;

% input_teacher = geometric_control(state, desired_state, S);

% Perform Euler integration.
next_state = discreteDynamics(state, input, S);
next_state(7) = pi/2;
next_state(8) = 0;
next_state(15) = 0;
next_state(16) = 0;
LoggedSignals.State = next_state;

% Transform state to observation.
NextObs = LoggedSignals.State;

% Check terminal condition.
IsDone = norm(NextObs(1:3)) > distance_Threshold;

% Penalty
pos_e = norm(NextObs(1:3))^2;
vel_e = norm(NextObs(4:6))^2;
ang_e = norm(NextObs(9:11))^2;
ang_vel_e = norm(NextObs(12:14))^2;

% Reward
reward_step = 1;

% Penalty for being terminated
penalty_terminate = -100;

% Penalty for being far from the origin
penalty = min(pos_e, distance_Threshold^2);

% Get reward.
if ~IsDone
    Reward = reward_step - penalty - penalty_action;
else
    Reward = penalty_terminate;
end

end
