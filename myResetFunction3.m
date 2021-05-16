function [InitialObservation, LoggedSignal] = myResetFunction3()
% Reset function to place custom quadrotor environment into a random

% Set global variables
global Step State Tau_vec PATH P Time Action_hist Fext Fext_hist Mext external_callback reward_accum;

Action_hist = [];
Fext_hist = zeros(3,1);
reward_accum = 0;
external_callback = true;
% Random force with random direction;
Fext = 0 * rand * random_unit_vector; % 0~1N (about apple weight)

% Random moment with random direction;
% Mext = 0 * rand * random_unit_vector; % 0~1Nm
Mext = 0 * rand * [rand, rand, 0]'/norm([rand, rand, 0]);
Step = 0;
State = zeros(12,1);
Time = 0;

vel_max = 4;
vel_min = 1;

% target = (2*rand(1,3)-1)*50;
% tau_min = norm(target)/vel_max;
% tau_max = norm(target)/vel_min;
% PATH = [zeros(1,3); target];
% Tau_vec = (tau_max-tau_min)*rand + tau_min;

Tau_vec = 9;
dist_min = Tau_vec * vel_min;
dist_max = Tau_vec * vel_max;
dist = (dist_max-dist_min)*rand + dist_min;
target = dist * random_unit_vector;
PATH = [zeros(1,3); target'];

% Tau_vec = 10 + rand;
% PATH = [0, 0, 0; 5, 0, 0];

% Tau_vec = 5*ones(8,1);
% PATH = [0,0,0;...
%         1,-1,0;...
%         2,0,0;...
%         1,1,0;...
%         0,0,0;...
%         -1,-1,0;...
%         -2,0,0;...
%         -1,1,0;...
%         0,0,0]*5;

traj = MinimumSnapTrajectory(Tau_vec, PATH);
P = traj.P;

% initial state.
pos_e0 = zeros(3,1);
vel_e0 = zeros(3,1);
acc_e0 = zeros(3,1);
R0 = ROTZ(State(9))*ROTX(State(7))*ROTY(State(8));
ang_vel_e0 = zeros(3,1);
s0 = [pos_e0;vel_e0;acc_e0;R0(:);ang_vel_e0];
LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;

end

