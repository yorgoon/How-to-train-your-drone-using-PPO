function [InitialObservation, LoggedSignal] = myResetFunction3()
% Reset function to place custom quadrotor environment into a random

% Set global variables
global Step State Tau_vec PATH P Time Action_hist Fext Fext_hist Mext external_callback;

Action_hist = [];
Fext_hist = [];

external_callback = true;
% Random force with random direction;
Fext = 0 * rand * random_unit_vector; % 0~1N (about apple weight)

% Random moment with random direction;
Mext = 0 * rand * random_unit_vector; % 0~1Nm

Step = 0;
State = zeros(12,1);
Time = 0;
Tau_vec = 5+5*rand();
PATH = [0, 0, 0; (2*rand(1,3)-1)*5];

% Tau_vec = 2*ones(4,1);
% PATH = [0,0,0;...
%         1,0,0;...
%         1,1,0;...
%         0,1,0;...
%         0,0,0];

% Tau_vec = 2*ones(5,1);
% PATH = [0,0,0;...
%         1,-1,0;...
%         2,0,0;...
%         1,1,0;...
%         0,0,0;...
%         1,-1,0];

% Tau_vec = 4*ones(8,1);
% PATH = [0,0,0;...
%         10,-10,0;...
%         20,0,0;...
%         10,10,0;...
%         0,0,0;...
%         -10,-10,0;...
%         -20,0,0;...
%         -10,10,0;...
%         0,0,0];

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

