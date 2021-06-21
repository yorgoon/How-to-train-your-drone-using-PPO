function [InitialObservation, LoggedSignal] = myReset()
% Reset function to place custom quadrotor environment into a random

% Set global variables
global Step Time State Traj Action_hist Fext Fext_hist Mext Zb;
% Model parameters
S.g = 9.807; % Gravity
S.mb = 1.477; % Mass of UAV
thrust = (S.mb)*S.g/4;
Action_hist = [];
Fext_hist = zeros(3,1);
Zb = false;
% Random force with random direction;
Fext = 1 * rand * random_unit_vector;

% Random torque with random direction on x-y plane;
rand_unit_vec = random_unit_vector;
rand_unit_vec(3) = 0;
rand_unit_vec = rand_unit_vec/norm(rand_unit_vec);
Mext = 0 * rand * rand_unit_vec;

% Domain randomization
tau_vec = 9;
vel_min = 1; vel_max = 5;
vel = (vel_max-vel_min)*rand + vel_min;
dist = vel*tau_vec;
target = dist * random_unit_vector;
path = [zeros(1,3); target'];

% path = [0,0,0;1,1,0;2,0,0;1,-1,0;0,0,0;-1,1,0;-2,0,0;-1,-1,0;0,0,0]*2;
% tau_vec = timeAllocation(path,100)';
% 
% [tau_vec, path] = splitS();

% Trajectory
Traj = MinimumSnapTrajectory(tau_vec, path);

% Initial state
Step = 0; Time = 0;
State = zeros(12,1);

% Desired initial state
desired_state0 = desiredState(Traj, Time);

% initial state.
pos_e0 = State(1:3) - desired_state0.pos;
vel_e0 = State(4:6) - desired_state0.vel;
acc_e0 = zeros(3,1) - desired_state0.acc;

R0 = ROTZ(State(9))*ROTX(State(7))*ROTY(State(8));
quat0 = rotm2quat(R0);
ang_vel_e0 = State(10:12);

s0 = [pos_e0;vel_e0;acc_e0;quat0';ang_vel_e0];
% s0 = [pos_e0;vel_e0;acc_e0;quat0';ang_vel_e0;thrust*ones(4,1)];
LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;
end