function [InitialObservation, LoggedSignal] = myReset()
% Reset function to place custom quadrotor environment into a random

% Set global variables
global Step Time State Traj Action_hist Fext Fext_hist Mext external_callback;

Action_hist = zeros(4,1);
Fext_hist = zeros(3,1);
external_callback = true;

% Random force with random direction;
Fext = 1 * rand * random_unit_vector; %

% Random torque with random direction on x-y plane;
rand_unit_vec = random_unit_vector;
rand_unit_vec(3) = 0;
rand_unit_vec = rand_unit_vec/norm(rand_unit_vec);
Mext = 0.263 * rand * rand_unit_vec;

vel_min = 1;
vel_max = 5;
vel = (vel_max-vel_min)*rand + vel_min;

tau_vec = 9;
dist = vel*tau_vec;
target = dist * random_unit_vector;
path = [zeros(1,3); target'];

% % Random aerobatic trajectory generation
% % [tau_vec, path] = randomTrajectorySelector();
% [tau_vec, path] = loopTheLoop();

% Trajectory
Traj = MinimumSnapTrajectory(tau_vec, path);

% Initial state
Step = 0;
State = zeros(12,1);
State(1:3) = 0.1 * rand(3,1);
State(4:6) = 0.1 * rand(3,1);
State(7:9) = 10/180*pi * rand(3,1);
State(10:12) = 10/180*pi * rand(3,1);
Time = 0;

% Desired initial state
desired_state0 = desiredState(Traj, 0);

% initial state.
pos_e0 = State(1:3) - desired_state0.pos;
vel_e0 = State(4:6) - desired_state0.vel;
acc_e0 = zeros(3,1) - desired_state0.acc;

R0 = ROTZ(State(9))*ROTX(State(7))*ROTY(State(8));
quat0 = rotm2quat(R0);
ang_vel_e0 = State(10:12);

s0 = [pos_e0;vel_e0;acc_e0;quat0';ang_vel_e0];

LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;

end