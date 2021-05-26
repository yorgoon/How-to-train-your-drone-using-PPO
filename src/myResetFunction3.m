function [InitialObservation, LoggedSignal] = myResetFunction3()
% Reset function to place custom quadrotor environment into a random

% Set global variables
global Step State Tau_vec PATH P Time Action_hist Fext Fext_hist Mext external_callback reward_accum vel;

Action_hist = zeros(4,1);
Fext_hist = zeros(3,1);
reward_accum = 0;
external_callback = true;
% Random force with random direction;
Fext = 0 * rand * random_unit_vector; % 0~1N (about apple weight)

% Random moment with random direction;
% Mext = 0 * rand * random_unit_vector; % 0~1Nm
rand_unit = [rand, rand, 0]';
rand_unit = rand_unit/norm(rand_unit);
Mext = 0 * rand * rand_unit;
Step = 0;
State = zeros(12,1);
State(1:3) = 0.1 * rand(3,1);
State(4:6) = 0.1 * rand(3,1);
State(7:9) = 10/180*pi * rand(3,1);
State(10:12) = 10/180*pi * rand(3,1);
Time = 0;


% target = (2*rand(1,3)-1)*50;
% tau_min = norm(target)/vel_max;
% tau_max = norm(target)/vel_min;
% PATH = [zeros(1,3); target];
% Tau_vec = (tau_max-tau_min)*rand + tau_min;

% random_unit_vec = random_unit_vector;
% random_unit_vec(3) = abs(random_unit_vec(3));
% 
% vel_min = 1;
% vel_max = 10 + 10*(1-abs(random_unit_vec(3)));
% vel = (vel_max-vel_min)*rand + vel_min;
% 
% Tau_vec = 10;
% dist = vel*Tau_vec;
% target = dist * random_unit_vec;
% PATH = [zeros(1,3); target'];

% Front flip
% r = [1, 1.3, 1.5, 1.7, 1.5, 1.3, 1];
% r = 1.5;
% pre = 20;
% gamma = 100000;
% theta = linspace(-pi/2,2*pi-pi/2,7);
% x = r.*cos(theta);
% x = x+pre;
% z = r.*sin(theta);
% z = z+r;
% PATH = [zeros(1,3);x',zeros(length(x),1),z';pre*2,0,0];
% Tau_vec = desired_trajectory(PATH, gamma)';
% Tau_vec = [2.2786 0.2400 0.3792 0.3740 0.3740 0.3792 0.2400 2.2786]';
% Tau_vec = [2.3885 0.2174 0.3832 0.4577 0.4577 0.3832 0.2174 2.3885]';

% Spiral roll
r = 1;
roll_num = 3;
pre = 10;
theta = linspace(-pi/2,roll_num*2*pi-pi/2,roll_num*4+1);
x = r.*cos(theta);
y = linspace(pre,2*pre,length(theta));
z = r.*sin(theta)+r;
PATH1 = [x',y',z';5,y(end)+1,0];
PATH1 = PATH1+[5,-9,0];
PATH = [zeros(1,3);PATH1];
Tau_vec = zeros(length(PATH)-1,1);
Tau_vec(2:end-1) = 0.35;
Tau_vec(1) = 1.6;
Tau_vec(end) = 1.6;

% Trajectory
traj = MinimumSnapTrajectory(Tau_vec, PATH);
P = traj.P;

% Desired initial state
desired_state0 = desired_state_optimal(Tau_vec, Time, PATH, P);

% initial state.
pos_e0 = State(1:3) - desired_state0.pos;
vel_e0 = State(4:6) - desired_state0.vel;
acc_e0 = zeros(3,1) - desired_state0.acc;

R0 = ROTZ(State(9))*ROTX(State(7))*ROTY(State(8));
quat0 = rotm2quat(R0);
ang_vel_e0 = State(10:12);

% s0 = [pos_e0;vel_e0;acc_e0;R0(:);ang_vel_e0];
s0 = [pos_e0;vel_e0;acc_e0;quat0';ang_vel_e0];

LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;

end

