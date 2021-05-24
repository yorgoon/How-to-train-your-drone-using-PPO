function [InitialObservation, LoggedSignal] = myResetFunction3()
% Reset function to place custom quadrotor environment into a random

% Set global variables
global Step State Tau_vec PATH P Time Action_hist Fext Fext_hist Mext external_callback reward_accum vel;

Action_hist = [];
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

% Barrel roll
r = 1.5;
roll_num = 3;
pre = 20;
% gamma = 200000;
theta = linspace(-pi/2,roll_num*2*pi-pi/2,roll_num*6);
x = r.*cos(theta);
y = linspace(pre,r*pre,length(theta));
z = r.*sin(theta)+r;
PATH = [zeros(1,3);x',y',z';0,y(end)+pre,0];
Tau_vec =[2.3281,...
    0.3914,...
    0.4626,...
    0.3838,...
    0.4027,...
    0.3797,...
    0.3867,...
    0.3796,...
    0.3800,...
    0.3775,...
    0.3774,...
    0.3747,...
    0.3798,...
    0.3731,...
    0.3957,...
    0.3793,...
    0.4603,...
    0.3933,...
    2.2897]';
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
ang_vel_e0 = State(10:12);

s0 = [pos_e0;vel_e0;acc_e0;R0(:);ang_vel_e0];
LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;

end

