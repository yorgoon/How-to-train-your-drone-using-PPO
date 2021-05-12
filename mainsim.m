close all;
clear;
clc
addpath(genpath('./'));
%% Path Plan
% Obstacle parameters
E.s1 = [2, 2]*0.8;
E.r = [2.5, 1.5, 0;7 -0.5 0];
% Initial position
E.init = [0 0 0];
% Goal position
E.goal = [10 0.9 0];
% Gain parameters
E.Kob = 15; % Obstacle cost gain
E.Kt = 50; % Time penalty gain
E.Kv = 5; % Velocity gain

options = optimoptions(@fminunc,'Algorithm','quasi-newton',...
    'MaxIterations', 20000, 'MaxFunctionEvaluations', 20000);
% Initialize
wp0 = [2.1 0.2 0 6.1 0.4 0 1 1 1];
[wp, fval, exitflag,output] = fminunc(@(x)computeCost2(x, E), wp0, options);

%% Plot Optimized Trajectory
plot_traj(wp, E)

tau_vec = wp(end-2:end)';
wp_temp = wp;
wp_temp(end-2:end) = [];
wp_temp = reshape(wp_temp,[3,2]);
wp_temp = wp_temp';
PATH = [E.init;wp_temp;E.goal];
traj_class = MinimumSnapTrajectory(tau_vec, PATH);
P = traj_class.P;
%% EKF Simulation
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
S.g = 9.81; % Gravity

% Control Parameters
K.Kp = 11.9;
K.Kv = 4.443;
K.KR = .1;
K.K_omega = 1;
% K.Kp = 1;
% K.Kv = 7;
% K.KR = .03;
% K.K_omega = 1.5;
S.K = K;
S.dt = 1/100;
ts = 0:S.dt:15; % Time steps
% State dimension
S.n = 16;
% Input dimension
S.m = 8;
% Initialize True State
state_true = zeros(S.n, length(ts));
% state_true(1,1) = -0.5;
state_true(7,1) = pi/2;
% state_true(8,1) = pi/4;
% Initialize State Estimate
state_est = state_true;
% Uncertainty parameters
% mu = 0;
% sigma_xyz = .1;
% sigma_rpy = .01;
% sigma_q = .05;
Q1 = blkdiag(1e-8*eye(3),7e-11*eye(3),1e-8*eye(2));
Q2 = blkdiag(1e-8*eye(3),7e-11*eye(3),1e-8*eye(2));
% Q1 = blkdiag(1e-5*eye(3),1e-6*eye(3),1e-4*eye(2));
% Q2 = blkdiag(1e-6*eye(3),1e-6*eye(3),1e-4*eye(2));
S.Q = blkdiag(Q1,Q2);
% Q = 1e-10*eye(16);
% P1 = blkdiag(0.000000001*eye(3),0.0000000000001*eye(3),0.00000001*eye(2));
% P2 = blkdiag(0.000000001*eye(3),0.0000000000001*eye(3),0.00000001*eye(2));
% P = blkdiag(P1,P2);
S.P = 100*S.Q;
R1 = blkdiag(1e-4*eye(3),1e-3*eye(3),1e-6*eye(2));
R2 = blkdiag(1e-4*eye(3),1e-3*eye(3),1e-6*eye(2));
S.R = blkdiag(R1,R2);

% Initial Guess
state_est(:,1) = state_true(:,1) + sqrt(S.P) * randn(16,1);

% history of state estimate covariance
Ps(:, :, 1) = S.P;
x_true = state_true(:,1);
x_predict = state_est(:,1);
x_correct = state_est(:,1);
u_hist = zeros(S.m,length(ts));
U1 = [];
U2 = [];
U3 = [];
U4 = [];
arm_fold = false;
% Simulation
for i=1:length(ts)-1
	t = ts(i);
    if mod(t,1) == 0
        formatSpec = "Current simulation time: %d %s";
        A2 = 'seconds';
        str = sprintf(formatSpec,t,A2);
        fprintf(str + '\n')
    end
    
%     input = controller(x_true,t,S); % Controller using true state
    ddd = desired_state_optimal(tau_vec, t, PATH, P);
%     [u1, u2, u3, u4] = geometric_control(x_true, ddd, S); % True state

    input = geometric_control(x_correct, ddd, S);
%     input = input1 + sqrt(Q1)*randn(8,1); % correct state
%     input_arm = controller(x_correct, t, S);
%     input = geometric_control(x_true, ddd, S); % correct state
%     input_arm = controller(x_true, t, S);
%     U1(i) = u1;
%     U2(i) = u2;
%     U3(i) = u3;
%     U4(i) = u4;
%     input = [input;0;0];
%     input = [0 0 u1 u2 u3 u4 0 0]';
%     R = eulerZYX(x_true(4:6));
%     Q_inv = ...
%     [1, (sin(x_true(5))*sin(x_true(4)))/cos(x_true(5)), (cos(x_true(4))*sin(x_true(5)))/cos(x_true(5));
%      0, cos(x_true(4)), -sin(x_true(4));
%      0, sin(x_true(4))/cos(x_true(5)), cos(x_true(4))/cos(x_true(5))];
%     R_bar = blkdiag(R, Q_inv, eye(2));
%     input = R_bar * input;
%     input = controller(x_predict,t,S); % Controller using a priori estimate
%     input = controller(x_correct,t,S); % Controller using posterior estimate
%     input = [input(1:6);input_arm(7:8)];
    u_hist(:,i) = input;
    
    % True evolution of dynamics
    x_true = discreteDynamics(x_true, input, S);
    if ~arm_fold
        x_true(7) = pi/2;
        x_true(15) = 0;
        x_true(8) = 0;
        x_true(16) = 0;
    end
    if norm(x_correct(1:3)- PATH(end,:)') < 0.03 && ~arm_fold
        arm_fold = true;
        T = t;
    end
    if arm_fold
        t = t - T;
        if t < 5
            x_true(7) = pi/2 - pi/4*1/2*(sin(pi/5*t-pi/2)+1);
            x_true(15) = -pi/4*1/2*cos(pi/5*t-pi/2)*pi/5;
            x_true(8) = 45/180*pi*1/2*(sin(pi/5*t-pi/2)+1);
            x_true(16) = 45/180*pi*1/2*cos(pi/5*t-pi/2)*pi/5;
        else
            x_true(7) = pi/4;
            x_true(15) = 0;
            x_true(8) = pi/4;
            x_true(16) = 0;
        end
    end
    x_true = x_true + sqrt(S.Q) * randn(16,1);
    state_true(:,i+1) = x_true;
    
    % Prediction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     x_predict = ekf_predict(x_predict, input, S);
%     x_predict = discreteDynamics(x_predict, input, S);
    x_predict = ekf_predict(x_correct, input, S);
    % Correction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z = x_true + sqrt(S.R) * randn(16,1);
    x_correct = ekf_correct(x_predict, z, S);
%     x_correct = x_predict;
    % Update history
    state_est(:,i+1) = x_correct;
    Ps(:, :, i) = S.P;
end
disp('Simulation completed.')

% Plot Results
plot_allstuff(state_true(:,1:end-1)', state_est(:,1:end-1)', u_hist(:,1:end-1), ts(1:end-1), S)

%% Plot Covariance
figure(5)
for i=1:length(Ps(:,1,1))
    covar = Ps(i,i,:);
    plot(ts(1:end-1), covar(:))
    hold on
    grid on
end
%% Video Generator (Need to Keep Figure 1, otherwise rerun line 24)
% External force
start_T = 0;
duration = 0;
mag = 0;
direction = [1 0 0]';
Fmat = extForce_gen(ts, start_T, duration, mag, direction);
% figure(1)
% plot3(10.6036, 0.9, -0.4536,'.g','MarkerSize',50);
filename = 'myVideo.avi';
video_gen2(ts(1:end-1), state_true(:,1:end-1)', S, filename, 30)
% close all