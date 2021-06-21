function [NextObs, Reward, IsDone, LoggedSignals] = myStep(Action, LoggedSignals)
% Custom step function to construct quadrotor environment for the function
% name case.
%
% This function applies the given action to the environment and evaluates
% the system dynamics for one simulation step.
% Model parameters
S.mb = 1.477; % Mass of UAV
S.d = 0.263; % Arm Length (Rotor and COM of UAV)
S.c = 8.004e-4; % Drag Factor
S.Ib = [0.01152 0 0;0 0.01152 0;0 0 0.0218];  % Moment of Inertia of UAV
% S.m1 = 0.05; % Mass of First Link
% S.m2 = 0.05; % Mass of Second Link
% S.l1 = 0.5; % Length of First Link
% S.l2 = 0.5; % Length of Second Link
S.g = 9.807; % Gravity
global Step Time State Traj Fext Fext_hist Action_hist Zb
% global Mext external_callback reward_accum ;

% Sample time
ts = 0.01;
% Total time and step
total_time = sum(Traj.tau_vec);
total_step = total_time/ts;
% Advance time and step
Time = Time + ts; Step = Step + 1;

% External force
% if Time > 1 && Time <= pi/2+1
%     Fext = sin(Time-1) * [0,0,-1]';
% elseif Time > pi/2+1
%     Fext = [0,0,-1]';
% end

% Current state
state = State(:,end);

% Desired state
desired_state = desiredState(Traj, Time);

% Conversion of action to double precision
Action = double(Action);
% Geometric controller
% Action = reference_controller(state, desired_state, S);

if any(isnan(Action(:)))
    error('going nuts')
end

state_dot = droneDynamics(state, Action, S);

% Update state (Euler integration)
state = state + ts * state_dot;

% Position error
pos_error = state(1:3) - desired_state.pos;

% Velocity error
vel_error = state(4:6) - desired_state.vel;

% Accel. error
acc_error = state_dot(4:6) - desired_state.acc;

% Update rotation
R = ROTZ(state(9))*ROTX(state(7))*ROTY(state(8));
quat = rotm2quat(R);

% Update Log
LoggedSignals.State = [pos_error;vel_error;acc_error;quat';state(10:12)];
% LoggedSignals.State = [pos_error;vel_error;acc_error;quat';state(10:12);Action];
NextObs = LoggedSignals.State;

% Concat state
State = [State, state];
% Concat force
Fext_hist = [Fext_hist, Fext];
% Concat action
Action_hist = [Action_hist ,Action];

% Error norms
pos_l2 = norm(pos_error);
vel_l2 = norm(vel_error);
acc_l2 = norm(acc_error);
yaw_error = abs(state(9));

% Body axes
% xb = R(:,1);
% yb = R(:,2);
% zb = R(:,3);

% Rewards
r_pos = exp(-(1/0.5*pos_l2));
r_vel = exp(-(1/1*vel_l2));
r_acc = exp(-(1/1*acc_l2));
r_yaw = exp(-(1/(5/180*pi)*yaw_error));

% % desired vel and acc angle
% vel_acc_rad = acos(getCosineSimilarity(desired_state.vel,desired_state.acc));
% 
% if Time/total_time > 0.5 && vel_acc_rad > 100/180*pi
%     x_cos = acos(getCosineSimilarity(xb,[-1,0,0]'));
%     r_xb = exp(-(1/(90/180*pi)*x_cos).^2);
% else
%     x_cos = acos(getCosineSimilarity(xb,desired_state.vel));
%     r_xb = exp(-(1/(30/180*pi)*x_cos).^2);
% end
% 
% if Time > sum(Traj.tau_vec(1:end-1)) && vel_acc_rad > 90/180*pi
%     z_cos = acos(getCosineSimilarity(zb,[0,0,1]'));
%     r_zb = exp(-(1/(90/180*pi)*z_cos).^2);
% else
%     z_cos = acos(getCosineSimilarity(zb,desired_state.acc));
%     r_zb = exp(-(1/(30/180*pi)*z_cos).^2);
% end

rewards = [0.6 0.1 0.1 0.2] .* [r_pos r_vel r_acc r_yaw];
% fprintf('%.2f\n',r_yaw)
% rewards = [0.6 0.1 0.1 0.1 0.1] .* [r_pos r_vel r_acc r_xb r_zb];

% Termination condition and reward
IsDone = false;
reward_terminate = 0;

if pos_l2 > 0.5*3 || yaw_error > 5*3/180*pi
    IsDone = true;
    reward_terminate = -1;
end

% if Time > Traj.tau_vec(1)
%     if vel_acc_rad < 90/180*pi && abs(x_cos) > pi/4
%         IsDone = true;
%         reward_terminate = -1;
%     end
%     if Time < sum(Traj.tau_vec(1:end-1)) && abs(z_cos) > pi/3
%         IsDone = true;
%         reward_terminate = -1;
%     end
% end
% 
% % Stabilizing ET
% if Time>sum(Traj.tau_vec(1:end-1)) && vel_acc_rad > 100/180*pi
%     if zb(3) > 0 && ~Zb
%         Zb = true;
%     end
% end
% 
% if Zb && zb(3) < 0
%     disp('spinny spinny')
%     IsDone = true;
%     reward_terminate = -1;
% end

% IsDone = false; % For test

if Time >= total_time+1 && ~IsDone
%     IsDone = true;
    State = State(:,end);
    State(1:3) = pos_error;
    Time = 0;
    tau_vec = 9;
    vel_min = 1; vel_max = 5;
    vel = (vel_max-vel_min)*rand + vel_min;
    dist = vel*tau_vec;
    target = dist * random_unit_vector;
    path = [zeros(1,3); target'];
    Traj = MinimumSnapTrajectory(tau_vec, path);
    Fext = 1 * rand * random_unit_vector;
end

if Step == 1e+6
    IsDone=true;
end

if ~IsDone
    Reward = sum(rewards);
else
    Reward = reward_terminate;
end

end