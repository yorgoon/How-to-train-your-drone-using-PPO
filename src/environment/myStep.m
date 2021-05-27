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
S.m1 = 0.05; % Mass of First Link
S.m2 = 0.05; % Mass of Second Link
S.l1 = 0.5; % Length of First Link
S.l2 = 0.5; % Length of Second Link
S.g = 9.807; % Gravity
global Step State Time Tau_vec PATH P Action_hist Fext Fext_hist vel;
% global Mext external_callback reward_accum;

% Sample time
ts = 0.01;

Step = Step + 1;

% Total execution time
total_time = sum(Tau_vec);

% Update time
Time = Time + ts;

% Current state
state = State(:,end);

% Desired state
desired_state = desired_state_optimal(Tau_vec, Time, PATH, P);

% % Reference controller (Comment out)
% action_ref_control = reference_controller(state, desired_state, S);
% Action = action_ref_control; 
state_dot = droneDynamics(state, Action, S);

% Update state
state = state + ts*state_dot;
Action_hist = [Action_hist ,Action];
Fext_hist = [Fext_hist ,Fext];

% Position error
pos_error = state(1:3) - desired_state.pos;

% Velocity error
vel_error = state(4:6) - desired_state.vel;

% Accel. error
acc_error = state_dot(4:6) - desired_state.acc;

% Update rotation
R = ROTZ(state(9))*ROTX(state(7))*ROTY(state(8));
quat = rotm2quat(R);
% Angular acceleration
% ang_acc = state_dot(end-2:end);

% Update Log
% LoggedSignals.State = [pos_error;vel_error;acc_error;state(7:12)];
% LoggedSignals.State = [pos_error;vel_error;state(7:12)];
% LoggedSignals.State = [pos_error;vel_error;acc_error;R(:);state(10:12)];
LoggedSignals.State = [pos_error;vel_error;acc_error;quat';state(10:12)];
% Transform state to observation.
NextObs = LoggedSignals.State;

% Concat State
State = [State, state];

% Penalties
pos_l2 = norm(pos_error);
vel_l2 = norm(vel_error);
acc_l2 = norm(acc_error);
yaw_error = abs(state(9));
% omega_z = abs(state(12));
% omega_z_dot = abs(state_dot(end));
% omega_l2 = norm(state(10:12));
% std_action = std(Action);
% action_l2 = norm(Action);
% Incentive on flipping (z_axis pointing downward)
z_axis = R(:,3);
% Cosine similarity between z_axis and acceleration direction
if desired_state.acc == zeros(3,1)
    z_cos = acc_l2;
else
%     z_cos = 1-getCosineSimilarity(z_axis,desired_state.acc);
    z_cos = acos(getCosineSimilarity(z_axis,desired_state.acc));
end


% Rewards
% tau_pos = 0.35/2.5*vel;
% tau_vel = 1.5/2.5*vel;
% tau_acc = 3/2.5*vel;
% tau_yaw = 5*pi/180/2.5*vel;
% r_pos = exp(-(1/tau_pos * pos_l2)^2);
% r_vel = exp(-(1/tau_vel * vel_l2)^2);
% r_acc = exp(-(1/tau_acc * acc_l2)^2);
% r_yaw = exp(-(1/tau_yaw * yaw_error)^2);
% r_pos = exp(-(1/0.35 * pos_l2).^2);
% r_vel = exp(-(1/1.5 * vel_l2).^2);
% r_acc = exp(-(1/3 * acc_l2).^2);
% r_yaw = exp(-(1/(5*pi/180) * yaw_error).^2);
% r_z_axis = exp(-(1/0.5 * z_cos).^2);

r_pos = betaReward(pos_l2, 0.5);
r_vel = betaReward(vel_l2, 1.5);
r_acc = betaReward(acc_l2, 3);
r_yaw = betaReward(yaw_error, 5*pi/180);
r_z_axis = betaReward(z_cos, 45/180*pi);

rewards = [0.4 0.1 0.1 0.4] .* [r_pos r_vel r_acc r_z_axis];

fprintf('r,e: %f %f %f %f %f| %f %f %f %f %f\n',r_pos,r_vel,r_acc,r_yaw,r_z_axis, pos_l2,vel_l2,acc_l2,yaw_error*180/pi,z_cos)
% fprintf('Actions: %f %f %f %f\n',Action(1),Action(2),Action(3),Action(4))

% Termination reward
reward_terminate = 0;

% Check termination condition
IsDone = total_time <= Time;

if pos_l2 > 1
    IsDone = true;
    reward_terminate = -1;
end

if ~IsDone
    Reward = sum(rewards);
else
    Reward = reward_terminate;
end

end
