function [NextObs, Reward, IsDone, LoggedSignals] = myStepFunction3(Action, LoggedSignals)
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

% Reference controller (Comment out)
% action_ref_control = reference_controller(state, desired_state, S);
% state_dot = droneDynamics(state, action_ref_control, S);
state_dot = droneDynamics(state, Action, S);

% Update state
% Guided training
% if Step <= 100
%     state(1:3) = desired_state.pos;
%     state(4:6) = desired_state.vel;
% end
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

% Angular acceleration
% ang_acc = state_dot(end-2:end);

% Update Log
% LoggedSignals.State = [pos_error;vel_error;acc_error;state(7:12)];
% LoggedSignals.State = [pos_error;vel_error;state(7:12)];
LoggedSignals.State = [pos_error;vel_error;acc_error;R(:);state(10:12)];

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
omega_l2 = norm(state(10:12));
% std_action = std(Action);
% action_l2 = norm(Action);

% % Rewards
% tau_pos = 0.35/2.5*vel;
% tau_vel = 1.5/2.5*vel;
% tau_acc = 3/2.5*vel;
% tau_yaw = 5*pi/180/2.5*vel;
% r_pos = exp(-(1/tau_pos * pos_l2)^2);
% r_vel = exp(-(1/tau_vel * vel_l2)^2);
% r_acc = exp(-(1/tau_acc * acc_l2)^2);
% r_yaw = exp(-(1/tau_yaw * yaw_error)^2);
r_pos = exp(-(1/0.35 * pos_l2).^2);
r_vel = exp(-(1/1.5 * vel_l2).^2);
r_acc = exp(-(1/3 * acc_l2).^2);
r_yaw = exp(-(1/(5*pi/180) * yaw_error).^2);
% r_omega_z = exp(-omega_z^2);
r_omega = exp(-(1/pi * omega_l2).^2);
% r_action = exp(-(1/2 * std_action)^2);
% r_action = exp(-(1/8 * action_l2)^2);
% r_action = exp(-0.001*action_l2^2); % For imitation learning

rewards = [0.6 0.1 0.3] .* [r_pos r_vel r_omega];

fprintf('r,e: %f %f %f %f %f| %f %f %f %f %f\n',r_pos,r_vel,r_acc,r_yaw,r_omega,pos_l2,vel_l2,acc_l2,yaw_error*180/pi,omega_l2*180/pi)
fprintf('Actions: %f %f %f %f\n',Action(1),Action(2),Action(3),Action(4))
% fprintf('Geo.Ctrlr: %f %f %f %f\n',action_ref_control(1),action_ref_control(2),action_ref_control(3),action_ref_control(4))
% Termination reward
reward_terminate = 0;

% Check termination condition
IsDone = total_time <= Time;


% relationship between omega & euler angles
% mapping_R = [cos(state(8)) 0 -cos(state(7))*sin(state(8));...
%              0         1            sin(state(7));...
%              sin(state(8)) 0  cos(state(7))*cos(state(8))];
% fprintf('cond(R): %f\n',det(mapping_R))

% if  Step > 200
if pos_l2 > 1
    IsDone = true;
    reward_terminate = -1;
end
% end
if ~IsDone
    Reward = sum(rewards);
else
    Reward = reward_terminate;
end

end
