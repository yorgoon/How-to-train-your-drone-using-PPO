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
global Step State Time Tau_vec PATH P Action_hist Fext Fext_hist;
% global Mext external_callback reward_accum;

% Sample time
ts = 0.01;

Step = Step + 1;

% Total execution time
total_time = sum(Tau_vec)+1;

% Update time
Time = Time + ts;

% External force applied (Comment out)
% if Time < 8 && Time > 7 && external_callback
%     Fext = 2 * [1,1,-1]'/norm([1,1,-1]); % 0~1N (about apple weight)
%     Mext = 0 * rand * random_unit_vector; % 0~1Nm
%     external_callback = false;
% end
% 
% if Time > 8 && ~external_callback
%     Fext = zeros(3,1);
%     external_callback = true;
% end
% if Time > 9 && external_callback
%     Fext = 2 * [-1,-1,-1]'/norm([-1,-1,-1]); % 0~1N (about apple weight)
%     Mext = 0 * rand * random_unit_vector; % 0~1Nm
%     external_callback = false;
% end

% if Time > 3
%     Fext = 2*[0,0,-1]';
% end

% Discrete Action sets
% Action = round(Action,1);

% Current state
state = State(:,end);

% Desired state
desired_state = desired_state_optimal(Tau_vec, Time, PATH, P);

% Reference controller (Comment out)
action_ref_control = reference_controller(state, desired_state, S);
% state_dot = droneDynamics(state, action_ref_control, S);

% Agent controller
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
omega_z = abs(state(12));
% omega_z_dot = abs(state_dot(end));
% action_l2 = norm(Action - action_ref_control);

% Rewards
r_pos = exp(-500*pos_l2^2);
r_vel = exp(-100*vel_l2^2);
r_acc = exp(-20*acc_l2^2);
r_yaw = exp(-1000*yaw_error^2);
r_omega = exp(-1000*omega_z^2);
% r_omega_dot = exp(-100*omega_z_dot^2);
% r_action = exp(-0.001*action_l2^2);

rt = [0.35 0.1 0.1] * [r_pos r_vel r_acc]';
rr = [0.35 0.1] * [r_yaw r_omega]';
% ra = 0.1 * r_action;

fprintf('all rewards: %f %f %f %f %f\n',r_pos,r_vel,r_acc,r_yaw,r_omega)

% Check termination condition
IsDone = total_time <= Time;

if  pos_l2 >= 0.1
    IsDone = true;
%     fprintf('pos, yaw: %f %f\n',pos_l2, yaw_error/pi*180)
end
if ~IsDone
    Reward = rt + rr;
else
    reward_terminate = -1;
    Reward = reward_terminate;
%     fprintf('Generated steps: %d \n',Step)
    if total_time < Time
%         fprintf('Goal reached with %d steps with error %f %f.\n',Step,pos_l2,yaw_error/pi*180)
    end
end

end
