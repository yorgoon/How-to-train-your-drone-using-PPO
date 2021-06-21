% Observation and Action Information
% Set global variables
numObs = 16;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';
numAct = 4;
actInfo = rlNumericSpec([numAct 1]);
actInfo.Name = 'Quad Action';
actInfo.LowerLimit = [0,0,0,0]';

% Define environment
env = rlFunctionEnv(obsInfo,actInfo,'myStep','myReset');
% Actor network
actor = actorNetwork(obsInfo, actInfo,'single');
% Critic network
critic = criticNetwork(obsInfo);
%% Options
agentOpts = rlPPOAgentOptions('SampleTime',0.01,...
    'ExperienceHorizon',2^11,...
    'MiniBatchSize',2^11,...
    'UseDeterministicExploitation',false,...
    'EntropyLossWeight',0.4);

agent = rlPPOAgent(actor,critic,agentOpts);
%%
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',1000000, ...
    'MaxStepsPerEpisode',1000000, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'ScoreAveragingWindowLength',100,...
    'StopTrainingCriteria',"AverageReward",...
    'UseParallel',true,...
    'StopTrainingValue',10000000);
trainingStats = train(agent,env,trainOpts);
%% Test
agent.AgentOptions.UseDeterministicExploitation = true;
simOptions = rlSimulationOptions('MaxSteps',10000);
experience = sim(env,agent,simOptions);
t = experience.Observation.QuadStates.Time;
t = t(1:end-1);
xsave = squeeze(experience.Observation.QuadStates.Data(:,1,:));
xsave = xsave(:,1:end-1);
action = squeeze(experience.Action.QuadAction.Data(:,1,:));
%%
global Traj State Fext_hist Mext_hist Action_hist;
vel = State(4:6,:);
acc = diff(vel')'/0.01;
windowSize = 1;
a=1;b=(1/windowSize)*ones(1,windowSize);
acc = filter(b,a,acc');acc=acc';

State = State(:,1:end-1);
tau_vec = Traj.tau_vec;
P = Traj.P;
t_ref = linspace(0,sum(tau_vec),100);
pos_ref = zeros(3,length(t_ref));
for i=1:length(t_ref)
    desired_state = desiredState(Traj, t_ref(i));
    pos_ref(:,i) = desired_state.pos;
end
struct = [];
struct.state = State;
struct.action = action_filtered;
struct.t = t';
struct.Fext = Fext_hist;
struct.Mext = Mext_hist;
struct.g_force = g_force;
struct.xsave = xsave;
action = Action_hist;
%% PLOT
clf
figure(1)
% Position
subplot(3,1,1)
plot(t_ref,pos_ref(1,:),'--','Color','red');grid on;hold on;
plot(t_ref,pos_ref(2,:),'--','Color','green')
plot(t_ref,pos_ref(3,:),'--','Color','blue')
plot(t, State(1,:),'Color','red')
plot(t, State(2,:),'Color','green')
plot(t, State(3,:),'Color','blue')
legend('x','y','z')
% Velocity
subplot(3,1,2)
plot(t, State(4,:),'Color','red');grid on;hold on;
plot(t, State(5,:),'Color','green')
plot(t, State(6,:),'Color','blue')
acc(3,:) = acc(3,:)-9.807;
g_force = vecnorm(acc)/9.807;

subplot(3,1,3)
plot(t, g_force);grid on;
%%
figure(2)
% Roll Pitch Yaw
subplot(2,1,1)
plot(t, State(7,:)/180*pi);grid on;hold on
plot(t, State(8,:)/180*pi)
plot(t, State(9,:)/180*pi)
legend('r','p','y')
% Angular velocity
subplot(2,1,2)
plot(t, State(10,:)/180*pi);grid on;hold on
plot(t, State(11,:)/180*pi)
plot(t, State(12,:)/180*pi)

windowSize = 1;
a=1;b=(1/windowSize)*ones(1,windowSize);
action_filtered = filter(b,a,action');action_filtered = action_filtered';
figure(3)
plot(t, action(1,:));grid on;hold on
plot(t, action(2,:))
plot(t, action(3,:))
plot(t, action(4,:))

figure(4)
da = abs(diff(action')'/0.01);
plot(t(2:end), da(1,:));grid on;hold on
plot(t(2:end), da(2,:))
plot(t(2:end), da(3,:))
plot(t(2:end), da(4,:))

%%
fig = figure(5);
clf
set(gcf, 'Position',  [0, 1000, 1440, 500])
% subplot(1,2,1)
subplot('Position',[0. 0 0.65 1])
plot3(pos_ref(1,:),pos_ref(2,:),pos_ref(3,:),'--','Color','red');grid on;axis equal;hold on
plot3(State(1,:),State(2,:),State(3,:),'Color','black')
% xlim([-10.5,10.5]);ylim([-6,6]);zlim([-0.5,0.5])
subplot('Position',[0.65 0.6 .3 .3])
% plot(t, g_force,'b');grid on;
% ylabel('G-Force, g')
plot(t, vecnorm(xsave(1:3,:)),'b');grid on;
ylabel('||e_{pos}||')

subplot('Position',[0.65 0.15 .3 .3])
plot(t,action_filtered(1,:));hold on;grid on;
plot(t,action_filtered(2,:))
plot(t,action_filtered(3,:))
plot(t,action_filtered(4,:))
xlabel('time, sec');ylabel('thrust, N')
a = annotation('textbox', [0.1, 0.9, 0, 0], 'string', ['elapsed time: ', num2str(t(1))],'FitBoxToText','on');
a.FontSize = 20;
%%
filename = '../videos/myVideo';
target_fps = 50;
video_gen(fig, struct, filename, target_fps)