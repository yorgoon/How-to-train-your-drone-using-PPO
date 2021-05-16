% Model parameters
S.g = 9.807; % Gravity
S.mb = 1.477; % Mass of UAV
S.d = 0.263; % Arm Length (Rotor and COM of UAV)
S.c = 8.004e-4; % Drag Factor
S.Ib = [0.01152; 0.01152; 0.0218]; % Moment of Inertia of UAV
% S.Ib = [0.01152 0 0;0 0.01152 0;0 0 0.0218];
S.m1 = 0.05; % Mass of First Link
S.m2 = 0.05; % Mass of Second Link
S.l1 = 0.5; % Length of First Link
S.l2 = 0.5; % Length of Second Link

% Observation and Action Information
numObs = 21;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';

thrust = (S.mb)*S.g/4;
numAct = 4;
actInfo = rlNumericSpec([numAct 1]);
actInfo.Name = 'Quad Action';

%Define Environment
env = rlFunctionEnv(obsInfo,actInfo,'myStepFunction3','myResetFunction3');
%%
% create the network to be used as approximator in the critic
% it must take the observation signal as input and produce a scalar value
criticNet = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(512,'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(512,'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(512,'Name', 'fc3')
    reluLayer('Name', 'relu3')
    fullyConnectedLayer(512,'Name', 'fc4')
    reluLayer('Name', 'relu4')
    fullyConnectedLayer(1,'Name','out')
    ];

% set some training options for the critic
criticOpts = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1);
if gpuDeviceCount("available")
    criticOpts.UseDevice = 'gpu';
end
% create the critic representation from the network
critic = rlValueRepresentation(criticNet,obsInfo,'Observation',{'state'},criticOpts);

%%
% input path layers (2 by 1 input and a 1 by 1 output)
statePath = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(512, 'Name','commonFC1')
    reluLayer('Name','CommonRelu1')
    fullyConnectedLayer(512, 'Name','commonFC3')
    reluLayer('Name','CommonRelu3')
    fullyConnectedLayer(512, 'Name','commonFC4')
    reluLayer('Name','CommonRelu4')
    fullyConnectedLayer(512, 'Name','commonFC2')
    reluLayer('Name','CommonRelu2')
    ];
meanPath = [
    fullyConnectedLayer(512,'Name','MeanFC1')
    reluLayer('Name','MeanRelu1')
    fullyConnectedLayer(512,'Name','MeanFC2')
    reluLayer('Name','MeanRelu2')
    fullyConnectedLayer(512,'Name','MeanFC3')
    reluLayer('Name','MeanRelu3')
    fullyConnectedLayer(numAct,'Name','Mean')
    tanhLayer('Name','MeanTanh')
    scalingLayer('Name','MeanScaling','Scale',3,'Bias',thrust+0.5)
    ];
stdPath = [
    fullyConnectedLayer(512,'Name','StdFC1')
    reluLayer('Name','StdRelu1')
    fullyConnectedLayer(512,'Name','StdFC2')
    reluLayer('Name','StdRelu2')
    fullyConnectedLayer(512,'Name','StdFC3')
    reluLayer('Name','StdRelu3')
    fullyConnectedLayer(numAct,'Name','StdFC4')
%     softplusLayer('Name', 'vp_out')
    sigmoidLayer('Name','StdSig')
    scalingLayer('Name','ActorScaling','Scale',0.1*thrust)
    ];
concatPath = concatenationLayer(1,2,'Name','GaussianParameters');

actorNetwork = layerGraph(statePath);
actorNetwork = addLayers(actorNetwork,meanPath);
actorNetwork = addLayers(actorNetwork,stdPath);
actorNetwork = addLayers(actorNetwork,concatPath);
actorNetwork = connectLayers(actorNetwork,'CommonRelu2','MeanFC1/in');
actorNetwork = connectLayers(actorNetwork,'CommonRelu2','StdFC1/in');
actorNetwork = connectLayers(actorNetwork,'MeanScaling','GaussianParameters/in1');
actorNetwork = connectLayers(actorNetwork,'ActorScaling','GaussianParameters/in2');

actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-4,...
                                 'GradientThreshold',1,'L2RegularizationFactor',1e-5);
if gpuDeviceCount("available")
    actorOpts.UseDevice = 'gpu';
end

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,actorOptions,...
    'Observation',{'observation'});
%%
agentOpts = rlPPOAgentOptions('SampleTime',0.01);
agent = rlPPOAgent(actor,critic,agentOpts);
%%
agent.AgentOptions.ExperienceHorizon = 2^11;
agent.AgentOptions.MiniBatchSize = 2^11;
agent.AgentOptions.UseDeterministicExploitation = false;
agent.AgentOptions.EntropyLossWeight = 0.4;
%agent.AgentOptions.NumEpoch = 10;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',10000000, ...
    'MaxStepsPerEpisode',10000, ...
    'Verbose',false, ...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',10000000,...
    'ScoreAveragingWindowLength',50);
trainOpts.UseParallel = true;
% trainOpts.ParallelizationOptions.StepsUntilDataIsSent = 2^11;
% agent.AgentOptions.ClipFactor = 0.1;
%%
trainingStats = train(agent,env,trainOpts);
% beta_rnd = betarnd(10,10,[size(Mean)]);
% beta_rnd = (beta_rnd-mean(beta_rnd))/std(beta_rnd);
% Action = Mean + StandardDeviation .* beta_rnd;
%%
agent.AgentOptions.UseDeterministicExploitation = true;
simOptions = rlSimulationOptions('MaxSteps',5000);
experience = sim(env,agent,simOptions);
t = experience.Observation.QuadStates.Time;
xsave = squeeze(experience.Observation.QuadStates.Data(:,1,:));
action = squeeze(experience.Action.QuadAction.Data(:,1,:));
%
global Tau_vec P State;
t_ref = 0.01:0.01:sum(Tau_vec)*0.999;
pos_ref = getPos(Tau_vec, t_ref', P);
figure(1)
subplot(2,1,1)
plot(t, State(1,:),'Color','red')
grid on
hold on
plot(t, State(2,:),'Color','green')
plot(t, State(3,:),'Color','blue')
plot(t_ref,pos_ref(:,1),'--','Color','red')
plot(t_ref,pos_ref(:,2),'--','Color','green')
plot(t_ref,pos_ref(:,3),'--','Color','blue')
legend('x','y','z')
hold off
subplot(2,1,2)
plot(t, State(4,:),'Color','red')
grid on
hold on
plot(t, State(5,:),'Color','green')
plot(t, State(6,:),'Color','blue')
hold off
figure(2)
subplot(2,1,1)
plot(t, State(7,:))
grid on
hold on
plot(t, State(8,:))
plot(t, State(9,:))
legend('r','p','y')
hold off
subplot(2,1,2)
plot(t, State(10,:))
grid on
hold on
plot(t, State(11,:))
plot(t, State(12,:))
hold off
windowSize = 2;
b = (1/windowSize)*ones(1,windowSize);
a=1;
action_filtered = filter(b,a,action');
action_filtered = action_filtered';
figure(3)
plot(t(10:end), action_filtered(1,9:end))
hold on
plot(t(10:end), action_filtered(2,9:end))
plot(t(10:end), action_filtered(3,9:end))
plot(t(10:end), action_filtered(4,9:end))
grid on
hold off
%%
global Fext_hist;
size(Fext_hist)
fig = figure(4);
plot3(State(1,:),State(2,:),State(3,:))
grid on
axis equal
hold on
plot3(pos_ref(:,1),pos_ref(:,2),pos_ref(:,3),'--','Color','red')
hold off
%%
filename = '0513_line_PPO_2.avi';
target_fps = 50;
video_gen(fig, t, State', filename, target_fps, Fext_hist)