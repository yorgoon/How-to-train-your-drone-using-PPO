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
actInfo.LowerLimit = [0 0 0 0]';
%Define Environment
env = rlFunctionEnv(obsInfo,actInfo,'myStepFunction3','myResetFunction3');
%%
actorNetwork = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(128, 'Name','ActorStateFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(256, 'Name','ActorStateFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(256, 'Name','ActorStateFC3')
    reluLayer('Name','ActorRelu3')
    fullyConnectedLayer(128, 'Name','ActorStateFC4')
    reluLayer('Name','ActorRelu4')
    fullyConnectedLayer(2*numAct,'Name','action')
    tanhLayer('Name','MeanTanh')
    scalingLayer('Name','MeanScaling','Scale',4.5,'Bias',5.5)
    ];
    
actorOpts = rlRepresentationOptions('LearnRate',1e-6,...
    'GradientThreshold',1,...
    'L2RegularizationFactor',0.0001);

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'state'},actorOpts);
%%
% create the network to be used as approximator in the critic
% it must take the observation signal as input and produce a scalar value
criticNet = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(192,'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(256,'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(256,'Name', 'fc3')
    reluLayer('Name', 'relu3')
    fullyConnectedLayer(192,'Name', 'fc4')
    reluLayer('Name', 'relu4')
    fullyConnectedLayer(1,'Name','out')
    ];

% set some training options for the critic
criticOpts = rlRepresentationOptions('LearnRate',2e-4,'GradientThreshold',1);
if gpuDeviceCount("available")
    criticOpts.UseDevice = 'gpu';
end
% create the critic representation from the network
critic = rlValueRepresentation(criticNet,obsInfo,'Observation',{'state'},criticOpts);
%%
criticNetwork = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(64,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(64,'Name','CriticStateFC2')
    reluLayer('Name','CriticRelu2')
    fullyConnectedLayer(64,'Name','CriticStateFC3')
    reluLayer('Name','CriticRelu3')
    fullyConnectedLayer(64,'Name','CriticStateFC4')
    reluLayer('Name','CriticRelu4')
    fullyConnectedLayer(1, 'Name', 'CriticFC')];

criticOpts = rlRepresentationOptions('LearnRate',1e-6,'GradientThreshold',1,...
    'L2RegularizationFactor',0.0001);

critic = rlValueRepresentation(criticNetwork,obsInfo,'Observation',{'state'},criticOpts);


%%
agentOpts = rlACAgentOptions(...
    'NumStepsToLookAhead',32,...
    'EntropyLossWeight',0.01,...
    'DiscountFactor',0.99);
agent = rlACAgent(actor,critic,agentOpts);
%%
agent.AgentOptions.SampleTime = 0.01;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',10000000, ...
    'MaxStepsPerEpisode',10000, ...
    'Verbose',false, ...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',10000000,...
    'ScoreAveragingWindowLength',100);
trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.DataToSendFromWorkers = 'Gradients';
trainOpts.ParallelizationOptions.StepsUntilDataIsSent = 1000;
% agent.AgentOptions.ClipFactor = 0.1;
%%
trainingStats = train(agent,env,trainOpts);
%%
simOptions = rlSimulationOptions('MaxSteps',5000);
experience = sim(env,agent,simOptions);
%
t = experience.Observation.QuadStates.Time;
xsave = squeeze(experience.Observation.QuadStates.Data(:,1,:));
action = squeeze(experience.Action.QuadAction.Data(:,1,:));
size(xsave)
figure(1)
plot(t, xsave(1,:),'.')
grid on
hold on
plot(t, xsave(2,:),'.')
plot(t, xsave(3,:),'.')
figure(2)
plot(t(2:end), action(1,:),'.')
hold on
plot(t(2:end), action(2,:),'o')
plot(t(2:end), action(3,:),'x')
plot(t(2:end), action(4,:),'.')
grid on
