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
S.g = 9.807; % Gravity

% Observation and Action Information
numObs = 12;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';

thrust = (S.mb)*S.g/4;
numAct = 4;
actInfo = rlNumericSpec([numAct 1]);
actInfo.Name = 'Quad Action';

%Define Environment
env = rlFunctionEnv(obsInfo,actInfo,'myStepFunction3','myResetFunction3');
%% Agent
statePath = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','CriticStateFC1')
    tanhLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(300,'Name','CriticStateFC2')
    tanhLayer('Name', 'CriticRelu2')
    fullyConnectedLayer(64,'Name','CriticStateFC4')];
actionPath = [
    featureInputLayer(numAct,'Normalization','none','Name','action')
    fullyConnectedLayer(400,'Name','CriticActionFC1')
    tanhLayer('Name', 'CriticActionRelu1')
    fullyConnectedLayer(300,'Name','CriticActionFC2')
    tanhLayer('Name', 'CriticActionRelu2')
    fullyConnectedLayer(64,'Name','CriticActionFC4','BiasLearnRateFactor',0)];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
    
criticNetwork = connectLayers(criticNetwork,'CriticStateFC4','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC4','add/in2');

criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOpts);

actorNetwork = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','ActorFC1')
    tanhLayer('Name','ActorRelu1')
    fullyConnectedLayer(300,'Name','ActorFC2')
    tanhLayer('Name','ActorRelu2')
    fullyConnectedLayer(numAct,'Name','ActorFC4')
    tanhLayer('Name','ActorTanh')
    scalingLayer('Name','ActorScaling','Scale',1,'Bias',thrust)
    ];

actorOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOpts);
%%
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',0.01,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',1024);
agentOpts.NoiseOptions.Variance = 0.6;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;

agent = rlDDPGAgent(actor,critic,agentOpts);

%%
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',100000, ...
    'MaxStepsPerEpisode',10000, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria',"AverageReward",...
    'UseParallel',true,...
    'StopTrainingValue',10000000);

trainingStats = train(agent,env,trainOpts);
%%
simOptions = rlSimulationOptions('MaxSteps',5000);
experience = sim(env,agent,simOptions);
%
t = experience.Observation.QuadStates.Time;
xsave = squeeze(experience.Observation.QuadStates.Data(:,1,:));
action = squeeze(experience.Action.QuadAction.Data(:,1,:));
%%
figure(1)
plot(t, xsave(1,:),'.')
grid on
hold on
plot(t, xsave(2,:),'.')
plot(t, xsave(3,:),'.')

figure(2)
plot3(xsave(1,:),xsave(2,:),xsave(3,:))
grid on
axis equal
figure(3)
plot(t(2:end), action(1,:),'.')
hold on
plot(t(2:end), action(2,:),'o')
plot(t(2:end), action(3,:),'x')
plot(t(2:end), action(4,:),'*')
grid on
