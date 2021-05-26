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
statePath1 = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(150,'Name','CriticStateFC1')
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(150,'Name','CriticStateFC3')
    reluLayer('Name','CriticStateRelu3')
    fullyConnectedLayer(150,'Name','CriticStateFC4')
    ];
actionPath1 = [
    featureInputLayer(numAct,'Normalization','none','Name','action')
    fullyConnectedLayer(150,'Name','CriticActionFC1')
    reluLayer('Name','CriticActionRelu1')
    fullyConnectedLayer(150,'Name','CriticActionFC3')
    reluLayer('Name','CriticActionRelu3')
    fullyConnectedLayer(150,'Name','CriticActionFC4')
    ];
commonPath1 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(1,'Name','CriticOutput')
    ];

criticNet = layerGraph(statePath1);
criticNet = addLayers(criticNet,actionPath1);
criticNet = addLayers(criticNet,commonPath1);
criticNet = connectLayers(criticNet,'CriticStateFC4','add/in1');
criticNet = connectLayers(criticNet,'CriticActionFC4','add/in2');

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
critic1 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
critic2 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
%
%% input path layers (2 by 1 input and a 1 by 1 output)
statePath = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(150, 'Name','commonFC1')
    reluLayer('Name','CommonRelu1')
    fullyConnectedLayer(150, 'Name','commonFC3')
    reluLayer('Name','CommonRelu3')
    fullyConnectedLayer(150, 'Name','commonFC4')
    reluLayer('Name','CommonRelu4')
    fullyConnectedLayer(150, 'Name','commonFC2')
    reluLayer('Name','CommonRelu2')
    ];
meanPath = [
    fullyConnectedLayer(150,'Name','MeanFC1')
    reluLayer('Name','MeanRelu1')
    fullyConnectedLayer(150,'Name','MeanFC2')
    reluLayer('Name','MeanRelu2')
    fullyConnectedLayer(150,'Name','MeanFC3')
    reluLayer('Name','MeanRelu3')
    fullyConnectedLayer(numAct,'Name','Mean')
    sigmoidLayer('Name','MeanSig')
    scalingLayer('Name','MeanScaling','Scale',9,'Bias',1)
    ];
stdPath = [
    fullyConnectedLayer(150,'Name','StdFC1')
    reluLayer('Name','StdRelu1')
    fullyConnectedLayer(150,'Name','StdFC2')
    reluLayer('Name','StdRelu2')
    fullyConnectedLayer(150,'Name','StdFC3')
    reluLayer('Name','StdRelu3')
    fullyConnectedLayer(numAct,'Name','StdFC4')
    sigmoidLayer('Name','StdSig')
    scalingLayer('Name','ActorScaling','Scale',0.5,'Bias',0)
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

actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',2e-4,...
                                 'GradientThreshold',1);
if gpuDeviceCount("available")
    actorOpts.UseDevice = 'gpu';
end

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,actorOptions,...
    'Observation',{'observation'});
%%
agentOptions = rlSACAgentOptions;
agentOptions.SampleTime = 0.01;
agentOptions.DiscountFactor = 0.99;
agentOptions.TargetSmoothFactor = 1e-4;
agentOptions.ExperienceBufferLength = 1e6;
agentOptions.MiniBatchSize = 2^11;

agent = rlSACAgent(actor,[critic1 critic2],agentOptions);

%%
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',1000000, ...
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
subplot(2,1,1)
plot(t, xsave(1,:),'.')
grid on
hold on
plot(t, xsave(2,:),'.')
plot(t, xsave(3,:),'.')
subplot(2,1,2)
plot(t, xsave(4,:),'.')
grid on
hold on
plot(t, xsave(5,:),'.')
plot(t, xsave(6,:),'.')
figure(2)
subplot(2,1,1)
plot(t, xsave(7,:),'.')
grid on
hold on
plot(t, xsave(8,:),'.')
plot(t, xsave(9,:),'.')
subplot(2,1,2)
plot(t, xsave(10,:),'.')
grid on
hold on
plot(t, xsave(11,:),'.')
plot(t, xsave(12,:),'.')
figure(3)
plot3(xsave(1,:),xsave(2,:),xsave(3,:))
grid on
axis equal
figure(4)
plot(t(2:end), action(1,:),'.')
hold on
plot(t(2:end), action(2,:),'o')
plot(t(2:end), action(3,:),'x')
plot(t(2:end), action(4,:),'*')
grid on