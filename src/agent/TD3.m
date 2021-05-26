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
% Control Parameters

numObs = 12;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';
obsInfo.Description = 'x,y,z,xdot,ydot,zdot,r,p,y,wx,wy,wz';

thrust = (S.mb)*S.g/4;
numAct = 4;
actInfo = rlNumericSpec([numAct 1]);
actInfo.Name = 'Quad Action';

%Define Environment
env = rlFunctionEnv(obsInfo,actInfo,'myStepFunction2','myResetFunction2');
%%
statePath1 = [
    sequenceInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','CriticStateFC1')
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(300,'Name','CriticStateFC2')
    ];
actionPath1 = [
    sequenceInputLayer(numAct,'Normalization','none','Name','action')
    fullyConnectedLayer(300,'Name','CriticActionFC1')
    ];
commonPath1 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    lstmLayer(16,'OutputMode','sequence','Name','CriticLSTM');
    fullyConnectedLayer(1,'Name','CriticOutput')
    ];

criticNet = layerGraph(statePath1);
criticNet = addLayers(criticNet,actionPath1);
criticNet = addLayers(criticNet,commonPath1);
criticNet = connectLayers(criticNet,'CriticStateFC2','add/in1');
criticNet = connectLayers(criticNet,'CriticActionFC1','add/in2');

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
critic1 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
critic2 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
%%
actorNet = [
    sequenceInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','ActorFC1')
    lstmLayer(8,'OutputMode','sequence','Name','ActorLSTM')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(300,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(numAct,'Name','ActorFC3')                       
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',0.01*thrust,'Bias',thrust)
    ];

actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);
actor  = rlDeterministicActorRepresentation(actorNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);

%%
agentOptions = rlTD3AgentOptions;
agentOptions.DiscountFactor = 0.99;
agentOptions.SequenceLength = 32;
agentOptions.TargetSmoothFactor = 5e-3;
agentOptions.TargetPolicySmoothModel.Variance = 0.2;
agentOptions.TargetPolicySmoothModel.LowerLimit = -0.5;
agentOptions.TargetPolicySmoothModel.UpperLimit = 0.5;

agent = rlTD3Agent(actor,[critic1 critic2],agentOptions);
%%
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',100000, ...
    'MaxStepsPerEpisode',10000, ...
    'Verbose',false, ...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',10000);

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