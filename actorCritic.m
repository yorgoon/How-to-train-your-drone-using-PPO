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
S.g = 9.81; % Gravity
% Control Parameters
S.dt = 1/100;


numObs = 12;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';
obsInfo.Description = 'x,y,z,xdot,ydot,zdot,r,p,y,wx,wy,wz';

thrust = (S.mb)*S.g/4;
numAct = 4;
actInfo = rlNumericSpec([numAct 1],'LowerLimit',(thrust*0.99)+[0;0;0;0],...
                                   'UpperLimit',(thrust*1.01)+[0;0;0;0]);
actInfo.Name = 'Quad Action';

%Define Environment
env = rlFunctionEnv(obsInfo,actInfo,'myStepFunction2','myResetFunction2');
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

actorNetwork = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(64, 'Name','ActorStateFC1')
    batchNormalizationLayer('Name','bn1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(64, 'Name','ActorStateFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(64, 'Name','ActorStateFC3')
    reluLayer('Name','ActorRelu3')
    fullyConnectedLayer(64, 'Name','ActorStateFC4')
    reluLayer('Name','ActorRelu4')
    fullyConnectedLayer(2*numAct,'Name','action')
    softplusLayer('Name', 'vp_out');
    scalingLayer('Name','ActorScaling','Scale',max(actInfo.UpperLimit))];
    
actorOpts = rlRepresentationOptions('LearnRate',1e-6,...
    'GradientThreshold',1,...
    'L2RegularizationFactor',0.0001);

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'state'},actorOpts);

agentOpts = rlACAgentOptions(...
    'NumStepsToLookAhead',32,...
    'EntropyLossWeight',0.01,...
    'DiscountFactor',0.99);

% Define Environment
env = rlFunctionEnv(obsInfo,actInfo,'myStepFunction','myResetFunction');

agent = rlACAgent(actor,critic,agentOpts);
%%
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',1000000, ...
    'MaxStepsPerEpisode',10000, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',10000000);

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
