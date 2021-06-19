function actor = actorNetwork(obsInfo, actInfo, activation)
% input path layers (2 by 1 input and a 1 by 1 output)
numObs = obsInfo.Dimension(1);
numAct = actInfo.Dimension(1);
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
thrust = (S.mb)*S.g/4;
thrust_max = 10;

% inverse of tanh
% a = (thrust - thrust_max/2)/(thrust_max/2);
% offset = 1/2*log((1+a)/(1-a));
% scale = 1/8;

% For sigmoid
scale = 1/2.5;
offset = 1/scale*log((thrust/thrust_max)/(1-thrust/thrust_max));

switch activation
    case 'single'
        statePath = [
            featureInputLayer(numObs,'Normalization','none','Name','observation')
            fullyConnectedLayer(192, 'Name','commonFC1')
            reluLayer('Name','CommonRelu1')
            fullyConnectedLayer(256, 'Name','commonFC3')
            reluLayer('Name','CommonRelu3')
            fullyConnectedLayer(256, 'Name','commonFC4')
            reluLayer('Name','CommonRelu4')
            fullyConnectedLayer(192, 'Name','commonFC2')
            reluLayer('Name','CommonRelu2')
            ];
        meanPath = [
            fullyConnectedLayer(192,'Name','MeanFC1')
            reluLayer('Name','MeanRelu1')
            fullyConnectedLayer(256,'Name','MeanFC2')
            reluLayer('Name','MeanRelu2')
            fullyConnectedLayer(192,'Name','MeanFC3')
            reluLayer('Name','MeanRelu3')
            fullyConnectedLayer(numAct,'Name','Mean')
            scalingLayer('Name','MeanScaling2','Scale',scale,'Bias',scale*offset)
            sigmoidLayer('Name','MeanTanh')
            scalingLayer('Name','MeanScaling','Scale',thrust_max)
            ];
        stdPath = [
            fullyConnectedLayer(192,'Name','StdFC1')
            reluLayer('Name','StdRelu1')
            fullyConnectedLayer(256,'Name','StdFC2')
            reluLayer('Name','StdRelu2')
            fullyConnectedLayer(192,'Name','StdFC3')
            reluLayer('Name','StdRelu3')
            fullyConnectedLayer(numAct,'Name','StdFC4')
            sigmoidLayer('Name','StdSigmoid')
            scalingLayer('Name','StdScaling','Scale',thrust*0.1)
            ];
        concatPath = concatenationLayer(1,2,'Name','GaussianParameters');

        actorNetwork = layerGraph(statePath);
        actorNetwork = addLayers(actorNetwork,meanPath);
        actorNetwork = addLayers(actorNetwork,stdPath);
        actorNetwork = addLayers(actorNetwork,concatPath);
        actorNetwork = connectLayers(actorNetwork,'CommonRelu2','MeanFC1/in');
        actorNetwork = connectLayers(actorNetwork,'CommonRelu2','StdFC1/in');
        actorNetwork = connectLayers(actorNetwork,'MeanScaling','GaussianParameters/in1');
        actorNetwork = connectLayers(actorNetwork,'StdScaling','GaussianParameters/in2');

    case 'dual'
        statePath = [
            featureInputLayer(numObs,'Normalization','none','Name','observation')
            fullyConnectedLayer(192, 'Name','commonFC1')
            reluLayer('Name','CommonRelu1')
            fullyConnectedLayer(256, 'Name','commonFC3')
            reluLayer('Name','CommonRelu3')
            fullyConnectedLayer(256, 'Name','commonFC4')
            reluLayer('Name','CommonRelu4')
            fullyConnectedLayer(192, 'Name','commonFC2')
            reluLayer('Name','CommonRelu2')
            ];
        meanCommonPath = [
            fullyConnectedLayer(192,'Name','MeanFC1')
            reluLayer('Name','MeanRelu1')
            fullyConnectedLayer(256,'Name','MeanFC2')
            reluLayer('Name','MeanRelu2')
            fullyConnectedLayer(192,'Name','MeanFC3')
            reluLayer('Name','MeanRelu3')
            fullyConnectedLayer(numAct,'Name','Mean')
            ];
        meanTanh1Path = [
            scalingLayer('Name','meanScaling1','Bias',1.5)
            tanhLayer('Name','meantanh1')
            scalingLayer('Name','meantanh1scaling','Scale',(thrust/2),'Bias',(thrust/2))
            ];
        meanTanh2Path = [
            scalingLayer('Name','meanScaling2','Bias',-1.5)
            tanhLayer('Name','meantanh2')
            scalingLayer('Name','meantanh2scaling','Scale',(thrust_max-thrust)/2,'Bias',(thrust_max-thrust)/2)
            ];
        add = additionLayer(2,'Name','add_1');
        stdPath = [
            fullyConnectedLayer(192,'Name','StdFC1')
            reluLayer('Name','StdRelu1')
            fullyConnectedLayer(256,'Name','StdFC2')
            reluLayer('Name','StdRelu2')
            fullyConnectedLayer(192,'Name','StdFC3')
            reluLayer('Name','StdRelu3')
            fullyConnectedLayer(numAct,'Name','StdFC4')
            sigmoidLayer('Name','StdSigmoid')
            scalingLayer('Name','StdScaling','Scale',1)
            ];

        concatPath = concatenationLayer(1,2,'Name','GaussianParameters');

        actorNetwork = layerGraph(statePath);
        actorNetwork = addLayers(actorNetwork,meanCommonPath);
        actorNetwork = addLayers(actorNetwork,meanTanh1Path);
        actorNetwork = addLayers(actorNetwork,meanTanh2Path);
        actorNetwork = addLayers(actorNetwork,add);
        actorNetwork = addLayers(actorNetwork,stdPath);
        actorNetwork = addLayers(actorNetwork,concatPath);
        actorNetwork = connectLayers(actorNetwork,'CommonRelu2','MeanFC1/in');
        actorNetwork = connectLayers(actorNetwork, 'Mean', 'meanScaling1/in');
        actorNetwork = connectLayers(actorNetwork, 'Mean', 'meanScaling2/in');
        actorNetwork = connectLayers(actorNetwork,'meantanh1scaling','add_1/in1');
        actorNetwork = connectLayers(actorNetwork,'meantanh2scaling','add_1/in2');
        actorNetwork = connectLayers(actorNetwork,'CommonRelu2','StdFC1/in');
        actorNetwork = connectLayers(actorNetwork,'add_1','GaussianParameters/in1');
        actorNetwork = connectLayers(actorNetwork,'StdScaling','GaussianParameters/in2');
end
actorOpts = rlRepresentationOptions('Optimizer','adam','LearnRate',2e-4,...
                                 'GradientThreshold',1);
if gpuDeviceCount("available")
    actorOpts.UseDevice = 'gpu';
end

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,actorOpts,...
    'Observation',{'observation'});
end