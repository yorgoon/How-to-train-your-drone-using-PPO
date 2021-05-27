function critic = criticNetwork(obsInfo)
% create the network to be used as approximator in the critic
% it must take the observation signal as input and produce a scalar value
numObs = obsInfo.Dimension(1);
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
end