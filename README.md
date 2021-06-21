# How to train your drone using PPO
 RL based robust controller for quadrotor. Proximal Policy Optimization (PPO) is used. This project is a part of the course XAI 601 Applications in Deep Learning in Korea University.
 
 This repository contains MATLAB-based simulation for quadrotor using PPO agent.
 
 **Main Contributors**: Soowon Kim (soowon_kim@korea.ac.kr)

**Affiliation**: Korea University

## How To

#### State, action and reward
Observation (state) consists of the errors between the reference trajectory and current position, velocity, and acclereation. Addition to that, the state has orientation (quaternion) and angular velocity of the agent. Then the state can be expressed as follows:

<img src="https://render.githubusercontent.com/render/math?math=s=[ e_{pos}, e_{vel}, e_{acc},q,\omega]\in \mathbb{R}^{16}">

````
numObs = 16;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';
````

Action consists of four thrusts by each rotor in Newton.
````
numAct = 4;
actInfo = rlNumericSpec([numAct 1]);
actInfo.Name = 'Quad Action';
actInfo.LowerLimit = [0,0,0,0]';
````
In the declaration of ``actor``, you can choose either ``single`` or ``dual`` tanh activation for the mean output of the actions.

````
% Define environment
env = rlFunctionEnv(obsInfo,actInfo,'myStep','myReset');
% Actor network
actor = actorNetwork(obsInfo, actInfo,'single');
% Critic network
critic = criticNetwork(obsInfo);
````
Now you can create PPO agent. You can experiment with different parameters but these setup worked for me.
````
agentOpts = rlPPOAgentOptions('SampleTime',0.01,...
    'ExperienceHorizon',2^11,...
    'MiniBatchSize',2^11,...
    'UseDeterministicExploitation',false,...
    'EntropyLossWeight',0.4);

agent = rlPPOAgent(actor,critic,agentOpts);
````

Reward functions are inspired by [Peng's work](https://xbpeng.github.io/). You can modify them in ``myStep.m``.
````
% Rewards
r_pos = exp(-(1/0.5*pos_l2).^2);
r_vel = exp(-(1/1*vel_l2).^2);
r_acc = exp(-(1/1*acc_l2).^2);
r_yaw = exp(-(1/(5/180*pi)*yaw_error).^2);
````

To test the agent, run ``Test`` section under ``PPO.m``. You can create your own trajectory by modifying ``myReset.m`` function by declaring your own ``path`` and ``tau_vec``. 
### Trajectory
``path`` consists of ``x,y,z`` coordinates of way points. It always starts and ends with zero velocity, acceleration, jerk, and snap. Then you have to declare time interval between way points. For example, you can set ``path=[0,0,0;1,1,1];``. Since it only has start and end points, time interval can be set ``tau_vec=10;``. It would look something like this.

![alt text](https://github.com/yorgoon/How-to-train-your-drone-using-PPO/blob/main/src/figures/traj_example.jpg?raw=true)

You can also use time allocation technique to automatically set time intervals ``tau_vec`` using ``timeAllocation`` with γ parameter. For example,
````
path = [0,0,0;1,1,0;2,0,0;1,-1,0;0,0,0;-1,1,0;-2,0,0;-1,-1,0;0,0,0];
tau_vec = timeAllocation(path, 100)';
````
![alt text](https://github.com/yorgoon/How-to-train-your-drone-using-PPO/blob/main/src/figures/traj_example2.jpg?raw=true)

The greater the γ is, the higher the time penalty, resulting a faster trajectory.

To try a trained model, you can load ``0615_FM.mat`` in results folder.

## Some results
PPO agent trained with random disturbances

[![PPO agent trained with random disturbances](https://img.youtube.com/vi/j0C2QGruKn4/0.jpg)](https://www.youtube.com/watch?v=j0C2QGruKn4)

PPO agent trained to perform Inside loop

[![PPO agent trained to perform Inside loop](https://img.youtube.com/vi/7qMmjWN5tKg/0.jpg)](https://www.youtube.com/watch?v=7qMmjWN5tKg)

PPO agent trained to perform Split S

[![PPO agent trained to perform Split S](https://img.youtube.com/vi/Bap1MF9zrjI/0.jpg)](https://www.youtube.com/watch?v=Bap1MF9zrjI)

PPO agent trained to perform Canopy roll

[![PPO agent trained to perform Canopy roll](https://img.youtube.com/vi/4YRYZYiwABw/0.jpg)](https://www.youtube.com/watch?v=4YRYZYiwABw)
