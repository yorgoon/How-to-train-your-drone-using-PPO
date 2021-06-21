# How to train your drone using PPO
 RL based robust controller for quadrotor. Proximal Policy Optimization (PPO) is used. This project is a part of the course XAI 601 Applications in Deep Learning in Korea University.
 
 This repository contains MATLAB-based simulation for quadrotor using PPO agent.
 
 **Main Contributors**: Soowon Kim (soowon_kim@korea.ac.kr)

**Affiliation**: Korea University

## How To
To train the agent, please run the main script sequentially in ``PPO.m``. In the declaration of ``actor``, you can choose either ``single`` or ``dual`` tanh activation for the mean output of the actions.

To test the agent, run ``Test`` section under ``PPO.m``. You can create your own trajectory by modifying ``myReset.m`` function by declaring your own ``path`` and ``tau_vec``. 

``path`` consists of ``x,y,z`` coordinates of way points. It always starts and ends with zero velocity, acceleration, jerk, and snap. Then you have to declare time interval between way points. For example, you can set ``path=[0,0,0;1,1,1];``. Since it only has start and end points, time interval can be set ``tau_vec=10;``.

![alt text](https://github.com/yorgoon/How-to-train-your-drone-using-PPO/blob/main/src/figures/traj_example.pdf?raw=true)

For more information on how to create the trajectory, please visit my other repository, https://github.com/yorgoon/minimum-snap-geometric-control/.

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
