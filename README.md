# How to train your drone using PPO
 RL based robust controller for quadrotor. Proximal Policy Optimization (PPO) is used. This project is a part of the course XAI 601 Applications in Deep Learning in Korea University.
 
 This repository contains MATLAB-based simulation for quadrotor using PPO agent.
 
 **Main Contributors**: Soowon Kim (soowon_kim@korea.ac.kr)
 **Affiliation**: Korea University

## How To
To train the agent, please run the main script sequentially in ``PPO.m``. In the declaration of ``actor``, you can choose either ``single`` or ``dual`` tanh activation for the mean output of the actions.

To test the agent, run ``test`` section under ``PPO.m``. You can create your own trajectory by modifying ``myReset.m`` function by declaring your own ``path`` and ``tau_vec``. For more information on how to create the trajectory, please visit my other repository, https://github.com/yorgoon/minimum-snap-geometric-control/.

To try a trained model, you can load ``0615_FM.mat`` in results folder.

## Some results
![IMAGE ALT TEXT HERE](https://img.youtu.be/vi/j0C2QGruKn4/0.jpg)](https://youtu.be/j0C2QGruKn4)
<!-- [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/m1lIAcgYFYQ/0.jpg)](https://www.youtube.com/watch?v=m1lIAcgYFYQ) -->
