function [Tau_vec, PATH] = randomTrajectorySelector()
% Number of trajectories
num = 4;
traj_num = randi(num);
switch traj_num
    case 1
    % Loop the loop
    [Tau_vec, PATH] = loopTheLoop();
    case 2
    % Immelmann turn
    [Tau_vec, PATH] = immelmannTurn();
    case 3
    % Split S
    [Tau_vec, PATH] = splitS();
    case 4
    % Canopy roll
    [Tau_vec, PATH] = canopyRoll();
end

