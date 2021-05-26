function reward = betaReward(error, decay_rate)
    % Shape coefficient
    a = 4; b = 4;
    % Standardized error
    x = error/decay_rate/4 + 0.5;
    % Reward
    reward = betapdf(x, a, b)/betapdf(0.5, a, b);
end