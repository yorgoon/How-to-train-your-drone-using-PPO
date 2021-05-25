function J = costFunc_appA(tau_vec, PATH, gamma)

% Compute cost
J = MinimumSnapTrajectory(tau_vec', PATH).J + gamma*sum(tau_vec);

end

