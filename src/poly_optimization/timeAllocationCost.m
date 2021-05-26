function J = timeAllocationCost(traj, gamma)
% Compute cost
J = traj.J + gamma*sum(tau_vec);
% J = MinimumSnapTrajectory(tau_vec', PATH).J + gamma*sum(tau_vec);
end