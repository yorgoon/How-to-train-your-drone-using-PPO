function J_grad = timeAllocationCostGradient(tau_vec, PATH, gamma, epsilon_t)

epsilon_t = [epsilon_t, -epsilon_t];
J_pur = zeros(length(tau_vec),2);
for n = 1:2
    for i = 1:length(tau_vec)
        tau_vec_perturb = tau_vec;
        tau_vec_perturb(i) = tau_vec_perturb(i) + epsilon_t(n);
        traj_perturb = MinimumSnapTrajectory(tau_vec_perturb', PATH);
        J_pur(i,n) = timeAllocationCost(traj_perturb, gamma);
    end
end

% Use midpoint rule to compute gradient of the cost function
J_grad = (J_pur(:,1)-J_pur(:,2))./(2*epsilon_t(1));
J_grad = J_grad';

end