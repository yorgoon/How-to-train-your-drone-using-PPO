function J = costFunc_appA(tau_vec, PATH, gamma)

A = A_const_appA(tau_vec);
K = length(tau_vec);
C = permut_mat(K);
bF = const_bf(PATH);
Q = Q_const_appA(tau_vec);
R = comp_R_appA(A,C,Q);
b_sorted = comp_b_sorted(R, bF);

% Compute cost
J = trace(b_sorted'*R*b_sorted) + gamma*sum(tau_vec);

end

