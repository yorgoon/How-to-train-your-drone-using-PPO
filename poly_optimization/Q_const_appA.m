function QQ = Q_const_appA(tau_vec)

m = length(tau_vec);
QQ = zeros(10*m);
for i = 1:m
    QQ(10*(i-1)+1:10*i,10*(i-1)+1:10*i) = cost_mat_appA(tau_vec(i));
end

