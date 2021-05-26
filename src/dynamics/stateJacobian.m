function F = stateJacobian(x, u, S)
dt = S.dt;
eps = 1e-8;
F = zeros(16);
for i=1:length(x)
    x_eps1 = x;
    x_eps2 = x;
    x_eps1(i) = x_eps1(i) + eps/2;
    x_eps2(i) = x_eps2(i) - eps/2;
    F(:,i) = (discreteDynamics(x_eps1, u, S) - discreteDynamics(x_eps2, u, S))/(eps);
end
end