function x = discreteDynamics(xk, uk, S)

dt = S.dt;

B = inertia2(xk(1:8),S);

C = coriolis3(xk(1:8),xk(9:16),S);

g_psi = potential2(xk(1:8),S);

f = [xk(9:16); B\(uk - C*xk(9:16) -g_psi)];

x = xk + dt * f;

end