function H = measurementJacobian(x,u,dt,S)
eps = 1e-3;
H = zeros(8,length(u));
for i=1:length(u)
    u_eps1 = u;
    u_eps2 = u;
    u_eps1(i) = u_eps1(i) + eps/2;
    u_eps2(i) = u_eps2(i) - eps/2;
%     discreteDynamics(x,u_eps1,dt,S)
%     discreteDynamics(x,u_eps2,dt,S)
    H_tmp = (discreteDynamics(x,u_eps1,dt,S) - discreteDynamics(x,u_eps2,dt,S))/(eps);
    H(:,i) = H_tmp(1:8);
end
end