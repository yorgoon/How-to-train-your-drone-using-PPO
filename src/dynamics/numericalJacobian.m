function GxFu = numericalJacobian(x,t,dt,S)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
u = controller(x,t,S);
Fu = zeros(length(x),length(u));
x_update = discreteDynamics(x,u,dt,S);
eps = 1e-8;
for i=1:length(u)
    u_eps1 = u;
    u_eps2 = u;
    u_eps1(i) = u_eps1(i) + eps/2;
    u_eps2(i) = u_eps2(i) - eps/2;
    Fu(:,i) = (discreteDynamics(x,u_eps1,dt,S) - discreteDynamics(x,u_eps2,dt,S))/(eps);
end

Gx = zeros(length(u),length(x));
% u_update = controller(x_update,t+dt,S);
for i=1:length(x)
    x_eps1 = x_update;
    x_eps2 = x_update;
    x_eps1(i) = x_eps1(i) + eps/2;
    x_eps2(i) = x_eps2(i) - eps/2;
    Gx(:,i) = (controller(x_eps1,t+dt,S) - controller(x_eps2,t+dt,S))/(eps);
end
GxFu = Gx*Fu;
end

