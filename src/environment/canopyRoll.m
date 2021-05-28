function [Tau_vec, PATH] = canopyRoll()
% Pre acceleration
pre = 10;
% Theta; Specifies number of roll
roll_num = 1;
% Number of points on a circle
pts_circle = 3;
angle_0 = pi-pi/6;
theta = linspace(angle_0, pi/6, roll_num*pts_circle+1);
% Circle stride
dx = linspace(0,0,length(theta));
% Nominal radius
r = 6;
% X,Y,Z
x = linspace(pre,2*pre,length(theta));
y = r.*cos(theta) + r;
z = r.*sin(theta);
% Way points
PATH = [x',y',z'];
PATH = [zeros(1,3);PATH];
PATH = [PATH;x(end)+pre,2*r,0];
% Time intervals
Tau_vec = zeros(length(PATH)-1,1);
Tau_vec(2:end-1) = 0.5;
Tau_vec(1) = 1.75;
Tau_vec(end) = 1.75;
end

