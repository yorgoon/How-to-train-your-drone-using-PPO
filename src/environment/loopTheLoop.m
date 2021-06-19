function [Tau_vec, PATH] = loopTheLoop()
% Pre acceleration
pre = 20;
% Theta; Specifies number of roll
roll_num = 1;
% Number of points on a circle
pts_circle = 12;
angle_0 = -pi/2;
theta = linspace(angle_0, roll_num*2*pi + angle_0, roll_num*pts_circle+1);
% Circle stride
dx = linspace(0,0,length(theta));
% Nominal radius
r = 4;
% X,Y,Z
x = r.*cos(theta) + dx;
y = linspace(pre,1*pre,length(theta));
z = (r.*sin(theta)+r);
% Way points
PATH = [x',y',z';pre/2+dx(end),y(end),0];
PATH = PATH+[pre/2, -y(1), 0];
PATH = [zeros(1,3);PATH];
% Stride along y axis
dy = linspace(0,0,length(PATH));
PATH(:,2,:) = PATH(:,2,:) + dy';
% Time intervals
Tau_vec = zeros(length(PATH)-1,1);
Tau_vec(2:end-1) = 0.25;
Tau_vec(1) = 2;
Tau_vec(end) = 2;
end

