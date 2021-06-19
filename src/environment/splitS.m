function [Tau_vec, PATH] = splitS()
% Pre acceleration
pre = 20;
% Theta; Specifies number of roll
roll_num = 1;
% Number of points on a circle
pts_circle = 4;
angle_0 = -pi/2+pi/6;
theta = linspace(angle_0, roll_num*(pi-pi/3)+angle_0, roll_num*pts_circle+1);
% Circle stride
dx = linspace(0,0,length(theta));
% Nominal radius
r = 4;
% X,Y,Z
x = r.*cos(theta) + dx;
y = linspace(pre,1*pre,length(theta));
z = (r.*sin(theta)+r);
% Way points
PATH = [x',y',z'];
PATH = PATH+[pre/2, -y(1), 0];
% PATH = [1,-1,0;PATH];
PATH = [PATH;0,0,2*r];
PATH = [zeros(1,3);PATH];

% Stride along y axis
dy = linspace(0,-5,length(PATH));
PATH(:,2) = PATH(:,2) + dy';
PATH(:,3) = -PATH(:,3);
% Time intervals
Tau_vec = zeros(length(PATH)-1,1);
Tau_vec(2:end-1) = 0.25;
Tau_vec(1) = 2;
Tau_vec(end) = 2;
end

