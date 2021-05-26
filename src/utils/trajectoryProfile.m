% Barrel roll, spiral roll

% Pre acceleration
pre = 10;
% Theta; Specifies number of roll
roll_num = 2;
% Number of points on a circle
pts_circle = 5;
angle_0 = -pi/2;
theta = linspace(angle_0, roll_num*2*pi + angle_0, roll_num*pts_circle+1);
% Circle stride
dx = linspace(0,10,length(theta));
% Nominal radius
r = 2;
% X,Y,Z
x = r.*cos(theta) + dx;
y = linspace(pre,1*pre,length(theta));
z = (r.*sin(theta)+r)*2;
% Way points
PATH = [x',y',z';pre/2+dx(end),y(end),0];
PATH = PATH+[pre/2, -y(1), 0];
PATH = [zeros(1,3);PATH];
% Stride along y axis
dy = linspace(0,1,length(PATH));
PATH(:,2,:) = PATH(:,2,:) + dy';
% Time intervals
Tau_vec = zeros(length(PATH)-1,1);
Tau_vec(2:end-1) = 0.75;
Tau_vec(1) = 1.75;
Tau_vec(end) = 1.75;

% Trajectory
traj = MinimumSnapTrajectory(Tau_vec, PATH);
P = traj.P;
t = sum(Tau_vec);
dt = 0.1;
ts = 0:dt:t;
ts_pos = 0:0.001:t;
pos_plot = [];
pos = [];
vel = [];
acc = [];
for i=1:length(ts_pos)
    desired_state = desired_state_optimal(Tau_vec, ts_pos(i), PATH, P);
    pos_plot = [pos_plot, desired_state.pos];
end
for i=1:length(ts)
    desired_state = desired_state_optimal(Tau_vec, ts(i), PATH, P);
    pos = [pos, desired_state.pos];
    vel = [vel, desired_state.vel];
    acc = [acc, desired_state.acc];
end
pos_plot = pos_plot';
pos = pos';
vel = vel';
acc = acc';
% scale factor
vel = vel*0.1;
acc = acc*0.01;
fig = figure(1);

plot3(pos_plot(:,1),pos_plot(:,2),pos_plot(:,3), 'LineWidth',1.0)
xlabel('x');ylabel('y');zlabel('z')
axis equal
grid on
hold on
% Velocity
for i=1:length(vel)
    traj = plot3([pos(i,1),pos(i,1)+vel(i,1)],[pos(i,2),pos(i,2)+vel(i,2)], [pos(i,3),pos(i,3)+vel(i,3)], '-r');
    traj.Color(4) = 0.6;
end
% Acceleration
for i=1:length(acc)
    traj = plot3([pos(i,1),pos(i,1)+acc(i,1)],[pos(i,2),pos(i,2)+acc(i,2)], [pos(i,3),pos(i,3)+acc(i,3)], '-b');
    traj.Color(4) = 0.6;
end
% Way points
for i=1:size(PATH, 1)
    plot3(PATH(i,1),PATH(i,2),PATH(i,3), '*g', 'MarkerSize',20.0)
end
hold off
