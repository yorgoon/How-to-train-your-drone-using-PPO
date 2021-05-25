% Barrel roll
r = 1;
roll_num = 3;
pre = 10;
gamma = 20000;
theta = linspace(-pi/2,roll_num*2*pi-pi/2,roll_num*4+1);
x = r.*cos(theta);
y = linspace(pre,2*pre,length(theta));
z = r.*sin(theta)+r;
PATH1 = [x',y',z';5,y(end)+1,0];
PATH1 = PATH1+[5,-9,0];
PATH = [zeros(1,3);PATH1];
Tau_vec = desired_trajectory(PATH, gamma)';

%%
Tau_vec(2:end-1) = 0.4;
Tau_vec(1) = 1.75;
Tau_vec(end) = 1.75;

% Trajectory
traj = MinimumSnapTrajectory(Tau_vec, PATH);
P = traj.P;
t = sum(Tau_vec);
dt = 0.1;
ts = 0:dt:t;
pos = [];
vel = [];
acc = [];
for i=1:length(ts)
    desired_state = desired_state_optimal(Tau_vec, ts(i), PATH, P);
    pos = [pos, desired_state.pos];
    vel = [vel, desired_state.vel];
    acc = [acc, desired_state.acc];
end
pos = pos';
vel = vel';
acc = acc';
% scale factor
vel = vel*0.5;
acc = acc*0.1;
fig = figure(1);

plot3(pos(:,1),pos(:,2),pos(:,3), 'LineWidth',2.0)
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
