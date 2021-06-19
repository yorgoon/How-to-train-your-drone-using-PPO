[Tau_vec, PATH] = canopyRoll();

% Trajectory
traj = MinimumSnapTrajectory(Tau_vec, PATH);
P = traj.P;
t = sum(Tau_vec);
dt = 0.05;
ts = 0:dt:t;
ts_pos = 0:0.001:t;
pos_plot = [];
pos = [];
vel = [];
acc = [];
for i=1:length(ts_pos)
    desired_state = desiredState(traj, ts_pos(i));
    pos_plot = [pos_plot, desired_state.pos];
end
for i=1:length(ts)
    desired_state = desiredState(traj, ts(i));
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
acc = acc*0.1;
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
%%
vel_acc_rad = zeros(1,length(vel));
for i=10:length(vel)
    vel_acc_rad(i) = acos(getCosineSimilarity(vel(i,:),acc(i,:)));
end
plot(vel_acc_rad/pi*180)