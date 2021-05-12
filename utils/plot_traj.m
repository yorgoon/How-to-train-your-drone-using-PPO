function plot_traj(wp, E)
s1 = E.s1-0.6;
r = E.r;
tau_vec = wp(end-2:end)';
wp_temp = wp;
wp_temp(end-2:end) = [];
wp_temp = reshape(wp_temp,[3,2]);
wp_temp = wp_temp';
PATH = [E.init;wp_temp;E.goal];
% Trajectory
traj_class = MinimumSnapTrajectory(tau_vec, PATH);
P = traj_class.P;
t = sum(tau_vec);
dt = 0.1;
ts = 0.01:dt:t*0.999;
pos = getPos(tau_vec, ts', traj_class.P);
vel = diff(pos)/dt;
% scale factor
vel = vel*0.5;
% Trajectory
fig = figure(1);
if size(pos, 2) == 2
    plot(pos(:,1),pos(:,2), 'LineWidth',3.0)
    xlabel('x');ylabel('y');
    axis equal
    grid on
    hold on
    % Velocity
    for i=1:length(vel)
        traj = plot([pos(i,1),pos(i,1)+vel(i,1)],[pos(i,2),pos(i,2)+vel(i,2)], '-r');
    	traj.Color(4) = 0.6;
    end
    % Way points
    for i=1:size(PATH, 1)
        plot(PATH(i,1),PATH(i,2), '*g', 'MarkerSize',20.0)
    end
    % Obstacles
    [x,y,z] = sphere;
    for i=1:length(s1)
        circle(r(i,1), r(i,2), s1(i));
        plot(r(i,1),r(i,2), '.r', 'MarkerSize',20.0)
    end
end

if size(pos, 2) == 3
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
    % Way points
    for i=1:size(PATH, 1)
        plot3(PATH(i,1),PATH(i,2),PATH(i,3), '*g', 'MarkerSize',20.0)
    end
    % Obstacles
    [x,y,z] = sphere;
    for i=1:length(s1)
        h = surfl(x*s1(i)+r(i,1),y*s1(i)+r(i,2),z*s1(i)+r(i,3));
        plot3(r(i,1),r(i,2),r(i,3), '.r', 'MarkerSize',20.0)
        axis equal
        set(h, 'FaceAlpha', 0.5)
        shading flat
    end
end

end

