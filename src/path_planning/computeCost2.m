function J = computeCost2(wp, E)

s1 = E.s1;
r = E.r;
tau_vec = wp(end-2:end)';
wp(end-2:end) = [];
wp = reshape(wp,[3,2]);
wp = wp';
PATH = [E.init;wp;E.goal];

% For visualization purpose (moving obstacle)
% This case, wp becomes center of obstacle
% s1 = 0.5;
% s2 = 0.5;
% r = wp;
% tau_vec = [2 2]';
% PATH = [0 0 0;1 1 1;2 2 2];

% Reference trajectory
traj_class = MinimumSnapTrajectory(tau_vec, PATH);
t_total = sum(tau_vec);
ts = linspace(0.01,t_total*0.99,100);
dt = ts(2)-ts(1);
Kv = E.Kv; % Case for penalizing velocity
% Initial trajectory cost
J_traj = traj_class.J;
i = 0;
pos = getPos(tau_vec, ts, traj_class.P);
vel = diff(pos)/dt;
J_obs = 0;
for i=1:length(s1)    
    posrnorm_diff1 = vecnorm(pos-r(i,:),2,2) - s1(i);
%     posrnorm_diff2 = vecnorm(pos-r(i,:),2,2) - s2(i);
%     speed = vecnorm(vel(posrnorm_diff1 < 0,:),2,2);
%     avg_speed = Kv * norm(speed)/size(speed,1);
    % % 1st method
%     % % Distance cost
%     cost_d = 1./(vecnorm(pos(posrnorm_diff < 0,:)-r(i,:),2,2)); % for numeric stability
%     cost_d = cost_d.^2
%     % Sum it with gain multiplication
%     Jd = K*(sum(cost_d));
%     
%     % Final cost
%     J = J+Jd;
    
    % % 2nd method
    % Squared Penetration Depth
%     d1 =vecnorm(pos(posrnorm_diff1 > 0 & posrnorm_diff2 < 0,:)-r(i,:),2,2);
    d1 =vecnorm(pos(posrnorm_diff1 < 0,:)-r(i,:),2,2);
    % Subtract by obstacle radius
    d2 = s1(i) - d1;
    % Squared d2
    d22 = d2.^2;
    Jd = sum(d22);
    % Obstacle cost
    J_obs = J_obs+Jd;
    
    % % 3rd method
    % Weight and COM of the Penetrated Trajectory
%     seg_arc = pos(posrnorm_diff1 < 0,:);
%     if size(seg_arc,1)
%         % Find center of mass
% %         com = sum(seg_arc)/size(seg_arc,1);
%         com = mean(seg_arc);
%         % Distance between COM and center of obstacle
% %         d_com_r = norm(com - r(i,:));
%         % penetration depth
%         d_com_r = norm(com - r(i,:))-s1(i);
%         
%         diff_seg_arc = diff(seg_arc);
% 
%         mass_arc = sum(vecnorm(diff_seg_arc,2,2));
% 
% %         Jd = K/1.2*mass_arc/d_com_r^2;
%         Jd = Kob*(mass_arc*d_com_r)^2;
% 
%         J = J+Jd;
%     end
end
% Time cost
J_time = sum(tau_vec);
% Kob = 15000;
% Kt = 50;
Kob = E.Kob;
Kt = E.Kt;
% Compute cost
J = J_traj + Kob*J_obs + Kt*J_time;