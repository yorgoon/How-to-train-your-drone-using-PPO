function sdot = Dynamics(t,s,S)

% psi = s(1:8);
% s(8) = t*pi/180;
% s(16) = 1*pi/180;
% final_angle = 5;
% if t>final_angle
%     s(8) = final_angle*pi/180;
%     s(16) = 0;
% end
% psi_dot = s(9:16);
sdot = zeros(16,1);
B = inertia2(s(1:8),S);
C = coriolis3(s(1:8),s(9:16),S);
g_psi = potential2(s(1:8),S);

R = eulerZYX(s(4:6));
% r = 4
% p = 5
% y = 6
Q =[ 1,       0,       -sin(s(5));
     0,  cos(s(4)), cos(s(5))*sin(s(4));
     0, -sin(s(4)), cos(s(5))*cos(s(4))];

Q_inv =[ 1, (sin(s(5))*sin(s(4)))/cos(s(5)), (cos(s(4))*sin(s(5)))/cos(s(5));
        0,                 cos(s(4)),                -sin(s(4));
        0,          sin(s(4))/cos(s(5)),          cos(s(4))/cos(s(5))];

% Total weight
m_total = S.mb + S.m1 + S.m2;

% q1d = 90/180*pi;
% q1d_dot = 0;
% q1d_2dot = 0;
q1d = -1/2*sin(2*pi*t)+pi/2;
q1d_dot = -1/2*2*pi*cos(2*pi*t);
q1d_2dot = 1/2*(2*pi)^2*sin(2*pi*t);

% q2d = 45/180*pi;
% q2d_dot = 0;
% q2d_2dot = 0;
q2d = sin(2*pi*t);
q2d_dot = 2*pi*cos(2*pi*t);
q2d_2dot = -(2*pi)^2*sin(2*pi*t);


error_q1 = s(7) - q1d;
error_q2 = s(8) - q2d;

error_q1dot = s(15) - q1d_dot;
error_q2dot = s(16) - q2d_dot;

error_s = [s(1:6); error_q1; error_q2];
error_sdot = [s(9:14); error_q1dot; error_q2dot];

Kp1 = 2*eye(3);
Kp2 = 1*eye(3);
Kp3 = 20*eye(2);
Kp = blkdiag(Kp1,Kp2,Kp3);

Kv1 = 2*eye(3);
Kv2 = 1*eye(3);
Kv3 = 20*eye(2);
Kv = blkdiag(Kv1,Kv2,Kv3);

psi_2dot_d = [zeros(6,1);q1d_2dot;q2d_2dot];

tau = B*(psi_2dot_d -Kv*error_sdot - Kp*error_s) + C*s(9:16) + g_psi;

% tau1 = 0.001*sin(pi*t);
% f = [ones(1,4)*m_total*S.g/4, tau1, 0]';

% Omega = [zeros(2,4);ones(1,4);...
%         0 S.d 0 -S.d;...
%         S.d 0 -S.d 0;...
%         S.c -S.c S.c -S.c];
% R_bar = blkdiag(R, Q_inv, eye(2));
% N = blkdiag(Omega, eye(2));
% f = (N'*N)\N'*inv(R_bar)*tau;
% figure(3)
% subplot(2,1,1)
% plot(t,f(1),'.r',t,f(2),'.g',t,f(3),'.b',t,f(4),'.c')
% grid on
% hold on
% subplot(2,1,2)
% plot(t,f(5),'.r',t,f(6),'.g')
% grid on
% hold on
% fb = Omega * f(1:4);
% u = zeros(8,1);
% u(1:3) = fb(3) * R(:,3);
% u(4:6) = Q_inv * fb(4:6);
% u(7:8) = f(5:6);
% add noise
mu = 0;
sigma_xyz = .001;
noise_xyz = mu + sigma_xyz*randn(3,1);
sigma_rpy = .01;
noise_rpy = mu + sigma_rpy*randn(3,1);
sigma_q = .01;
noise_q = mu + sigma_q*randn(2,1);

sdot(1:8) = s(9:16);
sdot(9:16) = B\(tau - g_psi - C*s(9:16));

% sdot(9:11) = sdot(9:11) + noise_xyz;
% sdot(12:14) = sdot(12:14) + noise_rpy;
% sdot(15:16) = sdot(15:16) + noise_q;
end

