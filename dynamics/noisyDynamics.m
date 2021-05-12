function sdot = noisyDynamics(t,s,S,nominal_t,nominal_x)

% s(1:16) = (linear_interpol(nominal_t, nominal_x, t))';

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

q1d = 90/180*pi;
q1d_dot = 0;
q1d_2dot = 0;

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

% tau2 = B*(psi_2dot_d -Kv*error_sdot - Kp*error_s) + C*s(9:16) + g_psi;

tau = linear_interpol(nominal_t, nominal_x, t);
tau = tau';
% add noise
mu = 0;
sigma_xyz = .01;
noise_xyz = mu + sigma_xyz*randn(3,1);
sigma_rpy = .01;
noise_rpy = mu + sigma_rpy*randn(3,1);
sigma_q = .05;
noise_q = mu + sigma_q*randn(2,1);

% tau(7:8) = tau(7:8) + noise_q;
% tau2(7:8) = tau(7:8) + noise_q;
sdot(1:8) = s(9:16);
sdot(9:16) = B\(tau - g_psi - C*s(9:16));
% sdot(1:3) = sdot(1:3) + noise_xyz;
% sdot(4:6) = sdot(4:6) + noise_rpy;
% sdot(7:8) = sdot(7:8) + noise_q;
% sdot(9:11) = sdot(9:11) + noise_xyz;
% sdot(12:14) = sdot(12:14) + noise_rpy;
% sdot(15:16) = sdot(15:16) + noise_q;
end

