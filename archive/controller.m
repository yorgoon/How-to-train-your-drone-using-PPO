function tau = controller(s,t,S)
B = inertia2(s(1:8),S);
C = coriolis3(s(1:8),s(9:16),S);
g_psi = potential2(s(1:8),S);
q1d = 45/180*pi;
q1d_dot = 0;
q1d_2dot = 0;
% q1d = -1/2*sin(2*pi*t)+pi/2;
% q1d_dot = -1/2*2*pi*cos(2*pi*t);
% q1d_2dot = 1/2*(2*pi)^2*sin(2*pi*t);

q2d = 35/180*pi;
q2d_dot = 0;
q2d_2dot = 0;
% q2d = sin(2*pi*t);
% q2d_dot = 2*pi*cos(2*pi*t);
% q2d_2dot = -(2*pi)^2*sin(2*pi*t);

error_q1 = s(7) - q1d;
error_q2 = s(8) - q2d;

error_q1dot = s(15) - q1d_dot;
error_q2dot = s(16) - q2d_dot;

error_s = [s(1:6); error_q1; error_q2];
error_sdot = [s(9:14); error_q1dot; error_q2dot];

Kp1 = 5*eye(3);
Kp2 = 5*eye(3);
Kp3 = 20*eye(2);
Kp = blkdiag(Kp1,Kp2,Kp3);

Kv1 = 2*eye(3);
Kv2 = 1*eye(3);
Kv3 = 20*eye(2);
Kv = blkdiag(Kv1,Kv2,Kv3);

psi_2dot_d = [zeros(6,1);q1d_2dot;q2d_2dot];

tau = B*(psi_2dot_d - Kv*error_sdot - Kp*error_s) + C*s(9:16) + g_psi;
end

