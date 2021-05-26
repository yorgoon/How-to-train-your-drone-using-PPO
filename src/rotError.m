function eR = rotError(current_state, desired_state, S)

% Control Parameters
K.Kp = 11.9;
K.Kv = 4.443;

% Gain
% KK.Kp = 2500;%11.9;
% KK.Kv = 750;%4.443;
% KK.KR = 15000;%10;
% KK.K_omega = 7300;%6;

% Define the environment constants.
g = S.g;
m = S.mb;
Kp = K.Kp;
Kv = K.Kv;

% position error
ep = current_state(1:3)-desired_state.pos;

% velocity error
ev = current_state(4:6)-desired_state.vel;

% desired force, F_des
Fd = -Kp.*ep -Kv.*ev + m*g*[0 0 1]' + m*desired_state.acc;

% desired input u1
yaw = current_state(9);
roll = current_state(7);
pitch = current_state(8);
R = ROTZ(yaw)*ROTX(roll)*ROTY(pitch); % Current rotation

% desired rotation, Rd = [xbd ybd zbd]
zbd = Fd/norm(Fd);


% ybd = zbd X xcd/norm(zbd X xcd)
% xcd
yawd = desired_state.yaw;

xcd = [cos(yawd) sin(yawd) 0]';
ybd = hat(zbd)*xcd/norm(hat(zbd)*xcd);
% xbd = ybd X zbd
xbd = hat(ybd)*zbd;
Rd1 = [xbd ybd zbd];

% eR orientation error
R1 = (Rd1'*R - R'*Rd1);
eR1 = 1/2*vee(R1);
eR = eR1;

end