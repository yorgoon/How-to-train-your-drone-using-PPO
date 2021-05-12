syms q1 q2 l1 l2 mb m1 m2 q1_dot q2_dot r p y r_dot p_dot y_dot r_2dot p_2dot y_2dot Ix Iy Iz real
syms xb yb zb xb_dot yb_dot zb_dot real
% State
psi = [xb yb zb r p y q1 q2]';
% Time derivative of state
psi_dot = [xb_dot yb_dot zb_dot r_dot p_dot y_dot q1_dot q2_dot]';
% Inertia matrix for UAV main body
Ib = [Ix 0 0;0 Iy 0;0 0 Iz];
% Inertia matrix for link 1
I1 = [0 0 0;0 1 0;0 0 1]*1/12*m1*l1^2;
% Inertia matrix for link 2
I2 = [0 0 0;0 1 0;0 0 1]*1/12*m2*l2^2;

%%
% Position of link 1 wrt UAV
Eb0b = [eye(3),[0 0 -0.1]';0 0 0 1];
pb1 = ROTY(q1) * [l1/2 0 0]';
% Rotation of link 1 wrt UAV
Rb1 = ROTY(q1);
% Transformation matrix of link 1 wrt UAV
Eb1 = Eb0b * [Rb1, pb1;0 0 0 1];
pb1 = Eb1(1:3,4);

% Position of link 2 wrt link 1
p12 = [l1/2 + l2/2 * cos(q2);0;l2/2*sin(q2)];
% Rotation of link 2 wrt link 1
R12 = ROTY(-q2);
% Transformation matrix of link 2 wrt link 1
E12 = [R12, p12;0 0 0 1];
% Transformation matrix of link 2 wrt UAV
Eb2 = simplify(Eb1 * E12);
% Position of link 2 wrt UAV
pb2 = Eb2(1:3,4);
% Rotation of link 2 wrt UAV
Rb2 = Eb2(1:3,1:3);

% Test
q1double = 45/180*pi; % Angle of link 1
pb1double = double(subs(pb1,{q1, l1},{q1double, 1})); % Position of link 1
rads = linspace(0,2*pi,20);
x = zeros(length(rads),1);
z = zeros(length(rads),1);
for i=1:length(rads)
    E = double(subs(Eb2,{q1, q2, l1, l2},{q1double, rads(i), 1, 0.25}));
    pxz = [E(1,4);E(3,4)];
    x(i) = pxz(1);
    z(i) = pxz(2);
end
plot(x,z,'.r','MarkerSize',20.0)
grid on
hold on
axis equal
line([Eb0b(1,4),2*pb1double(1)-Eb0b(1,4)],[Eb0b(3,4),2*pb1double(3)-Eb0b(3,4)],'LineWidth',2.0)
for i=1:length(rads)
    line([2*pb1double(1)-Eb0b(1,4),2*x(i) - (2*pb1double(1)-Eb0b(1,4))],[2*pb1double(3)-Eb0b(3,4),2*z(i) - (2*pb1double(3)-Eb0b(3,4))],'LineWidth',2.0)
end

%% Position Jacobian pbi_dot = Jip * [q1_dot q2_dot]';
J1p = jacobian(pb1,[q1,q2]);
J2p = jacobian(pb2,[q1,q2]);
%% Angular velocity of link 1 wrt UAV body frame
% Angular vel of link 1 wrt UAV
wb1_hat = simplify(diff(Rb1,q1)*Rb1')*q1_dot;
wb1 = vee_operator(wb1_hat);
% Angular Jacobian
J1o = [0 0;1 0;0 0];
% verify jacobian: J1o * [q1_dot q2_dot]' - wb1
% Angular vel of link 2 wrt UAV
wb2_hat = simplify(diff(Rb2,q1)*Rb2')*(q1_dot - q2_dot);
wb2 = vee_operator(wb2_hat);
J2o = [0 0;1 -1;0 0];
% verify jacobian: J2o * [q1_dot q2_dot]' - wb2

%% Angular velocity of UAV wrt world
% Rotation of UAV wrt world: Yaw-Pitch-Roll sequence
R = simplify(ROTZ(y)*ROTY(p)*ROTX(r),100);
% R_dot using Chain rule
R_dot = diff(R,r)*r_dot + diff(R,p)*p_dot + diff(R,y)*y_dot;
R_2dot = diff(R_dot,r)*r_dot + diff(R_dot,p)*p_dot + diff(R_dot,y)*y_dot + ...
         diff(R_dot,r_dot)*r_2dot + diff(R_dot,p_dot)*p_2dot + diff(R_dot,y_dot)*y_2dot;
% Angular vel of UAV wrt world wb_hat = R_dot * R^-1
wb_hat = simplify(R_dot*R',100);
wb = vee_operator(wb_hat);

wb_dot_hat = simplify(R_2dot*R'+R_dot*R_dot',100);
wb_dot = vee_operator(wb_dot_hat);

% wb = Tb * rpy_dot
Tb = jacobian(wb,[r_dot, p_dot, y_dot]); % verify: Tb * [r_dot p_dot y_dot]' - wb
Tb2 = jacobian(wb_dot,[r_2dot, p_2dot, y_2dot]);
% wbb = R^T * Tb * rpy_dot
% Q = R^T * Tb
Q = simplify(R'*Tb,100);
%% Mass matrix, B
Rpb1_hat = hat_operator(simplify(R*pb1,1000));
Rpb2_hat = hat_operator(simplify(R*pb2,1000));
B21 = sym('B21',[3 3],'real')
% B21 = -Tb' * (Rpb1_hat' * m1 + Rpb2_hat' * m2);
B21 = simplify(-Tb' * Rpb1_hat' * m1,'Steps',1000) + simplify(-Tb' * Rpb2_hat' * m2,'Steps',1000);
%%
B21 = simplify(B21);

%%
B31 = sym('B31',[2 3],'real');
B31 = (m1 * J1p' + m2 * J2p') * R';
B31 = simplify(B31,100);
%%
% B22 = simplify(Tb' * Rpb1_hat' * Rpb1_hat * Tb,1000);
% B22(3,3) = 1/4 *l1^2 *((cos(q1))^2 *((cos(p))^2 *((cos(r))^2 + 1) - 1) - 2 *sin(p) *...
%     cos(p) *sin(q1) *cos(q1) *cos(r) - (cos(p))^2 *(cos(r))^2 + 1);
% 
% AA = simplify(Tb' * Rpb2_hat',1000);
% B22 = B22*m1 + simplify(AA*AA',100)*m2;
% %% Thin rod assumption
% B222 = Q'*Rb1*[0 0 0;0 1 0;0 0 1]*Rb1'*Q;
% B222 = simplify(B222)
% B22_end_alt = -(cos(p)*sin(q1)*cos(r) + sin(p)*cos(q1)-1)*(cos(p)*sin(q1)*cos(r) + sin(p)*cos(q1)+1)
% rand_angles = [rand, rand, rand];
% double(subs(B222(end),{r, p, q1},{rand_angles})) - double(subs(B22_end_alt,{r, p, q1},{rand_angles}))
% 
% B22_23 = sin(q1) *sin(r) *(cos(p) *sin(q1) *cos(r) + sin(p) *cos(q1))
% double(subs(B222(2,3),{r, p, q1},{rand_angles})) - double(subs(B22_23,{r, p, q1},{rand_angles}))
% 
% B222(end) = B22_end_alt;
% B222(2,3) = B22_23;
% B222(3,2) = sin(q1) *sin(r) *(cos(p)* sin(q1)* cos(r) + sin(p)* cos(q1));
% B222 = B222*1/12*m1*l1^2;
% B222 = B222 + Q'*Rb2*I2*Rb2'*Q;
% 
% B22 = Q'*Ib*Q + B22 + B222;
B22 = sym('B22',[3 3],'real');
B22 = Q'*(Ib + Rb1*I1*Rb1' + Rb2*I2*Rb2')*Q + Tb'*(m1*Rpb1_hat'*Rpb1_hat + m2*Rpb2_hat'*Rpb2_hat)*Tb;
B22 = simplify(B22);
%%
disp('B22...')
B22_alt = sym('B22_alt',[3 3],'real');
B22_alt = simplify(Q'*(Ib + simplify(Rb1*I1*Rb1') + simplify(Rb2*I2*Rb2'))*Q) + simplify(Tb'*(m1*simplify(Rpb1_hat'*Rpb1_hat) + m2*simplify(Rpb2_hat'*Rpb2_hat))*Tb);
disp('B22!')
%% B23
% B23_1 = simplify(J1p'*R'*Rpb1_hat*Tb,'Steps',1000)*(-m1);
% B23_12 = simplify(J2p'*R'*Rpb2_hat*Tb,'Steps',1000)*(-m2);
% B23_2 = J1o'*Rb1*I1*Rb1'*Q;
% B23_22 = J2o'*Rb2*I2*Rb2'*Q;
% B32 = B23_1 + B23_2 + B23_12 + B23_22;
% B32 = sym('B32',[2 3],'real');
% B32 = -(J1p'*R'*Rpb1_hat*Tb*m1 + J2p'*R'*Rpb2_hat*Tb*m2) + (J1o'*Rb1*I1*Rb1' + J2o'*Rb2*I2*Rb2')*Q;
B32_alt = sym('B32_alt',[2 3],'real');
B32_alt = -simplify(simplify(J1p'*R')*simplify(Rpb1_hat*Tb)*m1 + simplify(J2p'*R')*...
    simplify(Rpb2_hat*Tb*m2)) + simplify((simplify(J1o'*Rb1)*simplify(I1*Rb1') + simplify(J2o'*Rb2)*I2*Rb2')*Q);
disp('B23!')
%% B33

B33 = simplify(simplify(J1o'*Rb1*I1*Rb1'*J1o + J1p'*J1p*m1)+simplify(J2o'*Rb2*I2*Rb2'*J2o + J2p'*J2p*m2));

%%
B = [(mb+m1+m2)*eye(3) B21' B31';B21 B22_alt B32_alt';B31 B32_alt B33];
%%
for i=1:length(B(:))
    B(i) = simplify(B(i));
end
%% C
C = zeros(8);
C = sym(C);
disp('C...')
for i=1:8
    for j=1:8
        for k=1:8
            C(i,j) = C(i,j)+1/2*(simplify(diff(B(i,j),psi(k))) + simplify(diff(B(i,k),psi(j))) - simplify(diff(B(k,j),psi(i))))*psi_dot(k);
            C(i,j) = simplify(C(i,j));
        end
    end
end
disp('C!')

%% Potential Energy term
syms g real
pb = [xb yb zb]';
Ub = mb*g*[0 0 1]*pb;
U1 = m1*g*[0 0 1]*(pb + R*pb1);
U2 = m2*g*[0 0 1]*(pb + R*pb2);
U = Ub + U1 + U2;
%%
g_psi = jacobian(U,psi)';
g_psi = simplify(g_psi);
%%
fileID = fopen('exp3.txt','w');
CC = char(C);
fprintf(fileID,'%s',CC)
fclose(fileID)
%% Model parameters
S.mb = 1.477;
S.d = 0.263;
S.c = 8.004e-4;
S.Ib = [0.01152; 0.01152; 0.0218];
S.m1 = 0.05;
S.m2 = 0.025;
S.l1 = 0.5;
S.l2 = 0.5;
S.g = 9.81;

% Q_inv = simplify(inv(Q));
%
s0 = zeros(8*2,1);
% s0(1) = -.5;
s0(7) = pi/2;
% s0(8) = pi/4;
dt = 1/2000;
t_sim = 0:dt:10;
disp('running...')
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);
[ts, xsave] = ode45(@(t,s) Dynamics(t,s,S), t_sim, s0, opts);
disp('completed')

f = [];
input = [];
input2 = [];
for i=1:length(ts)-1
    
    RR = eulerZYX(xsave(i,4:6));
    Q_inv =[ 1, (sin(xsave(i,5))*sin(xsave(i,4)))/cos(xsave(i,5)), (cos(xsave(i,4))*sin(xsave(i,5)))/cos(xsave(i,5));
        0,                 cos(xsave(i,4)),                -sin(xsave(i,4));
        0,          sin(xsave(i,4))/cos(xsave(i,5)),          cos(xsave(i,4))/cos(xsave(i,5))];

    Omega = [zeros(2,4);ones(1,4);...
        0 S.d 0 -S.d;...
        S.d 0 -S.d 0;...
        S.c -S.c S.c -S.c];
    BB = inertia2(xsave(i,1:8),S);
    CCC = coriolis3(xsave(i,1:8),xsave(i,9:16),S);
    gg_psi = potential2(xsave(i,1:8),S);

    R_bar = blkdiag(RR, Q_inv, eye(2));
    N = blkdiag(Omega, eye(2));
    
    input(:,i) = BB*(diff(xsave(i:i+1,9:16))/diff(ts(i:i+1)))' + CCC*xsave(i,9:16)'+gg_psi;
    
    f(i,:) = (N'*N)\N'*inv(R_bar)*input(:,i);
    input2(:,i) = R_bar * N * f(i,:)';
end
%%
plot_allstuff(xsave,ts,S)

%%
object = OptimalControlProject(rand(8,1));
%% Model parameters
S.mb = 1.477;
S.d = 0.263;
S.c = 8.004e-4;
S.Ib = [0.01152; 0.01152; 0.0218];
S.m1 = 0.05;
S.m2 = 0.025;
S.l1 = 0.5;
S.l2 = 0.5;
S.g = 9.81;

% 20ms 
S.dt = 1/500;

% Time steps
ts = 0:S.dt:10;

% State dimension n = 16;
S.n = 16;

% Input dimension m = 8;
S.m = 8;

% True state
state_true = zeros(S.n, length(ts));

% Initial condition
state_true(7,1) = pi/2;

% state estimate
state_est = state_true;

% Uncertainty parameters
mu = 0;
sigma_xyz = .1;
sigma_rpy = .01;
sigma_q = .05;

Q1 = blkdiag(1e-12*eye(3),1e-12*eye(3),1e-10*eye(2));
Q2 = blkdiag(1e-12*eye(3),1e-12*eye(3),1e-10*eye(2));
S.Q = blkdiag(Q1,Q2);
% Q = 1e-10*eye(16);
% P1 = blkdiag(0.000000001*eye(3),0.0000000000001*eye(3),0.00000001*eye(2));
% P2 = blkdiag(0.000000001*eye(3),0.0000000000001*eye(3),0.00000001*eye(2));
% P = blkdiag(P1,P2);
S.P = S.Q;
S.R = 0.00000001*eye(16);

% Initial Guess
state_est(:,1) = state_true(:,1) + sqrt(S.P)*randn(16,1);

% history of state estimate covariance
Ps(:, :, 1) = S.P;

% Simulation
for i=1:length(ts)-1
	t = ts(i);
    if mod(t,1) == 0
        formatSpec = "Current simulation time: %d %s";
        A2 = 'seconds';
        str = sprintf(formatSpec,t,A2);
        fprintf(str + '\n')
    end
    
    x_true = state_true(:,i);
    x_correct = state_est(:,i);
%     input = controller(x_true,t,S); % Controller using true state
    input = controller(x_correct,t,S); % Controller using state estimate
    
    % True evolution of dynamics
    x_true = discreteDynamics(x_true, input, S) + sqrt(S.Q)*randn(16,1);
    state_true(:,i+1) = x_true;
    
    % Prediction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     x_predict = discreteDynamics(x_correct, input, S);
%     F = stateJacobian(x_correct,input,S);
%     P = F*S.P*F' + S.Q;
    x_predict = ekf_predict(x_estimate, input, S);
    
    % Correction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z = x_true + sqrt(S.R) * randn(16,1);
    z_predict = x_predict;
    H = eye(16);

    % Update covariance
    % S.P = P - P*H'*((H*P*H' + R)\H*P); 
    S.P = P - P*H'*inv(H*P*H' + S.R)*H*P;

    % Kalman Gain
    K = S.P * H'/S.R; % K = P*H'*inv(R);

    x_correct = x_predict + K*(z - z_predict);

    state_est(:,i+1) = x_correct;

end
disp('Simulation completed.')
%%
plot_allstuff(state_true(:,1:end-1),ts(1:end-1),S)
%%
error = state_true - state_est;
figure(4)
subplot(3,1,1)
plot(ts,error(1,:),ts,error(2,:),ts,error(3,:))
grid on
legend('x','y','z')
subplot(3,1,2)
plot(ts,error(4,:),ts,error(5,:),ts,error(6,:))
grid on
legend('r','p','y')
subplot(3,1,3)
plot(ts,error(8,:),ts,error(8,:))
grid on
legend('q_1','q_2')
%% Ps(:,:,i+1) = P;
normPs = [];
for i=1:length(ts)
    normPs(i) = norm(Ps(:,:,i));
end
plot(normPs)
%%
% External force
start_T = 0;
duration = 0;
mag = 0;
direction = [1 0 0]';
Fmat = extForce_gen(ts, start_T, duration, mag, direction);

filename = 'myVideo.avi';
video_gen(ts(1:end-1), state_true(:,1:end-1)', S, filename, 30, Fmat)
%%
syms f1 f2 f3 f4 tau1 tau2 d c real;
Omega = [zeros(2,4);ones(1,4);0 d 0 -d;d 0 -d 0;c -c c -c]
N = blkdiag(Omega, eye(2))
f = [f1 f2 f3 f4 tau1 input]'
N*f