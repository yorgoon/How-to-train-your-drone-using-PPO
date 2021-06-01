% Model parameters
S.g = 9.807; % Gravity
S.mb = 1.477; % Mass of UAV
S.d = 0.263; % Arm Length (Rotor and COM of UAV)
S.c = 8.004e-4; % Drag Factor
S.Ib = [0.01152; 0.01152; 0.0218]; % Moment of Inertia of UAV
% S.Ib = [0.01152 0 0;0 0.01152 0;0 0 0.0218];
S.m1 = 0.05; % Mass of First Link
S.m2 = 0.05; % Mass of Second Link
S.l1 = 0.5; % Length of First Link
S.l2 = 0.5; % Length of Second Link
thrust = (S.mb)*S.g/4;
X = -5:0.01:5;
f = tanh(X)*5 + 5;
f1 = tanh(X+1.5)*(thrust/2-1/2) + thrust/2+1/2;
f2 = tanh(X-1.5)*(4.5-(thrust/2-1/2)) + 4.5-thrust/2+1/2;
f1 = tanh(X+1)*(thrust/2) + thrust/2;
f2 = tanh(X-1)*(5-(thrust/2)) + 5-thrust/2;
plot(X,f)
hold on
grid on
yline(thrust)
plot(X,f1+f2)
hold off