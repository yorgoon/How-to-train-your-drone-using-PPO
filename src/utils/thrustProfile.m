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
thrust = (S.mb)*S.g/4;thrust_max = 15;

X = -5:0.01:5;
a = (thrust - thrust_max/2)/(thrust_max/2);
% inverse of tanh
offset = 1/2*log((1+a)/(1-a));


scale = 1/8;
f = tanh(X*scale+offset)*thrust_max/2 + thrust_max/2;

% Sigmoid
scale = 1/2.5;
offset = 1/scale*log((thrust/thrust_max)/(1-thrust/thrust_max));

f3 = sigmoid(scale*(X + offset))*thrust_max;

f1 = tanh(X+1.5)*(thrust/2) + thrust/2;
f2 = tanh(X-1.5)*(thrust_max/2-(thrust/2)) + thrust_max/2-thrust/2;
% plot(X,f)
hold on
grid on
yline(thrust)
plot(X,f1+f2)
plot(X,f3)

function f = sigmoid(x)
f = 1./(1+exp(-x));
end