function sdot = droneDynamics(state, action, S)

global Step Fext Mext;

% Define the environment constants.
g = S.g;
m = S.mb;
J = S.Ib;
L = S.d;
c_tf = S.c;

% Initialize state_dot.
sdot = zeros(12,1);

% Map thrust actions into force, moment actions.
mapping_u = [1 1 1 1;0 L 0 -L;-L 0 L 0;c_tf -c_tf c_tf -c_tf];
action = mapping_u * action;

% Total thrust
F = action(1);
% Generated Moment
M = [action(2), action(3), action(4)]';

R = ROTZ(state(9))*ROTX(state(7))*ROTY(state(8));
omega = [state(10) state(11) state(12)]';

sdot(1) = state(4);
sdot(2) = state(5);
sdot(3) = state(6);

zb = R*[0 0 1]';
accel_ext = Fext/m;
accel = -g*[0 0 1]' + F/m * zb + accel_ext;
sdot(4) = accel(1);
sdot(5) = accel(2);
sdot(6) = accel(3);

% Relationship between omega & euler angles.
mapping_R = [cos(state(8)) 0 -cos(state(7))*sin(state(8));...
             0         1            sin(state(7));...
             sin(state(8)) 0  cos(state(7))*cos(state(8))];

eulerdot = mapping_R\omega;
sdot(7) = eulerdot(1);
sdot(8) = eulerdot(2);
sdot(9) = eulerdot(3);

omegadot = J\(R*Mext + M - hat(omega)*J*omega);
sdot(10) = omegadot(1);
sdot(11) = omegadot(2);
sdot(12) = omegadot(3);
end