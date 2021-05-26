function g_psi = potential(psi,S)

xb = psi(1);
yb = psi(2);
zb = psi(3);
r = psi(4);
p = psi(5);
y = psi(6);
q1 = psi(7);
q2 = psi(8);

l1 = S.l1;
l2 = S.l2;
m1 = S.m1;
m2 = S.m2;
mb = S.mb;
Ix = S.Ib(1);
Iy = S.Ib(2);
Iz = S.Ib(3);
g = S.g;

g_psi = ...
[0;
 0;
 g*(m1 + m2 + mb);
 (g*cos(p)*sin(r)*(l1*m1*sin(q1) + 2*l1*m2*sin(q1) + l2*m2*sin(q1 - q2)))/2;
 -g*m2*(cos(p)*(l1*cos(q1) + (l2*cos(q1 - q2))/2) - cos(r)*sin(p)*(l1*sin(q1) + (l2*sin(q1 - q2))/2)) - (g*l1*m1*(cos(p)*cos(q1) - cos(r)*sin(p)*sin(q1)))/2;
 0;
 g*m2*(sin(p)*(l1*sin(q1) + (l2*sin(q1 - q2))/2) - cos(p)*cos(r)*(l1*cos(q1) + (l2*cos(q1 - q2))/2)) + (g*l1*m1*(sin(p)*sin(q1) - cos(p)*cos(q1)*cos(r)))/2;
 -(g*l2*m2*(sin(q1 - q2)*sin(p) - cos(q1 - q2)*cos(p)*cos(r)))/2];
 

end

