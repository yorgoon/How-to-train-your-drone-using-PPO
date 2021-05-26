function mat = eulerZYX(euler_angles)
r = euler_angles(1);
p = euler_angles(2);
y = euler_angles(3);
mat = ROTZ(y)*ROTY(p)*ROTX(r);
end