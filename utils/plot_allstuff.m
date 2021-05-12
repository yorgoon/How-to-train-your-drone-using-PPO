function plot_allstuff(x_hist, state_est, u_hist, ts, S)

f = zeros(6,length(ts));
u_remapped = zeros(size(u_hist));
for i=1:length(ts)
    s = x_hist(i,:);
    R = eulerZYX(s(4:6));
    Q_inv = ...
    [1, (sin(s(5))*sin(s(4)))/cos(s(5)), (cos(s(4))*sin(s(5)))/cos(s(5));
     0, cos(s(4)), -sin(s(4));
     0, sin(s(4))/cos(s(5)), cos(s(4))/cos(s(5))];
    R_bar = blkdiag(R, Q_inv, eye(2));
    Omega = [zeros(2,4);ones(1,4);...
            0 S.d 0 -S.d;...
            S.d 0 -S.d 0;...%     U1(i) = u1;
%     U2(i) = u2;
%     U3(i) = u3;
%     U4(i) = u4;

            S.c -S.c S.c -S.c];
    N = blkdiag(Omega, eye(2));
    f(:,i) = (N'*N)\N'*inv(R_bar)*u_hist(:,i);
    F = sum(f(1:4,i));
%     u_remapped(:,i) = R_bar * N * f(:,i);
    
    R2 = [ cos(s(6)), -sin(s(6)), 0; sin(s(6)),  cos(s(6)), 0; 0, 0, 1];
    R_bar2 = blkdiag(R2, Q_inv, eye(2));
    u_remapped(:,i) = R_bar2 * N * f(:,i);
end

% figure(1)
% plot3(x_hist(:,1),x_hist(:,2),x_hist(:,3),'--')
% xlabel('x');ylabel('y');zlabel('z')
% grid on
% axis equal
% view([0,0])
%
figure(2)
subplot(3,1,1)
plot(ts, x_hist(:,1),'r', ts, x_hist(:,2),'g', ts, x_hist(:,3),'b')
% hold on
% plot(ts, state_est(:,1),'m', ts, state_est(:,2),'y', ts, state_est(:,3),'c')
title('True state evolution')
grid on
ylabel('meter')
legend('x','y','z')
subplot(3,1,2)
plot(ts, x_hist(:,4)*180/pi,'r', ts, x_hist(:,5)*180/pi,'g', ts, x_hist(:,6)*180/pi,'b')
grid on
ylabel('degree')
legend('roll','pitch','yaw')
subplot(3,1,3)
plot(ts, x_hist(:,7)*180/pi, 'r', ts, x_hist(:,8)*180/pi, 'b')
grid on
ylabel('degree')
legend('q_1','q_2')
% subplot(5,1,4)
% plot(ts(1:end-1),f(:,1),'-r',ts(1:end-1),f(:,2),'-g',ts(1:end-1),f(:,3),'-b',ts(1:end-1),f(:,4),'-c')
% grid on
% ylabel('Thrust, N')
% legend('1(-x)','2(+y)','3(+x)','4(-y)')
% subplot(5,1,5)
% plot(ts(1:end-1),f(:,5),'-r',ts(1:end-1),f(:,6),'-b')
% grid on
% ylabel('Torque, Nm')
% legend('1','2')
% hold off
%
figure(3)
subplot(4,1,1)
plot(ts, u_hist(1,:), '-r', ts, u_hist(2,:), '-g', ts, u_hist(3,:), '-b')
title('Input')
grid on
ylabel('Force, N')
legend('x','y','z')
% subplot(4,2,2)
% plot(ts, u_remapped(1,:), '-r', ts, u_remapped(2,:), '-g', ts, u_remapped(3,:), '-b')
% grid on
% legend('x','y','z')
subplot(4,1,2)
plot(ts, u_hist(4,:), '-r', ts, u_hist(5,:), '-g', ts, u_hist(6,:), '-b')
grid on
ylabel('Torque, Nm')
legend('roll','pitch','yaw')
% axis([0 15 -0.1 0.1])
% subplot(4,2,4)
% plot(ts, u_remapped(4,:), '-r', ts, u_remapped(5,:), '-g', ts, u_remapped(6,:), '-b')
% grid on
% legend('roll','pitch','yaw')
% subplot(4,2,5)
% plot(ts, u_hist(7,:),'-r', ts, u_hist(8,:), '-b')
% grid on
% legend('q_1','q_2')
% subplot(4,2,6)
% plot(ts, u_remapped(7,:), '-r', ts, u_remapped(8,:), '-b')
% grid on
% legend('q_1','q_2')
subplot(4,1,3)
plot(ts,f(1,:),'-r',ts,f(2,:),'-g',ts,f(3,:),'-b',ts,f(4,:),'-m')
grid on
ylabel('Force, N')
legend('f_1(x_-)','f_2(y_+)','f_3(x_+)','f_4(y_-)')
% axis([0 15 -0.1 10])
subplot(4,1,4)
plot(ts, x_hist(:,7)*180/pi, 'r', ts, x_hist(:,8)*180/pi, 'b')
grid on
xlabel('time, sec'); ylabel('degree')
legend('q_1','q_2')

error = x_hist - state_est;
error = error';
figure(4)
subplot(3,1,1)
plot(ts,error(1,:),'-r',ts,error(2,:),'-g',ts,error(3,:),'-b')
grid on
legend('x','y','z')
subplot(3,1,2)
plot(ts,error(4,:),'-r',ts,error(5,:),'-g',ts,error(6,:),'-b')
grid on
legend('r','p','y')
subplot(3,1,3)
plot(ts,error(7,:),'-r',ts,error(8,:),'-b')
grid on
legend('q_1','q_2')
end