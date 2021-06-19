function video_gen(fig, struct, filename, target_fps)

Fmat = struct.Fext';
Mmat = struct.Mext';
tsave = struct.t;
xsave = struct.state';
action = struct.action';
g_force = struct.g_force';
xxsave = struct.xsave;
fv = stlread('quadrotor_model/quadrotor_base.stl');
grid on;hold on;
subplot('Position',[0. 0 0.65 1])
Q = FramePlot(tsave(1), xsave(1,:), Fmat(1,:), fv);
% xlim([-10.5,10.5]);ylim([-6,6]);zlim([-0.5,0.5])

subplot('Position',[0.65 0.6 .3 .3])
% plot(tsave, g_force,'b');grid on;hold on;
% ylabel('G-Force, g')
plot(tsave, vecnorm(xxsave(1:3,:)),'b');grid on;hold on;
ylabel('||e_{pos}||')


subplot('Position',[0.65 0.15 .3 .3])
plot(tsave,action(:,1),'Color','red');hold on;grid on;
plot(tsave,action(:,2),'Color','green')
plot(tsave,action(:,3),'Color','blue')
plot(tsave,action(:,4),'Color','yello')

set(gcf,'Renderer','OpenGL')
v = VideoWriter(filename,'MPEG-4');
v.FrameRate = target_fps;
v.Quality = 50;
open(v)

given_fps = 1/(tsave(2)-tsave(1));
skip_interval = floor(given_fps/target_fps);
for i=2:skip_interval:length(xsave(:,1))
    subplot('Position',[0. 0 0.65 1])
    Q.UpdatePlot(tsave(i), xsave(i,:), Fmat(i,:))
    annotation('textbox', [0.1, 0.9, 0, 0], 'string', ['elapsed time: ', num2str(tsave(i))],'FitBoxToText','on','FontSize',20);
%     xlim([-10.5,10.5]);ylim([-6,6]);zlim([-0.5,0.5])
    
    subplot('Position',[0.65 0.6 .3 .3])
%     plot(tsave, g_force,'b');grid on;
%     ylabel('G-Force, g')
%     dot1=plot(tsave(i),g_force(i),'o','Color','red');
    plot(tsave, vecnorm(xxsave(1:3,:)),'b');grid on;
    ylabel('||e_{pos}||')
    dot1=plot(tsave(i), vecnorm(xxsave(1:3,i)),'o','Color','red');
    
    subplot('Position',[0.65 0.15 .3 .3])
    plot(tsave,action(:,1),'Color','red')
    plot(tsave,action(:,2),'Color','green')
    plot(tsave,action(:,3),'Color','blue')
    plot(tsave,action(:,4),'Color','yello')
    dot4=plot(tsave(i),action(i,1),'o','Color','red');
    dot5=plot(tsave(i),action(i,2),'o','Color','red');
    dot6=plot(tsave(i),action(i,3),'o','Color','red');
    dot7=plot(tsave(i),action(i,4),'o','Color','red');
    F = getframe(fig);
    writeVideo(v,F)
    delete(dot1);delete(dot4);delete(dot5);delete(dot6);delete(dot7);
    
    delete(findall(gcf,'type','annotation'))
end
hold off
close(v);
disp('Video generated.')
end

