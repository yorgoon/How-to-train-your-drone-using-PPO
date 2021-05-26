function video_gen(fig, tsave, xsave, filename, target_fps, Fmat)

fv = stlread('quadrotor_model/quadrotor_base.stl');
grid on
hold on
Q = FramePlot(tsave(1), xsave(1,:), Fmat(:,1), fv);
set(gcf,'Renderer','OpenGL')
v = VideoWriter(filename);
v.FrameRate = target_fps;
open(v)

given_fps = 1/(tsave(2)-tsave(1));
skip_interval = floor(given_fps/target_fps);
for i=2:skip_interval:length(xsave(:,1))
    Q.UpdatePlot(tsave(i), xsave(i,:), Fmat(:,i))
    annotation('textbox', [0.8, 0.3, 0, 0], 'string', ['elapsed time: ', num2str(tsave(i))])
    F = getframe(fig);
    writeVideo(v,F)
    delete(findall(gcf,'type','annotation'))
end
hold off
close(v);
end

