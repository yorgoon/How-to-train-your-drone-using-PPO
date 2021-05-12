classdef FramePlot2 < handle
    
    properties (SetAccess = public)
        t;
        state;
        S; % parameters;
        k = 0;
        R;  % SO(3)
        d;  % translation
        g;  % SE(3)
        XData0; % x coords of meshes of the quadrotor model
        YData0; % y coords of meshes of the quadrotor model
        ZData0; % z coords of meshes of the quadrotor model
        Fmat;   % External force
        state_hist;       % position history
        state_des_hist;   % desired position history
        time_hist;        % time history
        stl_handle;       % STL file handle
    end
    properties (SetAccess = private)
        % Figure handles 
        h_3d;
        h_frame_x;
        h_frame_y;
        h_frame_z;
        h_pos_hist;
        h_patch_stl;
        h_ext_F;
        h_link1;
        h_link2;
        h_ee;
        h_b0b;
        h_b;
        h_joint2;
    end
    
    methods
        function Q = FramePlot2(t, state, Fmat, stl_handle)
            Q.state = state;
%             Q.S = S;
            Q.R = ROTZ(Q.state(6))*ROTY(Q.state(5))*ROTZ(Q.state(4));
            Q.d = Q.state(1:3);
            Q.g = [Q.R Q.d';0 0 0 1];
            Q.stl_handle = stl_handle;
            Q.state_hist = zeros(6, 100);
            x_axis = Q.R(:,1);
            y_axis = Q.R(:,2);
            z_axis = Q.R(:,3);
            pt = Q.state(1:3);
            Q.t = t;
            Q.Fmat = Fmat;
            Eb0b = [eye(3),[0 0 -.1]';0 0 0 1];
%             Eb1 = Eb0b*[  cos(state(7)), 0, sin(state(7)),  (Q.S.l1*cos(state(7)));
%                     0, 1,       0,               0;
%              -sin(state(7)), 0, cos(state(7)), -(Q.S.l1*sin(state(7)));
%                     0, 0,       0,               1];
%             Eb2 = Eb0b*[  cos(state(7) - state(8)), 0, sin(state(7) - state(8)),   Q.S.l1*cos(state(7)) + (Q.S.l2*cos(state(7) - state(8)));
%                                 0, 1,            0,                                  0;
%                      -sin(state(7) - state(8)), 0, cos(state(7) - state(8)), - Q.S.l1*sin(state(7)) - (Q.S.l2*sin(state(7) - state(8)));
%                                  0, 0,            0,                                  1];           
%             Ef = [eye(3),[10 -0.9 0]';0 0 0 1];
            Eb = Q.g*Eb0b;
%             E1 = Q.g*Eb1;
%             E2 = Q.g*Eb2;
%             
%             p1 = E1(1:3,4);
%             p2 = E2(1:3,4);
            
            % initialization
            h_3d = gca;
            Q.h_3d = h_3d;
            hold(Q.h_3d, 'on')
            % Actual position history
            Q.h_pos_hist = plot3(Q.h_3d, Q.state(1), Q.state(2), Q.state(3), '.k', 'MarkerSize', 5);
            % Body frame
            Q.h_frame_x = plot3(Q.h_3d, [pt(1) pt(1) + x_axis(1)/2], ...
                                        [pt(2) x_axis(2)/2+pt(2)], ...
                                        [pt(3) x_axis(3)/2+pt(3)], 'r', 'LineWidth', 2);
            Q.h_frame_y = plot3(Q.h_3d, [pt(1) y_axis(1)/2+pt(1)], ...
                                        [pt(2) y_axis(2)/2+pt(2)], ...
                                        [pt(3) y_axis(3)/2+pt(3)], 'g', 'LineWidth', 2);
            Q.h_frame_z = plot3(Q.h_3d, [pt(1) z_axis(1)/2+pt(1)], ...
                                        [pt(2) z_axis(2)/2+pt(2)], ...
                                        [pt(3) z_axis(3)/2+pt(3)], 'b', 'LineWidth', 2);
            Q.h_b0b = plot3(Q.h_3d, [pt(1) Eb(1,4)], ...
                                    [pt(2) Eb(2,4)], ...
                                    [pt(3) Eb(3,4)], ...
                                    'Color', [0.25, 0.25, 0.25], 'LineWidth', 5);
%             % Link 1
%             Q.h_link1 = plot3(Q.h_3d, [Eb(1,4) p1(1)], ...
%                                       [Eb(2,4) p1(2)], ...
%                                       [Eb(3,4) p1(3)], 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 3);
%             Q.h_joint2 = plot3(Q.h_3d, p1(1),p1(2),p1(3),'.r','MarkerSize',30);
%             % Link 2
%             Q.h_link2 = plot3(Q.h_3d, [p1(1) p2(1)], ...
%                                       [p1(2) p2(2)], ...
%                                       [p1(3) p2(3)], 'Color', [0.75, 0, 0.75], 'LineWidth', 3);
%             Q.h_b = plot3(Q.h_3d, Eb(1,4),Eb(2,4),Eb(3,4),'.r','MarkerSize',30);
%             % End-effector
%             Q.h_ee = plot3(Q.h_3d, p2(1),p2(2),p2(3),'.r','MarkerSize',30);
            
            % external force
            Q.h_ext_F = plot3(Q.h_3d, [pt(1) Q.Fmat(1)/3+pt(1)], ...
                                      [pt(2) Q.Fmat(2)/3+pt(2)], ...
                                      [pt(3) Q.Fmat(3)/3+pt(3)], 'Color',[0.9290, 0.6940, 0.1250], 'LineWidth', 2.0);
            
            % Quadrotor model
            Q.h_patch_stl = patch(Q.stl_handle);
            % Model visual parameters
            Q.h_patch_stl.FaceColor = [0.25 0.25 0.25];
%             Q.h_patch_stl.EdgeColor = 'none';
            Q.h_patch_stl.FaceLighting = 'gouraud';
            Q.h_patch_stl.AmbientStrength = 0.15;
            axis('image')
            Q.XData0 = Q.h_patch_stl.XData;
            Q.YData0 = Q.h_patch_stl.YData;
            Q.ZData0 = Q.h_patch_stl.ZData;
            
            % transform the model to the starting point
            new_XData = zeros(size(Q.h_patch_stl.XData));
            new_YData = zeros(size(Q.h_patch_stl.YData));
            new_ZData = zeros(size(Q.h_patch_stl.ZData));
            
            for i=1:length(Q.h_patch_stl.XData(:))
                homo_coord = [Q.XData0(i) Q.YData0(i) Q.ZData0(i) 1]';
                transformed_homo_coord = Q.g*homo_coord;
                new_XData(i) = transformed_homo_coord(1);
                new_YData(i) = transformed_homo_coord(2);
                new_ZData(i) = transformed_homo_coord(3);
            end
            set(Q.h_patch_stl, 'XData', new_XData, 'YData', new_YData, 'ZData', new_ZData);
            hold(Q.h_3d, 'off')
            drawnow
        end
        % Update state
        function UpdateQuadState(Q, t, state, Fmat)
            Q.state = state;
            Q.R = ROTZ(Q.state(6))*ROTY(Q.state(5))*ROTZ(Q.state(4));
            Q.d = Q.state(1:3);
            Q.g = [Q.R Q.d';0 0 0 1];
            Q.t = t;
            Q.Fmat = Fmat;
        end
        % Update history
        function UpdateQuadHist(Q)
            Q.k = Q.k + 1;
            Q.state_hist(:,Q.k) = Q.state(1:6);
        end      
      
        function UpdatePlot(Q, t, state, Fmat)
            Q.UpdateQuadState(t, state, Fmat);
            Q.UpdateQuadHist();
            
            x_axis = Q.R(:,1);
            y_axis = Q.R(:,2);
            z_axis = Q.R(:,3);
            pt = Q.state(1:3);
            Eb0b = [eye(3),[0 0 -.1]';0 0 0 1];
            
%             Eb1 = [  cos(state(7)), 0, sin(state(7)),  (Q.S.l1*cos(state(7)));
%                     0, 1,       0,               0;
%              -sin(state(7)), 0, cos(state(7)), -(Q.S.l1*sin(state(7)));
%                     0, 0,       0,               1];
%             Eb2 = [  cos(state(7) - state(8)), 0, sin(state(7) - state(8)),   Q.S.l1*cos(state(7)) + (Q.S.l2*cos(state(7) - state(8)));
%                                 0, 1,            0,                                  0;
%                      -sin(state(7) - state(8)), 0, cos(state(7) - state(8)), - Q.S.l1*sin(state(7)) - (Q.S.l2*sin(state(7) - state(8)));
%                                  0, 0,            0,                                  1];
            Eb = Q.g*Eb0b;
%             E1 = Q.g*Eb1;
%             E2 = Q.g*Eb2;
%             
%             p1 = E1(1:3,4);
%             p2 = E2(1:3,4);
            
            set(Q.h_pos_hist, ...
            'XData', Q.state_hist(1,1:Q.k), ...
            'YData', Q.state_hist(2,1:Q.k), ...
            'ZData', Q.state_hist(3,1:Q.k));

            set(Q.h_frame_x, ...
            'XData', [pt(1) x_axis(1)/2+pt(1)], ...
            'YData', [pt(2) x_axis(2)/2+pt(2)], ...
            'ZData', [pt(3) x_axis(3)/2+pt(3)]);
            set(Q.h_frame_y, ...
            'XData', [pt(1) y_axis(1)/2+pt(1)], ...
            'YData', [pt(2) y_axis(2)/2+pt(2)], ...
            'ZData', [pt(3) y_axis(3)/2+pt(3)]);
            set(Q.h_frame_z, ...
            'XData', [pt(1) z_axis(1)/2+pt(1)], ...
            'YData', [pt(2) z_axis(2)/2+pt(2)], ...
            'ZData', [pt(3) z_axis(3)/2+pt(3)]);
            set(Q.h_b0b,...
            'XData', [pt(1) Eb(1,4)], ...
            'YData', [pt(2) Eb(2,4)], ...
            'ZData', [pt(3) Eb(3,4)]);
%             set(Q.h_link1, ...
%             'XData', [Eb(1,4) p1(1)], ...
%             'YData', [Eb(2,4) p1(2)], ...
%             'ZData', [Eb(3,4) p1(3)]);
%             
%             set(Q.h_joint2,'XData',p1(1),'YData',p1(2),'ZData',p1(3));
%             
%             set(Q.h_link2, ...  
%             'XData', [p1(1) p2(1)], ...
%             'YData', [p1(2) p2(2)], ...
%             'ZData', [p1(3) p2(3)]);
%             set(Q.h_b,'XData',Eb(1,4),'YData',Eb(2,4),'ZData',Eb(3,4));
%             set(Q.h_ee,'XData',p2(1),'YData',p2(2),'ZData',p2(3));
            
            set(Q.h_ext_F, ...
            'XData', [pt(1) Q.Fmat(1)/3+pt(1)], ...
            'YData', [pt(2) Q.Fmat(2)/3+pt(2)], ...
            'ZData', [pt(3) Q.Fmat(3)/3+pt(3)]);        
            
            
            
            new_XData = zeros(size(Q.h_patch_stl.XData));
            new_YData = zeros(size(Q.h_patch_stl.YData));
            new_ZData = zeros(size(Q.h_patch_stl.ZData));
            
            for i=1:length(Q.h_patch_stl.XData(:))
                homo_coord = [Q.XData0(i) Q.YData0(i) Q.ZData0(i) 1]';
                transformed_homo_coord = Q.g*homo_coord;
                new_XData(i) = transformed_homo_coord(1);
                new_YData(i) = transformed_homo_coord(2);
                new_ZData(i) = transformed_homo_coord(3);
            end
            set(Q.h_patch_stl, 'XData', new_XData, 'YData', new_YData, 'ZData', new_ZData);
        end
    end
   
end