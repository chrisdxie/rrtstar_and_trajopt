function plot_traj(make_robot_poly, obstacles, traj)
clf;
hold on;
axis equal;

for obstacle = obstacles
    drawPolygon(obstacle{:},'b');
end

h2 = [];
for t=1:size(traj,2)
    x = traj(:,t);
    robot_poly = make_robot_poly(x);
    h2(end+1) = drawPolygon(robot_poly, 'r');
end



for t=1:size(traj,2)
    h1=[];
    x = traj(:,t);
    robot_poly = make_robot_poly(x);
    h1(end+1) = drawPolygon(robot_poly, 'k','LineWidth',2);
    % draw distances to obstacles
    for obstacle = obstacles
        [dist, contact_pts] = signedDistancePolygons(robot_poly, obstacle{:});
        edge = createEdge(contact_pts(1,:), contact_pts(2,:));
        if dist < 0, color='r'; else color='g'; end;
        h1(end+1) = drawEdge(edge, color, 'LineWidth', 3);
    end
    pause(.02);
    delete(h1);
end

end


% function startVisualizer(self)
%     self.h_fig = figure;
%     hold on;
%     set(self.h_fig,'KeyPressFcn', @self.controlCallback);
%     axis equal;
% end
% function controlCallback(self, h_obj, evt)
%     if (strcmp(evt.Key, 'q')) 
%         close(findobj('type','figure','name',self.h_fig));
%         self.last_u = [];
%     elseif (strcmp(evt.Key, 'p')) 
%         self.paused = ~self.paused;
%     end
% end
