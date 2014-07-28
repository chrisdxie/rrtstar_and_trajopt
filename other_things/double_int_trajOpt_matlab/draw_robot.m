function draw_robot(robot)

    draw_distances = true;

    if isempty(findobj('type','figure','name',robot.h_fig))
        robot.startVisualizer();
        robot.h_state = [];
    end
    delete(robot.h_state);
    robot.h_state = [];
    
    if draw_distances
        % draw convex hull of the robot
        if robot.dimension == 2
            robot.h_state(end+1) = drawPolygon(robot.toRobotGeom(robot.getState()), 'r');
        else
            h = drawMesh(robot.toRobotGeom(robot.getState()), 'r');
            robot.h_state(end+1:end+length(h)) = h;
            alpha(h, 0.1);
        end
        % draw distances to obstacles
        for obstacle = robot.obstacles
            [dist, contact_pts] = robot.signedDistance(robot.getState(), obstacle{:});
            if robot.dimension == 2
                edge = createEdge(contact_pts(1,:), contact_pts(2,:));
            else
                edge = [contact_pts(1,:) contact_pts(2,:)];
            end
            color = 'g';
            if dist<0, color='r'; end
            robot.h_state(end+1) = drawEdge(edge, color, 'LineWidth', 3);
        end
    end