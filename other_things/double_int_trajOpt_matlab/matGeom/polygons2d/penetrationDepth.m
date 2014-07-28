function [dist contact_pts] = penetrationDepth(polygon1, polygon2)
%SIGNEDDISTANCEPOLYGONS Compute the penetration depth between 2 polygons
%   DIST = penetrationDepth(POLY1, POLY2)
%   Returns the signed distance between 2 polygons. The two polygons are
%   assumed to be intersecting each other, so the returned distance is
%   negative by definition.
%
%   [DIST, POINTS] = penetrationDepth(POLY1, POLY2)
%   Also returns the 2 points involved with the distance. The
%   first point belongs to POLY1 and the second point belongs to POLY2.
%
%   Example
%   penetrationDepth
%
%   See also
%   signedDistancePolygons, distancePolygons
%
%
% ------
% Author: Alex Lee


% Separation of axis algorithm: http://physics2d.com/content/separation-axis

if sum(polygon1(end, :)==polygon1(1,:))~=2
    polygon1 = [polygon1; polygon1(1,:)];
end
if sum(polygon2(end, :)==polygon2(1,:))~=2
    polygon2 = [polygon2; polygon2(1,:)];
end

global_contact_infos = zeros(0,5);
for i=1:2
    if i==1
        poly1 = polygon1;
        poly2 = polygon2;
    else
        poly1 = polygon2;
        poly2 = polygon1;
    end
    
    % number of faces
    Nf = size(poly1,1)-1;

    vertices1 = poly1(1:end-1,:);
    vertices2 = poly2(1:end-1,:);
    
    % number of vertices in poly1 and poly2
    Nv1 = size(vertices1,1);
    Nv2 = size(vertices2,1);

    contact_infos = zeros(Nf, 5);
    for f = 1:Nf
        face = createLine(poly1(f,:), poly1(f+1,:));

        sep_axis = orthogonalLine(face, [0 0]);
        sep_axis_edge = createEdge(sep_axis, 1);

        pts = projPointOnLine([vertices1; vertices2], sep_axis);
        pos = edgePosition(pts, sep_axis_edge);

        pos1 = pos(1:Nv1);
        pos2 = pos(Nv1+1:end);

        [min_pos2 argmin_pos2] = min(pos2);
        [max_pos2 argmax_pos2] = max(pos2);
        [min_pos1 argmin_pos1] = min(pos1);
        [max_pos1 argmax_pos1] = max(pos1);
        diffs = [ min_pos2-max_pos1; min_pos1-max_pos2 ];
        arg_diffs = [ argmax_pos1 argmin_pos2; argmin_pos1 argmax_pos2 ];
        if diffs(1)>=0 && diffs(2)>=0
            [dist arg_dist] = min(diffs);
        else
            [dist arg_dist] = max(diffs);
        end
        arg_face = arg_diffs(arg_dist,1);
        arg_vertex = arg_diffs(arg_dist,2);

        contact_vertex = poly2(arg_vertex,:);
        contact_point = contact_vertex + (pos1(arg_face)-pos2(arg_vertex))*sep_axis_edge(1,3:4);
        contact_infos(f,:) = [dist contact_vertex contact_point];
    end
    global_contact_infos(end+1:end+size(contact_infos,1),:) = contact_infos;
end

[dist arg_dist] = max(global_contact_infos(:,1));
if arg_dist < size(polygon1,1) % size of polygon is the size of vertices plus 1
    contact_pts = [global_contact_infos(arg_dist,4:5); global_contact_infos(arg_dist,2:3)];
else
    contact_pts = [global_contact_infos(arg_dist,2:3); global_contact_infos(arg_dist,4:5)];
end
