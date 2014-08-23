function signed_distance_test ()

    % Random triangles in [0,1] x [0,1]
    t1 = rand(3, 2); t1 = [t1; t1(1,:)];
    t2 = rand(3, 2); t2 = [t2; t2(1,:)];
    
    t1 = [.5 .48; .5 .52];
    %t2 = [.45 .6;.7 .6;.7 .3; .45 .3; .45 .6 ];
    
    [d pts] = signedDistancePolygons(t1, t2);
    pt1 = pts(1,:);
    pt2 = pts(2,:);
    
    hold on;
    plot(t1(:,1)', t1(:,2)');
    plot(t2(:,1)', t2(:,2)');

    if d < 0
        plot([pt1(1) pt2(1)],[pt1(2) pt2(2)], 'r');
    else
        plot([pt1(1) pt2(1)],[pt1(2) pt2(2)], 'g');
    end
    
    disp(['Dist: ' num2str(d)]);

end