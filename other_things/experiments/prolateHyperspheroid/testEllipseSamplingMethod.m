function testEllipseSamplingMethod(x_start, x_goal, c_best, c_min)

    % Testing method from this paper (Informed RRT*):
    % http://arxiv.org/pdf/1404.2334v2.pdf
    
    % 2D
    d = 4;
    
    x_c = (x_start + x_goal)/2;
    
    L = zeros(d,d);
    for i=1:d
       if i == 1
           L(i,i) = c_best/2;
       else
           L(i,i) = sqrt(c_best^2 - c_min^2)/2;
       end
    end
    
    a1 = (x_goal - x_start)/norm(x_goal - x_start);
    M = zeros(d, d);
    M(:,1) = a1;
    
    [U,S,V] = svd(M);
    
    for i=1:d
       if i == d
           W(i,i) = det(U)*det(V);
       else
           W(i,i) = 1;
       end
    end

    C = U*W*V';
    
    drawEllipseContour((C*L*L'*C')^-1, x_c);
    
    for i=1:1000
        s = sampleUnitBall(d);
        x = C*L*s + x_c;
        plot(x(1),x(2),'.r');
    end


end