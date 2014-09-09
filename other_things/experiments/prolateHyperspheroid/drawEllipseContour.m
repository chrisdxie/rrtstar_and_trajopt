function drawEllipseContour(Q, c)

    figure(1);
    hold on;

    assert(all(size(Q) == [2 2]));

    [x,y] = meshgrid(-20:0.1:20);
    
    % Center
    x = x-c(1);
    y = y-c(2);
     
    f = Q(1,1)*x.^2 + Q(1,2)*x.*y + Q(2,1)*x.*y + Q(2,2)*y.^2;
    
    % Uncenter for plotting purposes
    x = x+c(1);
    y = y+c(2);
    
    contour(x,y,f,[1 1],'b');
    
    % Sample from unit ball just to check my implementation
    %{
    for i=1:1000
        s = sampleUnitBall;
        plot(s(1),s(2),'.r');
    end
    %}
    
end