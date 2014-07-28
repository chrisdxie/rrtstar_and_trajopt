function obstacles = add_obstacle(obstacles)
clf;
axis([-2,2,-2,2]);
hold on;
for obs_cell = obstacles
    obs = obs_cell{1};
    plot([obs(:,1)',obs(1,1)],[obs(:,2)',obs(1,2)]);
end
[x,y] = ginput;
obstacles{end+1} = [x,y];
plot([x',x(1)], [y',y(1)]);
end