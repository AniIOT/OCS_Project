function drawpend(state,m,M,L)
x = state(1);
th = state(3);

% dimensions
W = 1*sqrt(M/5);  % cart width
H = .1*sqrt(M/5); % cart height
wr = .3;          % wheel radius
mr = .3*sqrt(m);  % mass radius

% positions
y = wr/2+H/2; % cart vertical position
pendxu = x + L*sin(th) - W/2 -mr/4;
pendyu = y - L*cos(th) + 0.4;
pendxl = x + L*sin(th)/2 - W/2 - mr/4;
pendyl = y - L*cos(th)/2 + 0.2;

plot([-10 10],[0 0],'k','LineWidth',2), hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x-wr/2,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
% rectangle('Position',[x+.9*W/2-wr,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
plot([x-W/2 pendxu],[y pendyu],'k','LineWidth',2); % Draw pendulum
plot([x+W/2 pendxu+W],[y pendyu],'k','LineWidth',2); % Draw pendulum
rectangle('Position',[pendxl,pendyl,W,H],'Curvature',.1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
rectangle('Position',[pendxu,pendyu,W,H],'Curvature',.1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

axis([-7 7 -2 2]); axis equal
set(gcf,'Position',[100 100 1000 400])
set(gca,'xtick', -10:1:10)
drawnow, hold off