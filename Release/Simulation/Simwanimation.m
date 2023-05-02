clear; close all; clc

m = 0.36;
M = 2.4;
L = 0.5;
g = 10;
d = 1;
I = 4/3*m*L^2;

xf = [0; 0; pi; 0];  % fixed point to linearize around
uf = 0;

A = [0 1 0 0;
    0 d*(I + L^2*m)/(L^2*m^2*cos(xf(3))^2 - (I + L^2*m)*(M + m)) -L*m*(L*m*(L^2*g*m^2*sin(2*xf(3)) + 2*(I + L^2*m)*(L*m*xf(4)^2*sin(xf(3)) - d*xf(2) + uf))*sin(xf(3))*cos(xf(3)) + (L^2*m^2*cos(xf(3))^2 - (I + L^2*m)*(M + m))*(L*g*m*cos(2*xf(3)) + xf(4)^2*(I + L^2*m)*cos(xf(3))))/(L^2*m^2*cos(xf(3))^2 - (I + L^2*m)*(M + m))^2 -2*L*m*xf(4)*(I + L^2*m)*sin(xf(3))/(L^2*m^2*cos(xf(3))^2 - (I + L^2*m)*(M + m));
    0 0 0 1;
    0 L*d*m*cos(xf(3))/(I*M + I*m + L^2*M*m + L^2*m^2*sin(xf(3))^2) L*m*(L^2*m^2*(L*m*xf(4)^2*sin(2*xf(3)) + 2*M*g*sin(xf(3)) - 2*d*xf(2)*cos(xf(3)) + 2*g*m*sin(xf(3)) + 2*uf*cos(xf(3)))*sin(xf(3))*cos(xf(3)) - (I*M + I*m + L^2*M*m + L^2*m^2*sin(xf(3))^2)*(L*m*xf(4)^2*cos(2*xf(3)) + M*g*cos(xf(3)) + d*xf(2)*sin(xf(3)) + g*m*cos(xf(3)) - uf*sin(xf(3))))/(I*M + I*m + L^2*M*m + L^2*m^2*sin(xf(3))^2)^2 -L^2*m^2*xf(4)*sin(2*xf(3))/(I*M + I*m + L^2*M*m + L^2*m^2*sin(xf(3))^2)];
B = [0; -(I + L^2*m)/(L^2*m^2*cos(xf(3))^2 - (I + L^2*m)*(M + m)); 0; -L*m*cos(xf(3))/(I*M + I*m + L^2*M*m + L^2*m^2*sin(xf(3))^2)];

% A = [0              1 0 0;
%      (M+m)*g/(M* L) 0 0 0;
%      0              0 0 1;
%      (-m*g)/M       0 0 0];
% B = [0;
%      -1/(M*L);
%      0;
%      1/M];
% C = [1 0 0 0;
%      0 0 1 0];
% D = 0;


eig(A)
rank(ctrb(A,B))

%%  Design LQR controller
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
R = 1e-3;
% Q = [1 0 0   0;
%      0 1 0   0;
%      0 0 500 0;
%      0 0 0   250];
% R = 1;

% Q = [1 0 0;
%     0 1 0;
%     0 0 100];
% R = 1e-3;

K = lqr(A,B,Q,R);

%% Simulate closed-loop system
tspan = 0:.01:10;
x0 = [-1; 0; pi-1; 0];  % initial condition
wr = [1; 0; pi-0.35; 0];      % reference position
% wrapAngle=@(x)[x(1); x(2); wrapTo2Pi(x(3)); x(4)];
u=@(x)-K*(x-wr);       % control law
[t,x] = ode45(@(t,x)pendcart(x,m,M,L,g,d,u(x)),tspan,x0);

for k=1:10:length(t)
    drawpend(x(k,:),m,M,L);
end

%%
figure(1);
plot(t,x,'LineWidth',2); hold on
l1 = legend('x','v','\theta','\omega');
% l1 = legend('x','v','\theta','\omega','i');
set(l1,'Location','SouthEast')
set(gcf,'Position',[100 100 500 200])
xlabel('Time')
ylabel('State')
grid on
set(gcf,'PaperPositionMode','auto')
% print('-depsc2', '-loose', 'figures/FIG_02_LQR');
figure(2);
plot(t, (-K*(x.' - wr)).', 'LineWidth', 2); hold on
l2 = legend('u');
set(l2,'Location','SouthEast')
grid on
set(gcf,'PaperPositionMode','auto')