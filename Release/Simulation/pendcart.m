function dx = pendcart(x,m,M,L,g,d,u,wr)

% I = 4/3*m*L^2;

D = m*L*L*(M+m*(1-cos(x(3))^2));
dx(1,1) = x(2);
dx(2,1) = (1/D)*(m^2*L^2*g*cos(x(3))*sin(x(3)) + m*L^2*(m*L*x(4)^2*sin(x(3)) - d*x(2))) + m*L*L*(1/D)*u;
dx(3,1) = x(4);
dx(4,1) = (1/D)*(-(m+M)*m*g*L*sin(x(3)) - m*L*cos(x(3))*(m*L*x(4)^2*sin(x(3)) - d*x(2))) - m*L*cos(x(3))*(1/D)*u;

% dx(1,1) = x(2);
% dx(2,1) = (L^2*g*m^2*sin(2*x(3))/2 + (I + L^2*m)*(L*m*x(4)^2*sin(x(3)) - d*x(2) + u))/(-L^2*m^2*cos(x(3))^2 + (I + L^2*m)*(M + m));
% dx(3,1) = x(4);
% dx(4,1) = -m*L/(I + m*L^2) * (dx(2,1)*cos(x(3)) + g*sin(x(3)));
% integral of theta error
if nargin == 8
    dx(5,1) = x(3) - wr(3);
end