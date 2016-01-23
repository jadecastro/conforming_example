function [Xdot] = unicycle(t,X,U)

x = X(1); y = X(2); th = X(3);
uV = U(1);  uW = U(2);

% Governing equations
xdot = uV*cos(th);
ydot = uV*sin(th);
thdot = uW;

Xdot = [xdot; ydot; thdot];
