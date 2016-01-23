
function [xk, uk] = simulateConformingFunnel(bc, stateOffset)


Nsteps = 120;
deltat = 0.1;

% Compute the polynomial approximation of the drake plant
[xp,tp] = double(bc.x0);
x0pp = PPTrajectory(foh(tp,xp));
x0pp = setOutputFrame(x0pp,bc.sys.drakeplant.getStateFrame);
[poly] = bc.sys.getSystemPoly(x0pp, bc.c, bc.V0);

% initial states/inputs
xk = zeros(bc.sys.params.n,Nsteps);
uk = zeros(bc.sys.params.m,Nsteps);

x = double(bc.x0,5.2) + stateOffset;

for k = 1:Nsteps
    k
    u = bc.execute(x,poly);
    x = bc.sys.simulate(x,u,deltat);
    
    xk(:,k) = x;
    uk(:,k) = u;
end

figure(10)
clf
plot(bc.x0,[],10)
hold on
plot(xk(1,:),xk(2,:),'r')
