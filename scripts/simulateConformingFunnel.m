
function [xk, uk] = simulateConformingFunnel(bc, stateOffset, timeOffset)
% simulateConformingFunnel: Executes a closed-loop simulation of the system
% composed with the found atomic controller.
%
%   [xk, uk] = simulateConformingFunnel(bc, stateTimeOffset)
%
% Inputs:
%
%   bc (AtomicController object) : If necessary, a barrier function atomic
%   controller object for the given trajectory.
%
%   stateOffset (n-by-1 double array) : Specifies the offset from the
%   nominal state bc.x0 at the specified timeOffset time (t = timeOffset). 
%
%   timeOffset (double) : Specifies the time offset from 0 at which to
%   start the simulation.
%
% Outputs:
%
%   xk (n-by-Nsteps double array) : An array of values representing the
%   closed-loop system's trajectory.
%
%   uk (m-by-Nsteps double array) : An array of values representing the
%   closed-loop system's control inputs.
%
%

Nsteps = 120;  % Specify the number of steps for the simulation

deltat = 0.1;  % Specify the simulator time increment


% Compute the polynomial approximation of the drake plant
[xp,tp] = double(bc.x0);
x0pp = PPTrajectory(foh(tp,xp));
x0pp = setOutputFrame(x0pp,bc.sys.drakeplant.getStateFrame);
[poly] = bc.sys.getSystemPoly(x0pp, bc.c, bc.V0);

% Specify the initial states/inputs
xk = zeros(bc.sys.params.n,Nsteps);
uk = zeros(bc.sys.params.m,Nsteps);

x = double(bc.x0,timeOffset) + stateOffset;

for k = 1:Nsteps
    disp(['Timestep: ',num2str(k*deltat)]);
    
    % Get the next control input
    u = bc.execute(x,poly);
    
    % Simulate the plant
    x = bc.sys.simulate(x,u,deltat);
    
    % Store the data
    xk(:,k) = x;
    uk(:,k) = u;
end

% Plot the result!
figure(10)
clf
plot(bc.x0,[],10)
hold on
plot(xk(1,:),xk(2,:),'r')
title(['Simulation results: conforming funnel, offset by ',num2str(stateOffset(1)),', ',num2str(stateOffset(2)),', ',num2str(stateOffset(3)),' at time offset ',num2str(timeOffset)])
xlabel('x [m]'), ylabel('y [m]')
