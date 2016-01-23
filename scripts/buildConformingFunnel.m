
function [ac, bc] = buildConformingFunnel(reg, aut, ac_trans, x0, u0, options)
% buildConformingFunnel: Computes a conforming funnel based on region
% geometry information, a set of funnels that are required for composition,
% given a trajectory, and system model description. Repartitions the
% workspace if necessary.
%
%   [ac, bc] = buildConformingFunnel(reg, aut, ac_trans, x0, u0, options)
%
% Inputs:
%
%   reg (Region object) : Contains all data relating to the geometry of the
%   problem domain.  
%
%   aut (struct) : Structure containing the states and transitions
%   whose numbers correspond to the ordered list of regions in the object
%   reg. 
%
%   ac_trans (1-by-N_trans cell array of AtomicController objects) :
%   Specifies the atomic controllers for each transition in the state
%   machine that have been constructed so far.  The "itrans" parameter must
%   be correspond to a non-empty entry in ac_trans upon which we will be
%   composing the required conforming funnel.  Each AtomicController 
%   consists of a trajectory, feedback controller, system model, and
%   funnel.
%
%   x0 (Traject object) : Specifies the state trajectory for which we will
%   be designing a feedback and about which we will need to verify
%   invariance (i.e. conforming funnel)
%
%   u0 (Traject object) : Specifies the nominal control input trajectory
%   that achieves the state trajectory given the specified system dynamics.
%
%   options (struct) : Contains various options; the pertinent ones being:
%
%         d (double) : The order of the barrier function to be computed
%         (default: 6)
%
%         solver (fcn handle) : specifies the optimization engine for
%         computing funnels (default: @spot_mosek) 
%
%         solver_name (string) : string name for the optimization engine
%         (default: 'mosek') 
%
%         isMaximization (logical) : specify whether the objective is to
%         maximize the size of the funnel or minimize it (default: true)  
%
% Outputs:
%
%   ac (AtomicController object) : Quadratically-parameterized atomic
%   controller objects are returned that parameterize the desired inward
%   funnel for the given trajectory.
%
%   bc (AtomicController object) : If necessary, a barrier function atomic
%   controller object for the given trajectory.
% 
% 

itrans = 5; % Specify which transition atomic controller to compose with an inward funnel

R = 0.01;  % Manually adjust the weighting for TVLQR feedback controller design

rhof = 0.1;   % Specify the final level set parameter, rho, for the

% Specify the index of an transition funnel at which to attempt to compose
% its cross-section with an inward-facing funnel whose final set lies in
% the intersection of all transition funnels outgoing from this region
indexToCompose = 50;  

%% Process the data structures
% Parse the transitions, and extract the correct FSM state
trans = vertcat(aut.trans{:});
idxMode = trans(itrans,1);
regMode = reg(aut.q{idxMode});

% Extract the atomic controller and system model for this transition
acNext = [ac_trans{itrans}];
tmp = acNext.ellipsoid;
ellToCompose = tmp(indexToCompose);

acNext.sys.params.ctrloptions.R = R;
sys = acNext.sys;

% Initialize our object arrays
ac = [];
bc = [];

% Plot various things
figure(90), clf, hold on, axis equal
plot(reg)
plot(acNext,sys,90)
title('Result: conforming funnel')
xlabel('x [m]'), ylabel('y [m]')

figure(5), clf, hold on, axis equal
plot(reg)
title('Partial result: unconstrained quadratic funnel')
xlabel('x [m]'), ylabel('y [m]')


%% Compute the inward atomic controller and region of invariance
% Perform the quadratic funnel computations
[ac, c] = computeAtomicController(u0,x0,sys,regMode,ellToCompose,options,rhof);

plot(ac.x0,'k',5)
rhoi = double(ac.rho,0);
[res, isectIdx, isectArray] = isinside(ac,reg(aut.q{trans(itrans,1)}),sys);

if ~isempty(isectIdx)
    disp(['the funnel intersects the boundary at time index: ',num2str(min(isectIdx))]);
end

% If the funnel computation was unsuccessful, compute a conforming funnel
if ~res
    % NB: the following assumes only one contiguous interval where the funnel left the region.
    % Here, we split the funnel into three parts:
    %   - one from the start to the time of the first intersection (acPre)
    %   - another during the interval of intersection (this is created using CBFs)
    %   - another following the intersection (acPost)
    t = ac.x0.getTimeVec();
    acPre = [];
    
    t1 = t(max(isectIdx)+1:end);
    x1 = Traject(t1,double(ac.x0,t1));
    u1 = Traject(t1,double(ac.u0,t1));
    K1 = Traject(t1,double(ac.K,t1));
    P1 = Traject(t1,double(ac.P,t1));
    rho1 = Traject(t1,double(ac.rho,t1));
    Vquad = ac.V(max(isectIdx)+1:end);
    acPost = QuadraticAC(x1,u1,K1,P1,rho1,Vquad,sys);
    
    if false %min(isectIdx) > 1  
        t0 = t(1:min(isectIdx)-1);
        x0 = Traject(t0,double(ac.x0,t0));
        u0 = Traject(t0,double(ac.u0,t0));
        
        % Compute a new atomic controller that minimizes the funnel (in contrast to the original method that maximizes it). 
        % The minimization is used here to find a suitable initial condition for the CBF.
        options.isMaximization = false;
        [acPre] = computeAtomicController(u0,x0,sys,regMode,ellToCompose,options,rhoi);
        
    end
    
    % Lastly, attempt to construct the barrier function atomic controllers
    t01 = t(min(isectIdx):max(isectIdx));
    x01 = Traject(t01,double(ac.x0,t01));
    u01 = Traject(t01,double(ac.u0,t01));                                        
    
    regMode = reg(aut.q{idxMode});
    
    % Define the initial set as the end of the prefix funnel
    if ~isempty(acPre)
        tmp = acPre.ellipsoid;
        ellToCompose = tmp(end);
    else
        tmp = acNext.ellipsoid;
        ellToCompose = tmp(indexToCompose);
    end
    
    % Call the conforming funnels computations
    [bc] = computeConformingFunnel(u0,x0,u01,x01,sys,ac,regMode,ellToCompose,options);
    
    % Plot the result
    figure(90)
    plot(reg(1),'r')
    plot(reg(2),'g')
    
    bc.plot(ac.x0,90)

    plot(acNext.x0,'k',90)
    plot(ac.x0,'k',90)
    
    % Store the data
    ac = acPre;
    ac = [ac; acPost];
    
end

%% Parse the result and construct a new region 
% If the funnel computation succeeded, then create a new region that
% repartitions the existing workspace.

% Create a new region based on the last unverified index of the next funnel.
funStepSize = 100;
idxLast = indexToCompose + funStepSize;

% Create the new region
newRegVert = [];
ellAcNext = projection(acNext,sys);
for j = length(acNext.x0.getTimeVec()):-funStepSize:idxLast
    newRegVert = [buildNewRegion(ellAcNext(j), true); newRegVert];
end
[newRegConvHullIdx] = convhull(newRegVert(:,1),newRegVert(:,2));
newReg = Region(newRegVert(newRegConvHullIdx,:));

try
    newReg = intersect(newReg,reg(aut.q{idxMode}));
    
    % Subtract the underapproximated reactive funnel
    newRegVert = buildNewRegion(ellAcNext(idxLast), true);
    
    [newRegConvHullIdx] = convhull(newRegVert(:,1),newRegVert(:,2));
    subReg = Region(newRegVert(newRegConvHullIdx,:));
    subReg = intersect(subReg,reg(aut.q{idxMode}));
    
    newReg = regiondiff(newReg.p,subReg.p);
    
    %reg = [reg; newReg];
catch
    warning(['The computed region partition may be erroneous. If running a MATLAB version earlier than v7.15, you need to install Polygon Clipper from MatlabCentral: ',...
        ' http://www.mathworks.com/matlabcentral/fileexchange/8818-polygon-clipper'])
end

% Plot the new region!
plot(newReg,'m')


      