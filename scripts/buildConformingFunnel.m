
function [ac, bc] = buildConformingFunnel(reg, aut, ac_trans, x0, u0, options)


idxMode = 4;
itrans = 5;

% Specify the index of an transition funnel at which to attempt to compose
% its cross-section with an inward-facing funnel whose final set lies in
% the intersection of all transition funnels outgoing from this region.
indexToCompose = 50;  

trans = vertcat(aut.trans{:});

% Avoid regions for starting region
% regSafeS = getReg(reg,regBnd,aut,idxMode);

ac = [];
bc = [];

% ==========================
% Compute the funnel

funFail = true;

isectReact = false;

lastTrans = trans(:,2)==idxMode;
    
acNext = [ac_trans{itrans}];
acNext.sys.params.ctrloptions.R = 0.01;
sys = acNext.sys;

regMode = reg(aut.q{idxMode});

x00 = x0;  u00 = u0;

tmp = acNext.ellipsoid;
ellToCompose = tmp(indexToCompose);


% plot things
figure(90), hold on, axis equal
%plot(reg)
plot(acNext,sys,90)

figure(3), hold on, axis equal
plot(reg(aut.q{idxMode}),'r')


rhof = 0.1;   %final rho.  TODO: handle the more general case and get it from containment
options.isMaximization = true;
[ac, c] = computeAtomicController(u00,x00,sys,regMode,ellToCompose,options,rhof);

plot(ac.x0,'k',5)
rhoi = double(ac.rho,0);
[res, isectIdx, isectArray] = isinside(ac,reg(aut.q{trans(itrans,1)}),sys);
if ~res
    % NB: the following assumes only one contiguous interval where the funnel left the region.
    % split the funnel into three parts:
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
    
    % Now, construct the barriers
    t01 = t(min(isectIdx):max(isectIdx));
    x01 = Traject(t01,double(ac.x0,t01));
    u01 = Traject(t01,double(ac.u0,t01));                                        
    
    regMode = reg(aut.q{idxMode});
    
    % define the initial set as the end of the prefix funnel
    if ~isempty(acPre)
        tmp = acPre.ellipsoid;
        ellToCompose = tmp(end);
    else
        tmp = acNext.ellipsoid;
        ellToCompose = tmp(indexToCompose);
    end
    
    [bc] = computeConformingFunnel(u00,x00,u01,x01,sys,ac,regMode,ellToCompose,options);
    
    % Plot stuff
    figure(90)
    axis equal
    hold on
    plot(reg(1),'r')
    plot(reg(2),'g')
    
    bc.plot(ac.x0,90)
    
    % pause
    % plot(acNext,sys,90,[],[0,0,1])
    % plot(acPost,sys,90,[],[0,1,0])
    plot(acNext.x0,'k',90)
    plot(ac.x0,'k',90)
    
end
funFail = false;

if ~funFail
    
    plot(ac.x0,'k',3)
    
    % Create a new region based on the last unverified index of the next funnel.
    funStepSize = 130;
    idxLast = indexToCompose + funStepSize;
    
    % Create the new region
    newRegVert = [];
    ellAcNext = projection(acNext,sys);
    for j = length(acNext.x0.getTimeVec()):-funStepSize:idxLast
        newRegVert = [buildNewRegion(ellAcNext(j), true); newRegVert];
    end
    [newRegConvHullIdx] = convhull(newRegVert(:,1),newRegVert(:,2));
    newReg = Region(newRegVert(newRegConvHullIdx,:));
    newReg = intersect(newReg,reg(aut.q{idxMode}));
    
    % Subtract the underapproximated reactive funnel
    newRegVert = buildNewRegion(ellAcNext(idxLast), true);
    
    [newRegConvHullIdx] = convhull(newRegVert(:,1),newRegVert(:,2));
    subReg = Region(newRegVert(newRegConvHullIdx,:));
    subReg = intersect(subReg,reg(aut.q{idxMode}));
    
    newReg = regiondiff(newReg.p,subReg.p);
    
    %reg = [reg; newReg];
    
    % plot it!
    plot(newReg,'m')
    
    ac = acPre;
    ac = [ac; acPost];
    
end
            