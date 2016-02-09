%
% Set up the path for using the "Conforming Funnels" HSCC 2016 Repeatability Package.
% 

% Modify the following to reflect the installation paths for Drake and Mosek.  
% The following assumes all dependencies have been installed in the 'lib' folder of the current project.
addpath(genpath([pwd,'/..']));
addpath(genpath([pwd,'/../lib/drake-distro/spotless']));
addpath([pwd,'/../lib/drake-distro/drake']);

addpath(genpath([pwd,'/../lib/mosek']));

run([pwd,'/../lib/drake-distro/drake/addpath_drake']);

% Comment/uncomment either of the following if you get 'not found on path' warnings when attempting to remove the examples folder 
rmpath(genpath([pwd,'/../lib/drake-distro/drake/examples']));
%rmpath(genpath(GetFullPath([pwd,'/../lib/drake-distro/drake/examples'])));

addpath([pwd,'/../lib/drake-distro/drake/examples/DubinsCar']);

% If installing ellipsoids version 1.1.3 and MPT version 2.6.3, these packages come bundled with SeDuMi, 
% which may cause errors (e.g. invalid MEX file errors). To remedy this, remove them from the path:
rmpath([pwd,'/../lib/ellipsoids/solvers/SeDuMi_1_1']);
rmpath([pwd,'/../lib/mpt/solvers/SeDuMi_1_3']);

