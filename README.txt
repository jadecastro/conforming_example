
This software package reproduces the conforming funnels computed in Fig. 5 from the paper:

[1] J. A. DeCastro and H. Kress-Gazit, “Nonlinear Controller Synthesis and Automatic Workspace Partitioning for Reactive High-Level Behaviors,” to appear in 19th ACM International Conference on Hybrid Systems: Computation and Control (HSCC). Vienna, Austria, 2016.

The example encapsulates the main contribution of the paper: computation of conforming funnels for a nonlinear system.


Installation
============

First, clone the repo: git@github.com:jadecastro/conforming_example.git

This package requires MATLAB 2012b or later.  In addition, it requires installation of the following dependencies.  It may be convenient to install these within the provided 'lib' folder in the project directory, to make the path set-up easier.

1) SeDuMi (version 1.3 tested):
     http://sedumi.ie.lehigh.edu/?page_id=58

2) Ellipsoidal Toolbox (version 1.1.3 tested):
     http://systemanalysisdpt-cmc-msu.github.io/ellipsoids/

3) Multi-Parametric Toolbox (version 2.6.3 tested) 
     http://people.ee.ethz.ch/~mpt/2/downloads/

4) Drake (either Binary or Source installation):
     https://github.com/RobotLocomotion/drake/wiki

5) Mosek (optional, but preferred):
     https://www.mosek.com/resources/downloads
     A license for Mosek can requested free of charge for academic users or purchased for non-academic users.  
     If Mosek is not installed, then SeDuMi is used instead as the optimization engine.

Running the Example
===================

To run the example, first set up the path using the script "setupPath.m".  The paths assume that the dependencies reside in the 'lib' folder, but these can be adjusted as needed.

Within your 'drake' directory, open the file /examples/DubinsCar/DubinsPlant.m.  This contains the dynamics of the system that we will be controlling and verifying. Modify the speed parameter ('v' in line 5) to be v = 0.07.  This will adjust the speed of the vehicle appropriately for the example problem domain.

Next, load the data contained in "box_pushing_example.mat".  This populates the workspace with pre-computed controllers that execute several of the transitions of the finite-state machine described in Sec. 7.1 of [1] for a unicycle-type robot, as well as a trajectory that drives the system from an initial condition within the controller to a final condition within the same region.  Several parameters are also given in the "options" structure.  

With the workspace variables loaded, execute the following functions:

1) buildConformingFunnel: given a transition atomic controller, a trajectory starting from within the funnel for that controller, a region object, and a finite-state machine, compute a feedback controller and verify invariance using a quadratically-parameterized funnel with respect to the given trajectory.  If this fails, then compute a conforming funnel to guarantee the system remains within the desired region using the barrier function technique of [1].

2) simulateConformingFunnel: given a barrier function-based atomic controller, simulate the system over a finite time interval from an initial condition of the user's choosing. 

Questions or comments can be addressed to Jon: jad455@cornell.edu
