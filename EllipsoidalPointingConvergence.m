function EllipsoidalPointingConvergenceCW()
%% CW Dynamics
    parallelFSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
    'MaxIter',3e1,'TolFun',1e-12,'TolX',1e-11,...
    'UseParallel',false); % fsolve  

    engineType = 3; % MONOPROPELLANT
    % engineType = 4; % COLD GAS
    
    %% 6dof, CW dynamics, no constraint, Easy convergence
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011, ...
        engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6); 
    solverParameters.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    solutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    
    solutionCG_AlignedUnconstrainedrhoSweep = SweepSolutions(solutionCG_AlignedUnconstrained, ...
        'rho',1e-3);
    solutionCG_AlignedUnconstrained = RerunSolution(solutionCG_AlignedUnconstrainedrhoSweep(end-5));

    PlotSolution.ThrustProfileAllEngines(solutionCG_AlignedUnconstrained);
    PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedUnconstrained);
    PlotSolution.SixDOF_Traj(solutionCG_AlignedUnconstrained);

    %% 6dof, CW Dynamics, CG Aligned with 1m spherical constraint (Radius Sweep, then Rho+Eps Sweep) Converge
    [problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    problemParametersCG_6_Constrained = UpdateSphereCircleRadius(problemParametersCG_6_Constrained,0.1/1000); % Very small radius
    problemParametersCG_6_Constrained.constraint.epsilon = .5;
    solutionCG_Aligned_Constrained_SmallRad = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
    solutionCG_Aligned_Constrained_SmallRad = RerunSolution(solutionCG_Aligned_Constrained_SmallRad); % Run this 4-5 times

    solutionCG_Aligned_Constrained_Sweep = SweepSolutions(solutionCG_Aligned_Constrained_SmallRad,'Radius',linspace(0.0001,0.00195,20),true);
    solutionCG_Aligned_Constrained = RerunSolution(solutionCG_Aligned_Constrained_Sweep(end));

    solutionCG_Aligned_Constrained_SmallRhoSmallEpsilon = SweepSolutions(solutionCG_Aligned_Constrained,'rhoepsilon',1e-4,true);
    
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained_SmallRhoSmallEpsilon(end));
    PlotSolution.SixDOF_Traj(solutionCG_Aligned_Constrained_SmallRhoSmallEpsilon(end));

    %% 6dof, CW Dynamics, CG Aligned with ellipsoidal constraint where all semi-major axis equal to 2m (Converged)
    [problemParametersCG_6_Constrained_Ellips, solverParametersCG_6_Constrained_Ellips] = ConstrainedApproachTestCondition(1015,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained_Ellips.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    problemParametersCG_6_Constrained_Ellips.constraint.epsilon = 0.5;

    problemParametersCG_6_Constrained_Ellips.constraint.targetAxisx = 0.0001;
    problemParametersCG_6_Constrained_Ellips.constraint.targetAxisy = 0.0001;
    problemParametersCG_6_Constrained_Ellips.constraint.targetAxisz = 0.0001;
    solutionCG_Aligned_Constrained_Ellips = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained_Ellips,solverParametersCG_6_Constrained_Ellips);
    solverParametersCG_6_Constrained_Ellips.initialCostateGuess = solutionCG_Aligned_Constrained_Ellips.newCostateGuess;

    newSols = SweepSolutions(solutionCG_Aligned_Constrained_Ellips, 'Ellipsoid1', linspace(0.0001,0.002,40), true);
    solutionCG_Aligned_Constrained_Ellips_Sweep = RerunSolution(newSols(end));
    solverParametersCG_6_Constrained_Ellips.initialCostateGuess = solutionCG_Aligned_Constrained_Ellips_Sweep.newCostateGuess;

    PlotSolution.SwitchFunctionAnalysisAllEngines(solutionCG_Aligned_Constrained_Ellips_Sweep);
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained_Ellips_Sweep);
    PlotSolution.SixDOF_Traj(solutionCG_Aligned_Constrained_Ellips_Sweep);

%% 6dof, CW Dynamics, CG Aligned with ellipsoidal constraint with different semi-major axis + rotated (ellipsoid) (Converged)
    [problemParametersCG_6_Constrained_Ellips2, solverParametersCG_6_Constrained_Ellips2] = ConstrainedApproachTestCondition(1015,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained_Ellips2.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    problemParametersCG_6_Constrained_Ellips2.constraint.epsilon = 0.5;

    problemParametersCG_6_Constrained_Ellips2.constraint.targetAxisx = 0.0001;
    problemParametersCG_6_Constrained_Ellips2.constraint.targetAxisy = 0.0001;
    problemParametersCG_6_Constrained_Ellips2.constraint.targetAxisz = 0.0001;
    
    solutionCG_Aligned_Constrained_Ellips2 = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained_Ellips2,solverParametersCG_6_Constrained_Ellips2);
    solverParametersCG_6_Constrained_Ellips2.initialCostateGuess = solutionCG_Aligned_Constrained_Ellips2.newCostateGuess;

    newSols = SweepSolutions(solutionCG_Aligned_Constrained_Ellips2, 'Ellipsoid2', [linspace(0.0001,0.0015,60)',linspace(0.0001,0.001,60)',linspace(0.0001,0.002,60)'], true);
    solutionCG_Aligned_Constrained_Ellips2 = RerunSolution(newSols(end));
    solverParametersCG_6_Constrained_Ellips2.initialCostateGuess = solutionCG_Aligned_Constrained_Ellips2.newCostateGuess;

    newSols2 = SweepSolutions(solutionCG_Aligned_Constrained_Ellips2,'rho',1e-1);
    newSols2a = SweepSolutions(newSols2(end),'epsilon',1e-1);
    newSols2b = SweepSolutions(newSols2a(end),'rho',1e-2);
    newSols2c = SweepSolutions(newSols2b(end),'epsilon',1e-2);
    newSols2d = SweepSolutions(newSols2c(end),'rho',1e-3);
    newSols2e = SweepSolutions(newSols2d(end),'epsilon',1e-3);
    solutionCG_Aligned_Constrained_Ellips2 = RerunSolution(newSols2e(end));

    PlotSolutionOLD.PlumeAngleSixDOF(solutionCG_Aligned_Constrained_Ellips2);
    PlotSolutionOLD.ThrustProfileAllEngines(solutionCG_Aligned_Constrained_Ellips2);
    PlotSolutionOLD.SixDOF_Traj(solutionCG_Aligned_Constrained_Ellips2);
    addThrustDirection6DOF(solutionCG_Aligned_Constrained_Ellips2,gca);

%% 6dof, CW Dynamics, CG Aligned with ellipsoidal constraint + Attitude Control with different semi-major axis (ellipsoid) (Not Converged)

    [problemParametersCG_6_Unconstrained_Ellips_Rot, solverParametersCG_6_Unconstrained_Ellips_Rot] = ConstrainedApproachTestCondition(2011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Unconstrained_Ellips_Rot.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    for ii = 1:6, problemParametersCG_6_Unconstrained_Ellips_Rot.dynamics.maxThrust(ii)=.005; end % Set max thrust for each thruster

    solutionCG_Aligned_Unconstrained_Ellips_Rot = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Unconstrained_Ellips_Rot,solverParametersCG_6_Unconstrained_Ellips_Rot);

    rhoSweepCG_Aligned_ControlTorque = SweepSolutions(solutionCG_Aligned_Unconstrained_Ellips_Rot,'rho',1e-1,true); % This was done using L2 norm regularization
    rhoKappaSweepCG_Aligned_ControlTorque = SweepSolutions(rhoSweepCG_Aligned_ControlTorque,'kappa',1e-3,true);
    sol =rhoKappaSweepCG_Aligned_ControlTorque(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
    
    [problemParametersCG_6_ConstraintTorque, solverParametersCG_6_ConstraintTorque] = ConstrainedApproachTestCondition(2015,3,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_ConstraintTorque.initialCostateGuess = [-0.4515    2.3920   -0.0000   -9.8852   57.1118   -0.0000    0.0001   -0.0000   -0.0000    0.0125    0.0000   -0.0000    0.0285]';

    problemParametersCG_6_ConstraintTorque.constraint.targetAxisx = 0.0001; %km, 0.1m to then start sweep
    problemParametersCG_6_ConstraintTorque.constraint.targetAxisy = 0.0001; %km
    problemParametersCG_6_ConstraintTorque.constraint.targetAxisz = 0.0001; %km
    problemParametersCG_6_ConstraintTorque.constraint.epsilon = .5;
    solverParametersCG_6_ConstraintTorque.initialCostateGuess = solutionCG_Aligned_Unconstrained_Ellips_Rot.newCostateGuess;
    solutionCG_Aligned_ConstrainedControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstraintTorque,solverParametersCG_6_ConstraintTorque);

    % Kappa Sweep: Because torque switch function is singular over some arc
    % of the trajectory. Use L2-norm regularization method. Large kappa -->
    % enforce optimal torque to 0
    % Then Semi-major axis sweep, then rho and epsilon sweeps
    newSols3 = SweepSolutions(solutionCG_Aligned_ConstrainedControlTorque, 'kappa', 1e-1, true);
    newSols3a = SweepSolutions(newSols3(end), 'Ellipsoid2', [linspace(0.0001,0.00075,50)',linspace(0.0001,0.0005,50)',linspace(0.0001,0.001,50)'], true, false);
    newSols3aa = SweepSolutions(newSols3a(end), 'kappa', 1e-2, true);
    newSols3b = SweepSolutions(newSols3aa(end), 'Ellipsoid2', [linspace(0.00075,0.0015,50)',linspace(0.0005,0.001,50)',linspace(0.001,0.002,50)'], true, false);
    newSols3c = SweepSolutions(newSols3b(end),'rho',1e-1);
    newSols3d = SweepSolutions(newSols3c(end),'epsilon',1e-1);
    newSols3e = SweepSolutions(newSols3d(end),'rho',1e-2);
    newSols3f = SweepSolutions(newSols3e(end),'epsilon',1e-2);
    newSols3g = SweepSolutions(newSols3f(end), 'kappa', 1e-3);
    solutionCG_Aligned_ConstrainedControlTorque = RerunSolution(newSols3g(end));

    PlotSolutionOLD.PlumeAngleSixDOF(solutionCG_Aligned_ConstrainedControlTorque);
    PlotSolutionOLD.ThrustProfileAllEngines(solutionCG_Aligned_ConstrainedControlTorque);
    PlotSolutionOLD.SixDOF_Traj(solutionCG_Aligned_ConstrainedControlTorque);
    addThrustDirection6DOF(solutionCG_Aligned_ConstrainedControlTorque,gca);












    %% Test rotation only
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(2100,engineType,THRUSTER_CONFIGURATION.RCS_12);
    solutionAttitudeOnly = OLD_Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    SymbolicsolutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    solutionAttitudeOnly.newCostateGuess - SymbolicsolutionAttitudeOnly.newCostateGuess
    
    %% With constraint
    [problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    problemParametersCG_6_Constrained = UpdateSphereCircleRadius(problemParametersCG_6_Constrained,1/1000);
    problemParametersCG_6_Constrained.constraint.epsilon = .5;
    for ii = 1:6, problemParametersCG_6_Constrained.dynamics.maxThrust(ii)=.005; end
    
    solutionCG_Aligned_Constrained = OLD_Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
    PlotSummary(solutionCG_Aligned_Constrained);
    PlotSolution.Costates(solutionCG_Aligned_Constrained)
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained);
    solutionCG_Aligned_ConstrainedEps_Sweep = SweepSolutions(solutionCG_Aligned_Constrained, ...
        'epsilon',1e-4);
    solutionCG_Aligned_ConstrainedRhoEps_Sweep = SweepSolutions(solutionCG_Aligned_Constrained, ...
        'rhoepsilon',1e-4);

    solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_Aligned_Constrained.newCostateGuess;
    propagatedSymbolic = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained,true);
    PlotSummary(propagatedSymbolic);
    PlotSolution.Costates(propagatedSymbolic)

    solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    solverParametersCG_6_Constrained.fSolveOptions.MaxIterations = 300;
    SymbolicSolutionCG_Aligned_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
    % Rerun a few times
    solverParametersCG_6_Constrained.initialCostateGuess = SymbolicSolutionCG_Aligned_Constrained.newCostateGuess;
    SymbolicSolutionCG_Aligned_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
    PlotSummary(SymbolicSolutionCG_Aligned_Constrained);
    
    solutionCG_AlignedSphericalConstraint2_SmallEpsilon = SweepSolutions(SymbolicSolutionCG_Aligned_Constrained, ...
        'epsilon',.1,true,true);
    PlotSolution.ConvergedCostateTrace(solutionCG_AlignedSphericalConstraint2_SmallEpsilon)
    PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedSphericalConstraint2_SmallEpsilon(1));
    PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedSphericalConstraint2_SmallEpsilon(end));
    PlotSummary(solutionCG_AlignedSphericalConstraint2_SmallEpsilon(end));

    solutionCG_AlignedSphericalConstraint2_SmallEpsilonSmallRho = SweepSolutions(solutionCG_AlignedSphericalConstraint2_SmallEpsilon(end), ...
        'rho',1e-1,true,true);
    PlotSolution.ConvergedCostateTrace(solutionCG_AlignedSphericalConstraint2_SmallEpsilonSmallRho)
    PlotSummary(solutionCG_AlignedSphericalConstraint2_SmallEpsilonSmallRho(end));
    solutionCG_AlignedSphericalConstraint2_SmallEpsilonVerySmallRho = SweepSolutions(solutionCG_AlignedSphericalConstraint2_SmallEpsilonSmallRho(end), ...
        'rho',1e-3,true,true);
    PlotSummary(solutionCG_AlignedSphericalConstraint2_SmallEpsilonVerySmallRho(end));
    PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedSphericalConstraint2_SmallEpsilonVerySmallRho(end));
    solutionCG_AlignedSphericalConstraint2_VSe_VSr= SweepSolutions(solutionCG_AlignedSphericalConstraint2_SmallEpsilonVerySmallRho(end), ...
        'epsilon',1e-2,true,true);
    PlotSolution.ConvergedCostateTrace(solutionCG_AlignedSphericalConstraint2_SmallEpsilon)
    PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedSphericalConstraint2_VSe_VSr(end));
    PlotSummary(solutionCG_AlignedSphericalConstraint2_VSe_VSr(end));

    solutionCG_AlignedSphericalConstraint2_VVSe_VSr= SweepSolutions(solutionCG_AlignedSphericalConstraint2_VSe_VSr(end), ...
        'epsilon',1e-4,true,true);
    PlotSummary(solutionCG_AlignedSphericalConstraint2_VVSe_VSr(end));
        PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedSphericalConstraint2_VVSe_VSr(end));




end 