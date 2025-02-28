function TestSymbolicImplementation()
    engineType = 3; % MONOPROPELLANT
    % engineType = 4; % COLD GAS
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    for ii = 1:6, problemParameters.dynamics.maxThrust(ii)=.005; end
    solutionCG_AlignedUnconstrained.solutionFound = false; 
    solverParameters.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    while any(~solutionCG_AlignedUnconstrained.solutionFound)
        %solverParameters.initialCostateGuess = [rand(3,1)/1e3;rand(3,1);.01;rand(6,1)*1e-3]*1e-3;
        
        solutionCG_AlignedUnconstrained = OLD_Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
        % unconstrainedInitialRho = PlotSolution.summary(solution);
    end 
    SymbolicSolutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    SymbolicSolutionCG_AlignedUnconstrained.newCostateGuess - solutionCG_AlignedUnconstrained.newCostateGuess
    PlotSummary(solutionCG_AlignedUnconstrained);
    PlotSummary(SymbolicSolutionCG_AlignedUnconstrained);
    %% Test rotation only
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(2100,engineType,THRUSTER_CONFIGURATION.RCS_12);
    solutionAttitudeOnly = OLD_Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    SymbolicsolutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    solutionAttitudeOnly.newCostateGuess - SymbolicsolutionAttitudeOnly.newCostateGuess
    
    %% With constraint
    [problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained.initialCostateGuess = SymbolicSolutionCG_AlignedUnconstrained.newCostateGuess;
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

    solverParametersCG_6_Constrained.initialCostateGuess = SymbolicSolutionCG_AlignedUnconstrained.newCostateGuess;
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


    %% Test unconstrained/constrained with canted thrusters. should be no difference in trajectories 
    [problemParametersUnconstrainedCG_Canted, solverParametersUnconstrainedCG_Canted] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8);
    
    solverParametersUnconstrainedCG_Canted.initialCostateGuess = [-0.000089618015403   2.382117256137634   0.000000000000000  -2.076139519736225  57.115489612329533  -0.000000000000000   0.000142936265578  -0.000008931051246  -0.000001093674343 0.000003524540051  -0.000071361039315  -0.000010314917224   0.000008709836245]';
    solutionUnconstrainedCG_Canted = Solve6DOFPointingConstrainedControlProblem(problemParametersUnconstrainedCG_Canted,solverParametersUnconstrainedCG_Canted);
    PlotSummary(solutionUnconstrainedCG_Canted)

    UnconstrainedCG_CantedShortTimeRhoSweep = SweepSolutions(solutionUnconstrainedCG_Canted,'rho',1e-4,true,true);
    solutionCG_CantedUnconstrainedSmallRho = RerunSolution(UnconstrainedCG_CantedShortTimeRhoSweep(end));
    PlotSummary(UnconstrainedCG_CantedShortTimeRhoSweep(end));
    PlotSolution.ConvergedCostateTrace(UnconstrainedCG_CantedShortTimeRhoSweep);

    % Now add a 1m constraint to this, Has no effect because thrusters are
    % canted
    [problemParametersCG_Canted_Constrained, solverParametersCG_Canted_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8);
    solverParametersCG_Canted_Constrained.initialCostateGuess = solutionUnconstrainedCG_Canted.newCostateGuess;
    problemParametersCG_Canted_Constrained = UpdateSphereCircleRadius(problemParametersCG_Canted_Constrained,1/1000);
    problemParametersCG_Canted_Constrained.constraint.epsilon = .5;
    SymbolicSolutionCG_Canted_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_Canted_Constrained,solverParametersCG_Canted_Constrained);

    solutionCG_Canted_Constrained = OLD_Solve6DOFPointingConstrainedControlProblem(problemParametersCG_Canted_Constrained,solverParametersCG_Canted_Constrained);
    solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon = SweepSolutions(SymbolicSolutionCG_Canted_Constrained,'rhoepsilon',1e-4,true);

%% 6x CG aligned with control Torque 
[problemParametersControlTorque, solverParametersControlTorque] = ConstrainedApproachTestCondition(2011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersControlTorque.initialCostateGuess = SymbolicSolutionCG_AlignedUnconstrained.newCostateGuess;
solutionCG_Aligned_ControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersControlTorque,solverParametersControlTorque);

sol =solutionCG_Aligned_ControlTorque; PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
rhoSweepCG_Aligned_ControlTorque = SweepSolutions(sol,'rho',1e-4,true);

sol =rhoSweepCG_Aligned_ControlTorque(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)

%% 6x CG aligned with and1m spherical constraint AND control torque
[problemParametersCG_6_ConstrainedControlTorque, solverParametersCG_6_ConstrainedControlTorque] = ConstrainedApproachTestCondition(2013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_ConstrainedControlTorque.initialCostateGuess = solutionCG_AlignedSphericalConstraint2_SmallEpsilon.newCostateGuess;
problemParametersCG_6_ConstrainedControlTorque = UpdateSphereCircleRadius(problemParametersCG_6_ConstrainedControlTorque,1/1000);
problemParametersCG_6_ConstrainedControlTorque.constraint.epsilon = .1;
solutionCG_Aligned_ConstrainedControlTorque = OLD_Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstrainedControlTorque,solverParametersCG_6_ConstrainedControlTorque);
solutionCG_Constraint2Torque_Sr_Se = SweepSolutions(solutionCG_Aligned_ConstrainedControlTorque,'rhoepsilon',1e-4,true);

sol = solutionCG_Constraint2Torque_Sr_Se(end);
PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
PlotSolution.ConvergedCostateTrace(solutionCG_Constraint2Torque_Sr_Se);
PlotSolution.CostBreakdown(sol);

%% 6x CG aligned with control torque and 1m spherical constraint, free final MRP
[problemParametersCG_6_ConstrainedFreeMRP, solverParametersCG_6_ConstrainedFreeMRP] = ConstrainedApproachTestCondition(2013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_ConstrainedFreeMRP.initialCostateGuess = solutionCG_AlignedSphericalConstraint2_SmallEpsilon(end).newCostateGuess;
problemParametersCG_6_ConstrainedFreeMRP = UpdateSphereCircleRadius(problemParametersCG_6_ConstrainedFreeMRP,2/1000);
problemParametersCG_6_ConstrainedFreeMRP.constraint.epsilon = .1;
problemParametersCG_6_ConstrainedFreeMRP.dynamics.finalAttitudeFree = true;

testing 
% solverParametersCG_6_ConstrainedFreeMRP.fSolveOptions.MaxIterations = 300;
[problemParametersCG_6_ConstrainedFreeMRP, inertiaNewFinal] = UpdateInertia(problemParametersCG_6_ConstrainedFreeMRP,100);

solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstrainedFreeMRP,solverParametersCG_6_ConstrainedFreeMRP);
solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia = RerunSolution(solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia);
solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep = SweepSolutions(solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia,'Inertia',...
    linspace(inertiaNewFinal(1),inertiaNewFinal(2),50),true);


end

function PlotSummary(sol)
    PlotSolution.ThrustProfileAllEngines(sol)
    PlotSolution.SixDOF_Traj(sol);
    
end