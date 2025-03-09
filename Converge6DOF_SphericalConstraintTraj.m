function TestSymbolicImplementation()
    engineType = 3; % MONOPROPELLANT
    % engineType = 4; % COLD GAS
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    for ii = 1:6, problemParameters.dynamics.maxThrust(ii)=.004; end
    solutionCG_AlignedUnconstrained.solutionFound = false; 
    solverParameters.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    solutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
   
    PlotSummary(solutionCG_AlignedUnconstrained);
    %% Test rotation only
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(2100,engineType,THRUSTER_CONFIGURATION.RCS_12);
    solutionAttitudeOnly = OLD_Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    SymbolicsolutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    solutionAttitudeOnly.newCostateGuess - SymbolicsolutionAttitudeOnly.newCostateGuess
    
    %% With constraint
    [problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    problemParametersCG_6_Constrained = UpdateSphereCircleRadius(problemParametersCG_6_Constrained,.1/1000);
    problemParametersCG_6_Constrained.constraint.epsilon = .15;
    solverParametersCG_6_Constrained.rho = .3;
    solverParametersCG_6_Constrained.fSolveOptions.MaxIterations = 200;
    for ii = 1:6, problemParametersCG_6_Constrained.dynamics.maxThrust(ii)=.004; end
    
    solutionCG_Aligned_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained);
    PlotSolution.SixDOF_Traj(solutionCG_Aligned_Constrained);
    PlotSolution.Costates(solutionCG_Aligned_Constrained)
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained);

    solutionCG_Aligned_ConstrainedRadius_Sweep = SweepSolutions(solutionCG_Aligned_Constrained, ...
        'Radius',[.1:.02:2]./1000,true);
    
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_ConstrainedRadius_Sweep(end));
    PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_ConstrainedRadius_Sweep)

    solutionCG_Aligned_Constrained2_rhoSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(end), ...
        'rho',1e-3,true);

    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained2_rhoSweep(end));
    PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_Constrained2_rhoSweep)

    solutionCG_Aligned_ConstrainedEps_Sweep = SweepSolutions(solutionCG_Aligned_Constrained2_rhoSweep(1), ...
        'epsilon',1e-3);

    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_ConstrainedEps_Sweep(end));
    PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_ConstrainedEps_Sweep)

    solutionCG_Aligned_Constrained2SmallEps_RhoSweep = SweepSolutions(solutionCG_Aligned_ConstrainedEps_Sweep(end), ...
        'rho',1e-3,true);

    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained2SmallEps_RhoSweep(end));
    PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_Constrained2SmallEps_RhoSweep)

    sol = solutionCG_Aligned_Constrained2SmallEps_RhoSweep(end);
    PlotSolution.SixDOF_Traj(sol)
    PlotSolution.ThrustProfileAllEngines(sol)
    PlotSolution.PlotOrientationChaserRelativeToTranslationalFrame(sol.x(end,8:10)',sol.x(end,1:3)'*1e3,sol.t(end),sol.problemParameters,gca)
    figure; PlotSolution.MassConsumption(sol,gca)

    % Q: Why do we see a singular arc-esque switch function here for
    % smaller values of epsilon?

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
solverParametersControlTorque.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
solutionCG_Aligned_ControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersControlTorque,solverParametersControlTorque);
% Potentially too many search directions if we use the L2 norm regularization

sol =solutionCG_Aligned_ControlTorque; PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
rhoSweepCG_Aligned_ControlTorque = SweepSolutions(sol,'rho',1e-4,true); % This was done using L2 norm regularization
rhoKappaSweepCG_Aligned_ControlTorque = SweepSolutions(sol,'rhokappa',[1e-4,1e-3],true);
sol =rhoKappaSweepCG_Aligned_ControlTorque(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)

% Findings: 1. The above two converge to the same value but have trouble
% reducing kappa further than this. 
% 2. The very slight rotation of the LVLH frame during the 48 second
% transfer means that the chaser should rotate slightly for the terminal
% burn. This is observed
% 3. The kappa sweep is easier than the L2 norm regularization

% A high initial value of kappa discourages the use of the control torque.
% Q1: What value is 'good' ? Want it to be 100x less than the cost of using
% the throttle
PlotSolution.CostBreakdown(rhoKappaSweepCG_Aligned_ControlTorque(end))
% High angular accelerations lead to instability in the MRP. 
% Q2 what value of max torque should we use? 

sol =rhoSweepCG_Aligned_ControlTorque(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
PlotSolution.ConvergedCostateTrace(rhoSweepCG_Aligned_ControlTorque);

%% 6x CG aligned with control Torque with free final MRP
[problemParametersControlTorqueFreeMRP, solverParametersControlTorqueFreeMRP] = ConstrainedApproachTestCondition(2011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersControlTorqueFreeMRP.initialCostateGuess = rhoKappaSweepCG_Aligned_ControlTorque(1).newCostateGuess;
problemParametersControlTorqueFreeMRP.dynamics.finalAttitudeFree = true;
solCG6_Torque_FreeMRP = Solve6DOFPointingConstrainedControlProblem(problemParametersControlTorqueFreeMRP,solverParametersControlTorqueFreeMRP);
rhoKappaSweepCG6_Torque_FreeMRP = SweepSolutions(solCG6_Torque_FreeMRP,'rhokappa',[1e-3,1e-3],true);
sol =rhoKappaSweepCG6_Torque_FreeMRP(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
% Negligible change here. maybe slightly lower cost? Likely more obvious
% for longer transfers where the frame rotates more. 

%% 6x CG aligned with and 2m spherical constraint AND control torque
[problemParametersCG_6_ConstraintTorque, solverParametersCG_6_ConstraintTorque] = ConstrainedApproachTestCondition(2013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_ConstraintTorque.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
problemParametersCG_6_ConstraintTorque = UpdateSphereCircleRadius(problemParametersCG_6_ConstraintTorque,.1/1000);
problemParametersCG_6_ConstraintTorque.constraint.epsilon = .1;
solutionCG_Aligned_ConstrainedControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstraintTorque,solverParametersCG_6_ConstraintTorque);
sweepCG_Aligned_ConstrainedControlTorque_Mr= SweepSolutions(solutionCG_Aligned_ConstrainedControlTorque, ...
    'rho',0.2,true);
PlotSolution.ThrustProfileAllEngines(sweepCG_Aligned_ConstrainedControlTorque_Mr(end))

solutionCG_Aligned_ConstrainedControlTorqueR2 = SweepSolutions(sweepCG_Aligned_ConstrainedControlTorque_Mr(end),'Radius', ...
    [0.1:.025:2]./1000,true);

PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_ConstrainedControlTorqueR2(end));
PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_ConstrainedControlTorqueR2);
PlotSolution.RotationalSummary(solutionCG_Aligned_ConstrainedControlTorqueR2(end));
solutionCG_Constraint2Torque_Sr_Se = SweepSolutions(solutionCG_Aligned_ConstrainedControlTorqueR2(end), ...
    'kappa',1e-3,true);

PlotSolution.ThrustProfileAllEngines(solutionCG_Constraint2Torque_Sr_Se(end));
PlotSolution.ConvergedCostateTrace(solutionCG_Constraint2Torque_Sr_Se);
PlotSolution.RotationalSummary(solutionCG_Constraint2Torque_Sr_Se(end));
PlotSolution.CostBreakdown(solutionCG_Constraint2Torque_Sr_Se(end));
PlotSolution.CostBreakdown(solutionCG_Constraint2Torque_Sr_Se(1));
% Save 33g of fuel by allowing for rotations! Awesome result (unconverged
% rho).
% Chaser gets to final pos/vel earlier (at 42 seconds) and then reorients
% so that final MRP is zero. This only costs 2-3g mroe fuel than when the
% final MRP is free

solutionCG_Constraint2ExpensiveTorque_Sr_Se = SweepSolutions(solutionCG_Constraint2Torque_Sr_Se(1), ...
    'rho',1e-3,true);
solutionCG_Constraint2CheaperTorque_Sr_Se = SweepSolutions(solutionCG_Constraint2Torque_Sr_Se(end), ...
    'rho',1e-3,true);
PlotSolution.ConvergedCostateTrace(solutionCG_Constraint2ExpensiveTorque_Sr_Se);
PlotSolution.ConvergedCostateTrace(solutionCG_Constraint2CheaperTorque_Sr_Se);

for sol = [solutionCG_Constraint2ExpensiveTorque_Sr_Se(end), solutionCG_Constraint2CheaperTorque_Sr_Se(end)]
    PlotSolution.ThrustProfileAllEngines(sol);
    PlotSolution.RotationalSummary(sol);
    PlotSolution.CostBreakdown(sol);
end
sol =  solutionCG_Constraint2CheaperTorque_Sr_Se(end);
PlotSolution.States(sol);

%% 6x CG aligned with control torque and 1.4m spherical constraint, free final MRP
testing = solutionCG_Aligned_ConstrainedControlTorqueR2(1);
testing.problemParameters.dynamics.finalAttitudeFree = true;

k=RerunSolution(testing);
solCG6_Constraint_1_4_Torque_FreeMRP = SweepSolutions(k,'Radius', ...
    [0.1:.025:1.4]./1000,true);
PlotSolution.ThrustProfileAllEngines(solCG6_Constraint_1_4_Torque_FreeMRP(end));
PlotSolution.ConvergedCostateTrace(solCG6_Constraint_1_4_Torque_FreeMRP);
PlotSolution.RotationalSummary(solCG6_Constraint_1_4_Torque_FreeMRP(end));


%% 6x CG aligned with control torque and 2m spherical constraint, free final MRP
testing3= SweepSolutions(solCG6_Constraint_1_4_Torque_FreeMRP(end),'kappa', ...
    .5,true);
PlotSolution.ThrustProfileAllEngines(testing3(end));
PlotSolution.ConvergedCostateTrace(testing3);
PlotSolution.RotationalSummary(testing3(end));

solCG6_Constraint_1_8_Torque_FreeMRP = SweepSolutions(testing3(end),'Radius', ...
    [1.4:.01:1.8]./1000,true);
testing2 = SweepSolutions(solCG6_Constraint_1_8_Torque_FreeMRP(end),'epsilon', ...
    .05,true);
solCG6_Constraint_2_Torque_FreeMRP = SweepSolutions(testing2(end),'Radius', ...
    [1.8:.01:2]./1000,true);
testing4= SweepSolutions(solCG6_Constraint_2_Torque_FreeMRP(end),'kappa', ...
    .25,true);
PlotSolution.ConvergedCostateTrace(testing4);
PlotSolution.ThrustProfileAllEngines(solCG6_Constraint_2_Torque_FreeMRP(end));
PlotSolution.ConvergedCostateTrace(solCG6_Constraint_2_Torque_FreeMRP);
PlotSolution.RotationalSummary(solCG6_Constraint_2_Torque_FreeMRP(end));

solCG6_Constraint2_Torque_FreeMRP_SmallRho = SweepSolutions(testing4(end),'rho', ...
    0.01,true);
PlotSolution.ThrustProfileAllEngines(solCG6_Constraint2_Torque_FreeMRP_SmallRho(end));
PlotSolution.ConvergedCostateTrace(solCG6_Constraint2_Torque_FreeMRP_SmallRho);
PlotSolution.RotationalSummary(solCG6_Constraint2_Torque_FreeMRP_SmallRho(end));
solCG6_Constraint2_Torque_FreeMRP_VerySmallEpsilon = SweepSolutions(solCG6_Constraint2_Torque_FreeMRP_SmallRho(end),'epsilon', ...
    0.001,true);
solCG6_Constraint2_Torque_FreeMRP_VVSe = SweepSolutions(solCG6_Constraint2_Torque_FreeMRP_VerySmallEpsilon(end),'epsilon', ...
    1e-5,true);
PlotSolution.ThrustProfileAllEngines(solCG6_Constraint2_Torque_FreeMRP_VVSe(end));
solCG6_Constraint2_Torque_FreeMRP_VerySmallRho = SweepSolutions(solCG6_Constraint2_Torque_FreeMRP_VVSe(end),'rho', ...
    0.002,true);
sol = solCG6_Constraint2_Torque_FreeMRP_VerySmallRho(end);
PlotSolution.SixDOF_Traj(sol)
PlotSolution.PlotOrientationChaserRelativeToTranslationalFrame(sol.x(end,8:10)',sol.x(end,1:3)'*1e3,sol.t(end),sol.problemParameters,gca)
PlotSolution.CostBreakdown(sol);
PlotSolution.RotationalSummary(sol)
PlotSolution.ConvergedCostateTrace(solCG6_Constraint2_Torque_FreeMRP_VerySmallRho)
PlotSolution.ThrustProfileAllEngines(sol);
% Compare cost breakdown vs when MRP is not free 

end

function PlotSummary(sol)
    PlotSolution.ThrustProfileAllEngines(sol)
    PlotSolution.SixDOF_Traj(sol);
    
end