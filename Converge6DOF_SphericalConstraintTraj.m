function Converge6DOF_SphericalConstraintTraj()
    engineType = 3; % MONOPROPELLANT
    engineType = 2; % BIPROPELLANT
    % engineType = 4; % COLD GAS
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    for ii = 1:6, problemParameters.dynamics.maxThrust(ii)=.004; end
    solverParameters.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    solutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    
    solutionCG_AlignedUnconstrainedrhoSweep = SweepSolutions(solutionCG_AlignedUnconstrained, ...
        'rho',1e-3);
    PlotSolution.ThrustProfileAllEngines(solutionCG_AlignedUnconstrainedrhoSweep(end));

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
    for ii = 1:6, problemParametersCG_6_Constrained.dynamics.maxThrust(ii)=.003; end
    
    solutionCG_Aligned_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained);
    PlotSolution.SixDOF_Traj(solutionCG_Aligned_Constrained);
    PlotSolution.Costates(solutionCG_Aligned_Constrained)
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained);

    solutionCG_Aligned_ConstrainedRadius_Sweep = SweepSolutions(solutionCG_Aligned_Constrained, ...
        'Radius',[.1:.02:3.4]./1000,true);

    PlotSolution.Sweep6DOF(solutionCG_Aligned_ConstrainedRadius_Sweep)
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_ConstrainedRadius_Sweep(end));
    PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_ConstrainedRadius_Sweep)

    %radii = [.5,1,1.5,2,2.5,3,3.4];
    idxsToSweep = [23,49,74,99,124,149,169];
    solutionCG_Aligned_Constrained2_rhoSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(end), ...
        'rho',1e-3,true);


    solutionCG_Aligned_ConstrainedR05_rhoSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(23), ...
        'rho',1e-3,true);
    solutionCG_Aligned_ConstrainedR05_VerySmallrhoSweep = SweepSolutions(solutionCG_Aligned_ConstrainedR05_rhoSweep(end), ...
        'rho',1e-4,true); % This does the trick!     
    NowSweepEps= SweepSolutions(solutionCG_Aligned_ConstrainedR05_rhoSweep(end), ...
        'epsilon',1e-2,true); % Got to 30 iterations and eps =.0878
    
    solutionCG_Aligned_ConstrainedR1_epsSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(49), ...
        'epsilon',1e-2,true);
    solutionCG_Aligned_ConstrainedR1_rhoSweep = SweepSolutions(solutionCG_Aligned_ConstrainedR1_epsSweep(4), ...
        'rho',1e-3,true); 

    solutionCG_Aligned_ConstrainedR15_epsSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(74), ...
        'epsilon',1e-2,true);
    solutionCG_Aligned_ConstrainedR15_Se_Sr = SweepSolutions(solutionCG_Aligned_ConstrainedR15_epsSweep(end), ...
        'rho',1e-2,true);

    solutionCG_Aligned_ConstrainedR25_epsSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(124), ...
        'epsilon',1e-2,true);
    solutionCG_Aligned_ConstrainedR25_Se_Sr = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(123), ...
        'rho',1e-2,true);
    solutionCG_Aligned_ConstrainedR25_epsSweep = SweepSolutions(solutionCG_Aligned_ConstrainedR25_Se_Sr(32), ...
        'epsilon',1e-2,true); 
    solutionCG_Aligned_ConstrainedR25_VE_Sr = SweepSolutions(solutionCG_Aligned_ConstrainedR25_epsSweep(9), ...
        'epsilon',1e-3,true); 

    solutionCG_Aligned_ConstrainedR3_epsSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(149), ...
        'epsilon',.1,true);
    solutionCG_Aligned_ConstrainedR3_Se_Sr = SweepSolutions(solutionCG_Aligned_ConstrainedR3_epsSweep(end), ...
        'rho',1e-2,true);
    solutionCG_Aligned_ConstrainedR3 = SweepSolutions(solutionCG_Aligned_ConstrainedR3_Se_Sr(end), ...
        'epsilon',1e-3,true);

    solutionCG_Aligned_ConstrainedR34_epsSweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(169), ...
        'epsilon',1e-2,true);
    solutionCG_Aligned_ConstrainedR34_Se_Sr = SweepSolutions(solutionCG_Aligned_ConstrainedR34_epsSweep(1), ...
        'rho',3e-2,true);    
    solutionCG_Aligned_ConstrainedR34 = SweepSolutions(solutionCG_Aligned_ConstrainedR34_Se_Sr(end), ...
        'epsilon',1e-3,true);
    
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained2_rhoSweep(end));
    PlotSolution.ConvergedCostateTrace(solutionCG_Aligned_Constrained2_rhoSweep)

    solutionCG_Aligned_ConstrainedEps_Sweep = SweepSolutions(solutionCG_Aligned_ConstrainedRadius_Sweep(end), ...
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
[problemParametersCG_6_ConstraintTorque, solverParametersCG_6_ConstraintTorque] = ConstrainedApproachTestCondition(2013,3,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_ConstraintTorque.initialCostateGuess = [-0.4515    2.3920   -0.0000   -9.8852   57.1118   -0.0000    0.0001   -0.0000   -0.0000    0.0125    0.0000   -0.0000    0.0285]';
problemParametersCG_6_ConstraintTorque = UpdateSphereCircleRadius(problemParametersCG_6_ConstraintTorque,.1/1000);
problemParametersCG_6_ConstraintTorque.constraint.epsilon = .1;
solverParametersCG_6_ConstraintTorque.fSolveOptions.MaxIterations=300;
solutionCG_Aligned_ConstrainedControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstraintTorque,solverParametersCG_6_ConstraintTorque);
thrustSweep = SweepSolutions(solutionCG_Aligned_ConstrainedControlTorque,...
    'Thrust',0.002:0.0005:0.003,true);
c1 = thrustSweep(end).problemParameters.dynamics.exhaustVelocity;
c2 =problemParameters.dynamics.exhaustVelocity;
exhaustVelSweep= SweepSolutions(thrustSweep(end),...
    'ExhaustVelocity',linspace(c1,c2,2),true);

sweepCG_Aligned_ConstrainedControlTorque_Mr= SweepSolutions(exhaustVelSweep(end), ...
    'rho',0.2,true);
PlotSolution.ConvergedCostateTrace(sweepCG_Aligned_ConstrainedControlTorque_Mr);
PlotSolution.Sweep6DOF(sweepCG_Aligned_ConstrainedControlTorque_Mr);
PlotSolution.ThrustProfileAllEngines(sweepCG_Aligned_ConstrainedControlTorque_Mr(end));

solCG6_Torque_RadSweep3= SweepSolutions(sweepCG_Aligned_ConstrainedControlTorque_Mr(end),'Radius', ...
    [0.1:.025:3]./1000,true);
solCG6_Torque_RadSweep38= SweepSolutions(solCG6_Torque_RadSweep3(end),'Radius', ...
    [3:.025:3.8]./1000,true);
solCG6_Torque_RadSweep385= SweepSolutions(solCG6_Torque_RadSweep38(end),'Radius', ...
    [3.8:.025:3.85]./1000,true);
solCG6_Torque_RadSweep39= SweepSolutions(solCG6_Torque_RadSweep385(end),'kappa', ...
    0.3,true);
solCG6_Torque_RadSweep39_LowK= SweepSolutions(solCG6_Torque_RadSweep39(end),'Radius', ...
    [3.85:.01:3.9]./1000,true);

PlotSolution.ThrustProfileAllEngines(solCG6_Torque_RadSweep39_LowK(end));
PlotSolution.ConvergedCostateTrace(solCG6_Torque_RadSweep3);
PlotSolution.Sweep6DOF(solCG6_Torque_RadSweep39_LowK,'Radius');

solCG6_R3_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep3(end), ...
    'kappa',1e-3,true);

solCG6_R3_TorqueRho= SweepSolutions(solCG6_R3_TorqueKappa(end), ...
    'rho',1e-2,true);

solCG6_R2_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep3(79), ...
    'kappa',1e-3,true);
solCG6_R2_TorqueRho= SweepSolutions(solCG6_R2_TorqueKappa(end), ...
    'rho',1e-2,true);

solCG6_R225_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep3(89), ...
    'kappa',1e-3,true);
solCG6_R225_TorqueRho= SweepSolutions(solCG6_R225_TorqueKappa(end), ...
    'rho',1e-2,true);

solCG6_R25_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep3(99), ...
    'kappa',1e-3,true);
solCG6_R25_TorqueRho= SweepSolutions(solCG6_R25_TorqueKappa(end), ...
    'rho',1e-2,true);

solCG6_R3_TorqueLowKappa= SweepSolutions(solCG6_R3_TorqueKappa(end), ...
    'kappa',1e-4,true);
allKappas =  [solCG6_R3_TorqueKappa, solCG6_R3_TorqueLowKappa];
PlotSolution.Sweep6DOF(allKappas);
PlotSolution.ConvergedCostateTrace(solCG6_R3_TorqueLowKappa);

kappaLowRadiusSweep = SweepSolutions(solCG6_R3_TorqueKappa(end),'Radius',(3:-0.05:1)/1e3, true);

solCG6_R1_TorqueRho= SweepSolutions(kappaLowRadiusSweep(end), ...
    'rho',1e-2,true); PlotSolution.Sweep6DOF(solCG6_R1_TorqueRho);
solCG6_R15_TorqueRho= SweepSolutions(kappaLowRadiusSweep(33), ...
    'rho',1e-2,true);
solCG6_R2_TorqueRho= SweepSolutions(kappaLowRadiusSweep(22), ...
    'rho',1e-2,true);
solCG6_R25_TorqueRho= SweepSolutions(kappaLowRadiusSweep(12), ...
    'rho',1e-2,true);
solCG6_R3_TorqueRho= SweepSolutions(kappaLowRadiusSweep(1), ...
    'rho',1e-2,true);
solCG6_R35_Kappa= SweepSolutions(solCG6_Torque_RadSweep38(22), ...
    'kappa',5e-4,true); 
solCG6_R35_TorqueRho= SweepSolutions(solCG6_R35_Kappa(72), ...
    'rho',1e-2,true);

%% R=3.4m, grid of solutions
testingTorque=SweepSolutions(solutionCG_Aligned_ControlTorque(1), ...
    'FinalX_Position',[0:0.05:1]*1e-3,true); 
testingTorqueK=SweepSolutions(testingTorque(end), ...
    'kappa',1e-3,true); 
testingTorqueKR=SweepSolutions(testingTorqueK(end), ...
    'rho',1e-3,true); 

torqueSeed =SweepSolutions(solutionCG_Aligned_ControlTorque, ...
    'rho',.1,true);  

seedA = testingTorque(end);
seedA.problemParameters.constraint=sweepCG_Aligned_ConstrainedControlTorque_Mr(end).problemParameters.constraint;
seedA = RerunSolution(seedA);
seedA_Rad = SweepSolutions(seedA,'Radius',[.1:.05:3.5]*1e-3,true); 
seedA_RadK = SweepSolutions(seedA_Rad(end),'kappa',1e-3,true); 
seedA_RadKE = SweepSolutions(seedA_RadK(end),'epsilon',1e-2,true); 
seedA_RadKER = SweepSolutions(seedA_RadK(17),'rho',1e-2,true); 
seedA_RadKE_34 = SweepSolutions(seedA_RadK(17),'Radius',[3.5:-.01:3.4]*1e-3,true); 
seedA_RadKER_34 = SweepSolutions(seedA_RadKE_34(end),'rho',1e-2,true); 

seedB = SweepSolutions(torqueSeed(end), ...
    'FinalX_Position',[0:-0.1:-1]*1e-3,true); 
seedB(end).problemParameters.constraint=sweepCG_Aligned_ConstrainedControlTorque_Mr(end).problemParameters.constraint;
seedB = RerunSolution(seedB(end));
seedB_Rad = SweepSolutions(seedB,'Radius',[.1:.1:3.5]*1e-3,true); 
seedB_RadK = SweepSolutions(seedB_Rad(end),'kappa',3e-3,true); 
seedB_RadKE = SweepSolutions(seedB_RadK(end),'epsilon',1e-2,true); 
seedB_RadKER = SweepSolutions(seedB_RadKE(end),'rho',1e-2,true); 
seedB_RadKE_34 = SweepSolutions(seedB_RadKE(end),'Radius',[3.5:-.01:3.4]*1e-3,true); 
seedB_RadKER_34 = SweepSolutions(seedB_RadKE_34(end),'rho',1e-2,true); 

seedC = SweepSolutions(torqueSeed(end), ...
    'FinalZ_Position',[0:0.1:1]*1e-3,true); 
seedC(end).problemParameters.constraint=sweepCG_Aligned_ConstrainedControlTorque_Mr(end).problemParameters.constraint;
seedC = RerunSolution(seedC(end));
seedC_Rad = SweepSolutions(seedC,'Radius',[.1:.15:3.5]*1e-3,true); 
seedC_RadK = SweepSolutions(seedC_Rad(end),'kappa',3e-3,true); 
seedC_RadKE = SweepSolutions(seedC_RadK(end),'epsilon',1e-2,true); 
seedC_RadKER = SweepSolutions(seedC_RadKE(end),'rho',1e-2,true); 

seedD = SweepSolutions(torqueSeed(end), ...
    'FinalZ_Position',[0:-0.1:-1]*1e-3,true); 
seedD(end).problemParameters.constraint=sweepCG_Aligned_ConstrainedControlTorque_Mr(end).problemParameters.constraint;
seedD = RerunSolution(seedD(end));
seedD_Rad = SweepSolutions(seedD,'Radius',[.1:.15:3.5]*1e-3,true); 
seedD_RadK = SweepSolutions(seedD_Rad(end),'kappa',3e-3,true); 
seedD_RadKE = SweepSolutions(seedD_RadK(end),'epsilon',1e-2,true); 
seedD_RadKER = SweepSolutions(seedD_RadKE(end),'rho',1e-2,true); 

seedE_Rad = SweepSolutions(seedC_Rad(end), ...
    'FinalX_Position',[0:-0.1:-1]*1e-3,true); 
seedE_RadK = SweepSolutions(seedE_Rad(end),'kappa',3e-3,true); 
seedE_RadKE = SweepSolutions(seedE_RadK(end),'epsilon',1e-2,true); 
seedE_RadKER = SweepSolutions(seedE_RadKE(end),'rho',1e-2,true); 

seedE = SweepSolutions(seedC(end), ...
    'FinalX_Position',[0:-0.1:-1]*1e-3,true); 
seedE_Rad2 = SweepSolutions(seedE(end),'Radius',[.1:.15:3.5]*1e-3,true); 
seedE_Rad2K = SweepSolutions(seedE_Rad2(end),'kappa',3e-3,true); 
seedE_Rad2KE = SweepSolutions(seedE_Rad2K(end),'epsilon',1e-2,true); 
seedE_Rad2KER = SweepSolutions(seedE_Rad2KE(end),'rho',1e-2,true); 

seedF_Rad = SweepSolutions(seedC_Rad(end), ...
    'FinalX_Position',[0:0.1:1]*1e-3,true); 
seedF_RadK = SweepSolutions(seedF_Rad(end),'kappa',3e-3,true); 
seedF_RadKE = SweepSolutions(seedF_RadK(end),'epsilon',1e-2,true); 
seedF_RadKER = SweepSolutions(seedF_RadKE(end),'rho',1e-2,true); 

seedG_Rad = SweepSolutions(seedD_Rad(end), ...
    'FinalX_Position',[0:-0.1:-1]*1e-3,true); 
seedG_RadK = SweepSolutions(seedG_Rad(end),'kappa',3e-3,true); 
seedG_RadKE = SweepSolutions(seedG_RadK(end),'epsilon',1e-2,true); 
seedG_RadKER = SweepSolutions(seedG_RadKE(end),'rho',1e-2,true); 

seedH_Rad = SweepSolutions(seedD_Rad(end), ...
    'FinalX_Position',[0:0.1:1]*1e-3,true); 
seedH_RadK = SweepSolutions(seedH_Rad(end),'kappa',3e-3,true); 
seedH_RadKE = SweepSolutions(seedH_RadK(end),'epsilon',1e-2,true); 
seedH_RadKER = SweepSolutions(seedH_RadKE(end),'rho',1e-2,true); 


PlotSolution.Sweep6DOF(seedE_RadK);
PlotSolution.Sweep6DOF(seedE_RadKER);
PlotSolution.Sweep6DOF(seedE_Rad2KER);
PlotSolution.ConvergedCostateTrace(seedA_RadK);

testing = solCG6_Torque_RadSweep3(end);
testing.problemParameters.dynamics.finalAttitudeFree = true; testing.solverParameters.fSolveOptions.MaxIterations=200;
sol=RerunSolution(testing);
freeMRP = SweepSolutions(sol, ...
    'kappa',1e-2,true);
freeMRPTinyKappa = SweepSolutions(freeMRP(end), ...
    'kappa',1e-4,true);

freeMRP1=SweepSolutions(freeMRPTinyKappa(3), ...
    'rho',1e-2,true);
freeMRP1eps=SweepSolutions(freeMRP1(end), ...
    'epsilon',1e-2,true);


freeMRPR32_Rad = SweepSolutions(freeMRPTinyKappa(1),...
    'Radius',[3:.01:3.25]*1e-3,true);
freeMRPR32_Kappa = SweepSolutions(freeMRPR32_Rad(end), ...
    'kappa',1e-3,true);
freeMRPR32_KR=SweepSolutions(freeMRPR32_Kappa(21), ...
    'rho',1e-2,true);
freeMRPR32_KRE=SweepSolutions(freeMRPR32_KR(end), ...
    'epsilon',1e-2,true);


PlotSolution.Sweep6DOF(freeMRPR32_KR); 
PlotSolution.ThrustProfileAllEngines(freeMRPR32_KRE(end)); 
PlotSolution.ConvergedCostateTrace(freeMRPR32_KR); 

PlotSolution.SixDOF_Traj(freeMRPR32_KRE(end))
for tim = [37]
    PlotSolution.OrientationAtTime(freeMRPR32_KRE(end),tim,gca)
end


% Larger values of R
solCG6_R32_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep38(10), ...
    'kappa',1e-3,true);
solCG6_R32_TorqueRho= SweepSolutions(solCG6_R32_TorqueKappa(end), ...
    'rho',1e-2,true);
solCG6_R32_TorqueEps= SweepSolutions(solCG6_R32_TorqueRho(end), ...
    'epsilon',1e-2,true);

solCG6_R35_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep38(22), ...
    'kappa',1e-3,true);
solCG6_R35_TorqueRho= SweepSolutions(solCG6_R35_TorqueKappa(end), ...
    'rho',1e-2,true);
solCG6_R35_TorqueEps= SweepSolutions(solCG6_R35_TorqueRho(end), ...
    'epsilon',1e-2,true);

solCG6_R35_TorqueKE= SweepSolutions(solCG6_R35_TorqueKappa(end), ...
    'epsilon',1e-2,true);
solCG6_R35_TorqueKER= SweepSolutions(solCG6_R35_TorqueKE(end), ...
    'rho',1e-2,true);

solCG6_R38_TorqueKappa= SweepSolutions(solCG6_Torque_RadSweep38(end), ...
    'kappa',1e-3,true); 
solCG6_R38_TorqueKE= SweepSolutions(solCG6_R38_TorqueKappa(end), ...
    'epsilon',1e-2,true);
solCG6_R38_TorqueKRadER= SweepSolutions(sol, ...
    'rho',1e-2,true); % not working

solCG6_R38_TorqueE= SweepSolutions(solCG6_Torque_RadSweep38(end), ...
    'epsilon',1e-2,true); % still not working 


end

function PlotSummary(sol)
    PlotSolution.ThrustProfileAllEngines(sol)
    PlotSolution.SixDOF_Traj(sol);
    
end