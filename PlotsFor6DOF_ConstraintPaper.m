%youfunction PlotsFor6DOF_ConstraintPaper
% This function contains results generated for 6DOF RPOD. 
parallelFSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
    'MaxIter',3e1,'TolFun',1e-12,'TolX',1e-11,...
    'UseParallel',false); % fsolve   

%% Unconstrained solution to 6x CG Aligned thrusters
%engineType = 1; % ION
%engineType = 2; % BIPROPELLANT
engineType = 3; % MONOPROPELLANT
% engineType = 4; % COLD GAS
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);

solutionCG_AlignedUnconstrained.solutionFound = false; 
solverParameters.initialCostateGuess = [-0.000089618015403   2.382117256137634   0.000000000000000  -2.076139519736225  57.115489612329533  -0.000000000000000   0.000142936265578  -0.000008931051246  -0.000001093674343 0.000003524540051  -0.000071361039315  -0.000010314917224   0.000008709836245]';
 
while any(~solutionCG_AlignedUnconstrained.solutionFound)
    solverParameters.initialCostateGuess = [rand(3,1)/1e3;rand(3,1);.01;rand(6,1)*1e-3]*1e-3;
    
    solutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    % unconstrainedInitialRho = PlotSolution.summary(solution);
end 
%solutionCG_AlignedUnconstrained = NEW_Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);

UnconstrainedShortTimeRhoSweep = SweepSolutions(solutionCG_AlignedUnconstrained,'rho',1e-4,true);
solutionCG_AlignedUnconstrainedSmallRho = RerunSolution(UnconstrainedShortTimeRhoSweep(end));
PlotSolution.ThrustProfileAllEngines(solutionCG_AlignedUnconstrainedSmallRho)
PlotSolution.SixDOF_Traj(solutionCG_AlignedUnconstrainedSmallRho);
PlotSolution.InitialOrientation(problemParameters);
PlotSolution.RotationalSummary(UnconstrainedShortTimeRhoSweep(end));

%% Unconstrained attitude only 60 degree rotation with control torques
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(2100,engineType,THRUSTER_CONFIGURATION.RCS_12);
solutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
solutionAttitudeOnlyRhoSweep = SweepSolutions(solutionAttitudeOnly,'rho',1e-4,true);

PlotSolution.ThrustProfileAllEngines(solutionAttitudeOnly)
PlotSolution.RotationalSummary(solutionAttitudeOnlyRhoSweep(end))

%% Unconstrained, attitude only, 180 degree rotation
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(2101,engineType,THRUSTER_CONFIGURATION.RCS_12);
solutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
solutionAttitudeOnlyRhoSweep = SweepSolutions(solutionAttitudeOnly,'rho',1e-4,true);

PlotSolution.ThrustProfileAllEngines(solutionAttitudeOnlyRhoSweep(end))
PlotSolution.RotationalSummary(solutionAttitudeOnlyRhoSweep(end))

%% Unconstrained CG aligned but canted thrusters
[problemParametersUnconstrainedCG_Canted, solverParametersUnconstrainedCG_Canted] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8);

solverParametersUnconstrainedCG_Canted.initialCostateGuess = [-0.000089618015403   2.382117256137634   0.000000000000000  -2.076139519736225  57.115489612329533  -0.000000000000000   0.000142936265578  -0.000008931051246  -0.000001093674343 0.000003524540051  -0.000071361039315  -0.000010314917224   0.000008709836245]';
solutionUnconstrainedCG_Canted = Solve6DOFPointingConstrainedControlProblem(problemParametersUnconstrainedCG_Canted,solverParametersUnconstrainedCG_Canted);

UnconstrainedCG_CantedShortTimeRhoSweep = SweepSolutions(solutionUnconstrainedCG_Canted,'rho',1e-4,true);
solutionCG_CantedUnconstrainedSmallRho = RerunSolution(UnconstrainedCG_CantedShortTimeRhoSweep(end));
PlotSolution.ThrustProfileAllEngines(solutionCG_CantedUnconstrainedSmallRho)
PlotSolution.SixDOF_Traj(solutionCG_CantedUnconstrainedSmallRho);
PlotSolution.InitialOrientation(problemParametersUnconstrainedCG_Canted);

% Now add a 2m constraint to this, Has no effect because thrusters are
% canted
[problemParametersCG_Canted_Constrained, solverParametersCG_Canted_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8);
solverParametersCG_Canted_Constrained.initialCostateGuess = solutionUnconstrainedCG_Canted.newCostateGuess;
problemParametersCG_Canted_Constrained = UpdateSphereCircleRadius(problemParametersCG_Canted_Constrained,2/1000);
problemParametersCG_Canted_Constrained.constraint.epsilon = .05;
solutionCG_Canted_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_Canted_Constrained,solverParametersCG_Canted_Constrained);
solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon = SweepSolutions(solutionCG_Canted_Constrained,'rhoepsilon',1e-4,true);

solutionCG_AlignedSphericalConstraint2_Converged = SweepSolutions(solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon(end),'rhoepsilon',1e-6,true);
solutionCG_Canted2mConstraintSmallRhoEpsilon = RerunSolution(solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon(end));
for sol = [solutionCG_Canted2mConstraintSmallRhoEpsilon(end), solutionCG_AlignedSphericalConstraint2_Converged]
    PlotSolution.ThrustProfileAllEngines(sol)
    PlotSolution.SixDOF_Traj(sol);
    PlotSolution.PlumeAngleSixDOF(sol)
end

%% 6x CG Aligned with 2m spherical constraint
[problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
problemParametersCG_6_Constrained = UpdateSphereCircleRadius(problemParametersCG_6_Constrained,2/1000);
problemParametersCG_Canted_Constrained.constraint.epsilon = .5;
solutionCG_Aligned_Constrained = RerunSolution(solutionCG_Aligned_Constrained); % Run this 4-5 times
solutionCG_Aligned_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon = SweepSolutions(solutionCG_Aligned_Constrained,'rhoepsilon',1e-4,true);

for sol = [solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon(26),solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon(end)]
    PlotSolution.ThrustProfileAllEngines(sol)
    PlotSolution.PlumeAngleSixDOF(sol)
    PlotSolution.SixDOF_Traj(sol);
end
PlotSolution.ConvergedCostateTrace(solutionCG_AlignedSphericalConstraint2_SmallRhoSmallEpsilon)

%% 6x CG aligned with control Torque 
[problemParametersControlTorque, solverParametersControlTorque] = ConstrainedApproachTestCondition(2011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersControlTorque.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
solutionCG_Aligned_ControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersControlTorque,solverParametersControlTorque);

sol =solutionCG_Aligned_ControlTorque; PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
rhoSweepCG_Aligned_ControlTorque = SweepSolutions(sol,'rho',1e-4,true);

sol =rhoSweepCG_Aligned_ControlTorque(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)

%% 6x CG aligned with and 2m spherical constraint AND control torque
[problemParametersCG_6_ConstrainedControlTorque, solverParametersCG_6_ConstrainedControlTorque] = ConstrainedApproachTestCondition(2013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_ConstrainedControlTorque.initialCostateGuess = solutionCG_Aligned_Constrained.newCostateGuess;
problemParametersCG_6_ConstrainedControlTorque = UpdateSphereCircleRadius(problemParametersCG_6_ConstrainedControlTorque,2/1000);
problemParametersCG_6_ConstrainedControlTorque.constraint.epsilon = .1;
solutionCG_Aligned_ConstrainedControlTorque = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstrainedControlTorque,solverParametersCG_6_ConstrainedControlTorque);
solutionCG_AlignedSphericalConstraintControlTorque2_SmallRhoSmallEpsilon = SweepSolutions(solutionCG_Aligned_ConstrainedControlTorque,'rhoepsilon',1e-4,true);

sol = solutionCG_AlignedSphericalConstraintControlTorque2_SmallRhoSmallEpsilon(end);
PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
PlotSolution.ConvergedCostateTrace(solutionCG_AlignedSphericalConstraintControlTorque2_SmallRhoSmallEpsilon);
PlotSolution.CostBreakdown(sol);

%% 6x CG aligned with control torque and 2m spherical constraint, free final MRP
[problemParametersCG_6_ConstrainedFreeMRP, solverParametersCG_6_ConstrainedFreeMRP] = ConstrainedApproachTestCondition(2013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_ConstrainedFreeMRP.initialCostateGuess = solutionCG_AlignedSphericalConstraintControlTorque2_SmallRhoSmallEpsilon(1).newCostateGuess;
problemParametersCG_6_ConstrainedFreeMRP = UpdateSphereCircleRadius(problemParametersCG_6_ConstrainedFreeMRP,2/1000);
problemParametersCG_6_ConstrainedControlTorque.constraint.epsilon = .1;
problemParametersCG_6_ConstrainedFreeMRP.dynamics.finalAttitudeFree = true;
% solverParametersCG_6_ConstrainedFreeMRP.fSolveOptions.MaxIterations = 300;
[problemParametersCG_6_ConstrainedFreeMRP, inertiaNewFinal] = UpdateInertia(problemParametersCG_6_ConstrainedFreeMRP,100);

solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_ConstrainedFreeMRP,solverParametersCG_6_ConstrainedFreeMRP);
solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia = RerunSolution(solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia);
solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep = SweepSolutions(solutionCG_Aligned_ConstrainedFreeMRP_LargeInertia,'Inertia',...
    linspace(inertiaNewFinal(1),inertiaNewFinal(2),50),true);

sol = solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)

solutionCG_Aligned_Constraint2_FreeMRP_SmallRhoSmallEpsilon = SweepSolutions(solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep(end),'rhoepsilon',1e-4,true);
% Steps are huge for the above problem. Is this due to epsilon or rho? 
% Try rho sweep alone to see what the norms of the steps are
SweepSolutions(solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep(end),'rho',1e-4,true);
% Still huge steps.
SweepSolutions(solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep(end),'epsilon',1e-4,true);
% Still huge steps.

sol = solutionCG_Aligned_Constraint2_FreeMRP_SmallRhoSmallEpsilon(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)

%% Try 45 degree rotation about z axis during rendezvous.
% First without constraint
[problemParametersControlTorqueRotation, solverParametersControlTorqueRotation] = ConstrainedApproachTestCondition(2011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
problemParametersControlTorqueRotation.pf = [0;0;1]*tan(45*pi/180/4); % = n*tan(theta/4), where n is unit vector, theta is angle of rotation. 
solverParametersControlTorqueRotation.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
solutionCG_Aligned_ControlTorqueRotation = Solve6DOFPointingConstrainedControlProblem(problemParametersControlTorqueRotation,solverParametersControlTorqueRotation);

sol =solutionCG_Aligned_ControlTorqueRotation; PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.SixDOF_Traj(sol);PlotSolution.RotationalSummary(sol)
rhoSweepCG_Aligned_ControlTorque = SweepSolutions(sol,'rho',1e-4,true);

sol =rhoSweepCG_Aligned_ControlTorque(end); PlotSolution.ThrustProfileAllEngines(sol); PlotSolution.RotationalSummary(sol)
PlotSolution.SixDOF_Traj(sol); PlotSolution.PlotOrientationChaserRelativeToTranslationalFrame(sol.x(end,8:10),sol.x(end,1:3)*1e3,sol.t(end),sol.problemParameters,gca)

% Then with constraint. FIX FSOLVE STEP SIZE 
[problemParametersControlTorqueRotationConstrained, solverParametersControlTorqueRotationConstrained] = ConstrainedApproachTestCondition(2013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersControlTorqueRotationConstrained.initialCostateGuess = solutionCG_Aligned_ControlTorqueRotation.newCostateGuess;
problemParametersControlTorqueRotationConstrained = UpdateSphereCircleRadius(problemParametersControlTorqueRotationConstrained,2/1000);
problemParametersControlTorqueRotationConstrained.pf = [0;0;1]*tan(45*pi/180/4); % = n*tan(theta/4), where n is unit vector, theta is angle of rotation. 
problemParametersControlTorqueRotationConstrained.constraint.epsilon = .1;

solutionCG_Aligned_ControlTorqueRotationConstrained = Solve6DOFPointingConstrainedControlProblem(problemParametersControlTorqueRotationConstrained,solverParametersControlTorqueRotationConstrained);


%% Above but experiment with larger torques (i.e. smaller cost multipliers)
torqueValues = linspace(problemParametersCG_6_ConstrainedFreeMRP.dynamics.torqueCostMultiplier,1e-4,20);
solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep(end).fSolveOptions.MaxIterations = 30;

sweep_CGAligned_Constraint2_FreeMRP_LowAttitudeCost = SweepSolutions(solutionCG_Aligned_ConstrainedFreeMRP_InertiaSweep(end),...
    'ControlTorqueCost', torqueValues,true);
solutionCG_Aligned_Constraint2_FreeMRP_LowAttitudeCost_SmallRhoSmallEpsilon = SweepSolutions(sweep_CGAligned_Constraint2_FreeMRP_LowAttitudeCost(end), ...
    'rhoepsilon',1e-4,true);            


%% 6x CG aligned with control Torque, Ruthvik's transfer, 100, transfer, slightly higher LEO orbit, smaller thrusters
[problemParametersRuthvik, solverParametersRuthvik] = ConstrainedApproachTestCondition(2200,5,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solutionRuthvik = Solve6DOFPointingConstrainedControlProblem(problemParametersRuthvik,solverParametersRuthvik);
solutionRuthvik = RerunSolution(solutionRuthvik);

finalI_Ruthvik = problemParametersRuthvik.dynamics.inertia(2,2);
initialI_Ruthvik = finalI_Ruthvik*100;
problemParametersRuthvik.dynamics.inertia = problemParametersRuthvik.dynamics.inertia*1e2;
problemParametersRuthvik.dynamics.inertiaInverse = problemParametersRuthvik.dynamics.inertiaInverse/1e2;
solverParametersRuthvik.rho = .5;
solverParametersRuthvik.initialCostateGuess(11:13) =0; 
solutionRuthvikLargeRho = Solve6DOFPointingConstrainedControlProblem(problemParametersRuthvik,solverParametersRuthvik);

outOfPlaneRCS_12_InertiaSweep = SweepSolutions(solutionRuthvikLargeRho,'Inertia',linspace(initialI_Ruthvik,finalI_Ruthvik*2,20),true);
solRuthvikRhoSweep = SweepSolutions(outOfPlaneRCS_12_InertiaSweep(15),'rho',.1,true);
outOfPlaneRCS_12_InertiaSweep = SweepSolutions(solRuthvikRhoSweep(end),'Inertia',linspace(solRuthvikRhoSweep(end).problemParameters.dynamics.inertia(2,2),finalI_Ruthvik*5,20),true);
solRuthvikRhoSweepinertiaX5 = SweepSolutions(outOfPlaneRCS_12_InertiaSweep(end),'rho',1e-4,true);

k = solRuthvikRhoSweepinertiaX5(end);
PlotSolution.SixDOF_Traj(k)
PlotSolution.ThrustProfileAllEngines(k)
PlotSolution.RotationalSummary(k)

%% Unconstrained RCS_12 with 180 degree rotation and out of plane maneuvre
% Try inertia sweep
[problemParametersRCS12_UnconsOutOfPlane, solverParametersRCS12_UnconsOutOfPlane] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.RCS_12);
finalI = problemParametersRCS12_UnconsOutOfPlane.dynamics.inertia(2,2);
initialI = finalI*100;
problemParametersRCS12_UnconsOutOfPlane.dynamics.inertia = problemParametersRCS12_UnconsOutOfPlane.dynamics.inertia*1e2;
problemParametersRCS12_UnconsOutOfPlane.dynamics.inertiaInverse = problemParametersRCS12_UnconsOutOfPlane.dynamics.inertiaInverse/1e2;
problemParametersRCS12_UnconsOutOfPlane.xf(3) = 0.004;
solverParametersRCS12_UnconsOutOfPlane.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;

seedOutOfPlane = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_UnconsOutOfPlane,solverParametersRCS12_UnconsOutOfPlane);
solverParametersRCS12_UnconsOutOfPlane.initialCostateGuess = seedOutOfPlane.newCostateGuess;

outOfPlaneRCS_12_InertiaSweep = SweepSolutions(seedOutOfPlane,'Inertia',linspace(initialI,finalI*2,20),true);
outOfPlaneRCS_12_rhoSweepDoubleInertia = SweepSolutions(outOfPlaneRCS_12_InertiaSweep(end),'rho',1e-4,true);
solverParameters.fSolveOptions.maxIter = 30;

%% Unconstrained RCS_12 with no attitude rotation
[problemParametersRCS12_Uncons, solverParametersRCS12_Uncons] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_6_RCS_12);
problemParametersRCS12_Uncons.dynamics.maxThrust(7:18) = 0;
% try to use initial costate guess from RCS_6 to guess this sol. 
solverParametersRCS12_Uncons.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
seed = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_Uncons,solverParametersRCS12_Uncons);

thrusterConfigSweep = SweepSolutions(seed,'ThrusterConfig',linspace(0,problemParametersRCS12_Uncons.dynamics.maxThrust(1),100),true);
PlotSolution.SixDOF_Traj(thrusterConfigSweep(end))
PlotSolution.SwitchFunctionAnalysisAllEngines(thrusterConfigSweep(end));
PlotSolution.ThrustProfileAllEngines(thrusterConfigSweep(end))
PlotSolution.RotationalSummary(thrusterConfigSweep(end))
PlotSolution.ConvergedCostateTrace(thrusterConfigSweep)

% Try inertia sweep
[problemParametersRCS12_Uncons2, solverParametersRCS12_Uncons2] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.RCS_12);
finalI = problemParametersRCS12_Uncons2.dynamics.inertia(2,2);
initialI = finalI*1000;
problemParametersRCS12_Uncons2.dynamics.inertia = problemParametersRCS12_Uncons2.dynamics.inertia*1e3;
problemParametersRCS12_Uncons2.dynamics.inertiaInverse = problemParametersRCS12_Uncons2.dynamics.inertiaInverse/1e3;

%% TRY THIS AGAIN AFTER MODIFYING ANDSCSLING THE COST. IF IT STILL DOESNT WORK
%% SOLVE THE PROBLEM WITH REACTION WHEELS
% 2. Debug what is happening as rho reduces, some kind of bug (for high
% inertia). Also there is some kind of limit (reachability for lower
% inertias?) Or is it just that a tiny thrust causes a large rotation
% angle. Is it reasonable to assume that fine attitude control (<1deg) is only
% achievable with reaction wheels? If so, scale the final state error for
% the MRP terms, This will likely aid in convergence significantly.
% 3. Change to reaction wheel formulation 

seed2 = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_Uncons2,solverParametersRCS12_Uncons2);
PlotSolution.InitialOrientation(seed2);
PlotSolution.SixDOF_Traj(seed2)
PlotSolution.ThrustProfileAllEngines(seed2)
PlotSolution.RotationalSummary(seed2)

UnconstrainedRCS_12_InertiaSweep = SweepSolutions(seed2,'Inertia',linspace(initialI,finalI),true);
load("matlab.mat")
k = UnconstrainedRCS_12_InertiaSweep(end); k = RerunSolution(k);
UnconstrainedRCS_12_InertiaSweep2 = SweepSolutions(k, 'Inertia',linspace(k.problemParameters.dynamics.inertia(2,2),finalI,20),true);
PlotSolution.SixDOF_Traj(k)
PlotSolution.ThrustProfileAllEngines(k)
PlotSolution.RotationalSummary(k)

ScaledErrorTermsRhoSweep_SmallInertia = SweepSolutions(k,'rho',1e-2,true);
k = RerunSolution(ScaledErrorTermsRhoSweep_SmallInertia(end));
ScaledErrorTermsRhoSweepSmaller_SmallInertia = SweepSolutions(k,'rho',1e-4,true);
PlotSolution.ThrustProfileAllEngines(UnconstrainedRCS_12_TripleInertiaRhoSweep(end))

UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep = SweepSolutions(UnconstrainedRCS_12_TripleInertiaRhoSweep(end), 'Inertia',...
    linspace(UnconstrainedRCS_12_TripleInertiaRhoSweep(end).problemParameters.dynamics.inertia(2,2),2*finalI,40),true);

PlotSolution.ThrustProfileAllEngines(UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep(end))
UnconstrainedRCS_12_FinalRhoDoubleInertiaSweep = SweepSolutions(UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep(end),'rho',1e-4,true);


UnconstrainedRCS_12_TripleInertiaRhoSweep_3 = SweepSolutions(UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep(end),'rho',1e-3,true);



UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep2 = SweepSolutions(k, 'Inertia',...
    linspace(UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep(end).problemParameters.dynamics.inertia(2,2),2*finalI,20),true);
UnconstrainedRCS_12_DoubleInertiaRhoSweep = SweepSolutions(UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep2(end),'rho',1e-6);
UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep3 = SweepSolutions(k, 'Inertia',...
    linspace(UnconstrainedRCS_12_SmallRhoDoubleInertiaSweep2(end).problemParameters.dynamics.inertia(2,2),finalI,20),true);
[problemParametersRCS12_Uncons2, solverParametersRCS12_Uncons2] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.RCS_12);
solverParametersRCS12_Uncons2.initialCostateGuess = UnconstrainedRCS_12_InertiaSweep(end-1).newCostateGuess;
seed3 = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_Uncons2,solverParametersRCS12_Uncons2);


solverParametersRCS12_Uncons2.initialCostateGuess = thrusterConfigSweep(end).newCostateGuess;
solverParametersRCS12_Uncons2.initialCostateGuess = [rand(3,1)/1e3;rand(3,1)*10;rand(7,1)*1e-4];
solverParametersRCS12_Uncons2.initialCostateGuess(4:6) = lv;
seed2 = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_Uncons2,solverParametersRCS12_Uncons2);

UnconstrainedRCS_12_RhoSweep = SweepSolutions(thrusterConfigSweep(end),'rho',1e-4,true);

solverParametersRCS12_Uncons.initialCostateGuess = seed.newCostateGuess;
while norm(seed1.finalStateError) > 5
    solverParametersRCS12_Uncons.initialCostateGuess(11:13) = lw;
    seed1 = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_Uncons,solverParametersRCS12_Uncons);
end

lw = seed.newCostateGuess(11:13);
lw= seed.newCostateGuess([13,11,12]);
lw= seed.newCostateGuess([13,12,11]);
lw= seed.newCostateGuess([12,13,11]);
lw= seed.newCostateGuess([12,11,13]);
lw= seed.newCostateGuess([11,13,12]);
for ii = 1:12
    lv = seed.newCostateGuess(4:6);
    u = problemParametersRCS12_Uncons.dynamics.thrustDirectionBody(:,ii);
    rv = cross(problemParametersRCS12_Uncons.dynamics.engineLocationBody(:,ii),u);
    ang = acos(dot(lv,u)/norm(lv))*180/pi;
    ang2 = acos(dot(lw,rv)/norm(rv)/norm(lw))*180/pi;
    fprintf(['Engine %d \t lambda_v %3.0f\tlambda_w %3.0f,\n'],ii,ang,ang2)
end

seed2 = Solve6DOFPointingConstrainedControlProblem(problemParametersRCS12_Uncons,solverParametersRCS12_Uncons);
PlotSolution.InitialOrientation(k);
PlotSolution.SixDOF_Traj(seed)
PlotSolution.SwitchFunctionAnalysisAllEngines(seed2);
PlotSolution.ThrustProfileAllEngines(seed2)
PlotSolution.RotationalSummary(seed2)

PlotSolution.SwitchFunctionAnalysisAllEngines(k);
PlotSolution.ThrustProfileAllEngines(k)
PlotSolution.RotationalSummary(k)

k=thrusterConfigSweep(27);
test = SweepSolutions(k,'ThrusterConfig',linspace(k.problemParameters.dynamics.maxThrust(end),5.8e-4,20));
PlotSolution.ConvergedCostateTrace(newSols)

seedSweep = SweepSolutions(seed,'rho',1e-4);
PlotSolution.SwitchFunctionAnalysisAllEngines(seed1);


%% Unconstrained, translation sweep wit 60 degree rotation - can't get passed Pos = 7.5 with this sweep
TranslationSweep =SweepSolutions(solutionAttitudeOnly,'FinalY_Position',linspace(0.01,0.004,40),true); % can't get passed Pos = 7.5 with this sweep
% Sweep the MRP from 30 degrees to zero 
PlotSolution.ConvergedCostateTrace(TranslationSweep,[10,4])

k = RerunSolution(newSols(3));
PlotSolution.ConvergedCostateTrace(newSols)
PlotSolution.ThrustProfileAllEngines(k)
PlotSolution.RotationalSummary(k)
PlotSolution.SixDOF_Traj(k);


RhoSweepSomeTranslation =SweepSolutions(k,'rho',1e-4,true);
PlotSolution.ThrustProfileAllEngines(RhoSweepSomeTranslation(end))
PlotSolution.RotationalSummary(RhoSweepSomeTranslation(end))
PlotSolution.SixDOF_Traj(RhoSweepSomeTranslation(end));
PlotSolution.ConvergedCostateTrace(RhoSweepSomeTranslation,[.5,1e-4])

%% 6x CG aligned + a pair of additional thrustuers, what happens here? - experiment with this?
 % experimenting with convergence as number of thrusters increases. expect to find bugs in attitude dynamics

%% RCS_12 configuration - currentlyNotWorking
[problemParametersRCS12_Uncons, solverParametersRCS12_Uncons] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.RCS_12);
% try to use initial costate guess from RCS_6 to guess this sol. 
solverParametersRCS12_Uncons.initialCostateGuess = [rand(3,1)/1e3;rand(2,1)*10;rand(8,1)*1e-4];
solutionRCS_12Unconstrained = Solve6DOFPointingConstrainedControlProblem( ...
    problemParametersRCS12_Uncons,solverParametersRCS12_Uncons);
PlotSolution.InitialOrientation(problemParametersRCS12_Uncons)
PlotSolution.(k)
PlotSolution.ThrustProfileAllEngines(k)
PlotSolution.RotationalSummary(k)


%% Plots 
figure; PlotSolution.addTrajectory(solutionCG_AlignedUnconstrained,gca)
figure; grid on; plot(solutionCG_AlignedUnconstrained.t, sum(solutionCG_AlignedUnconstrained.throttle)); hold on; plot(solutionCG_AlignedUnconstrained.t,vecnorm(solutionCG_AlignedUnconstrained.thrustTranslationalFrame))
figure; plot(solutionCG_AlignedUnconstrained.t,solutionCG_AlignedUnconstrained.throttle); legend('show')
figure; PlotSolution.PlotInitialOrientation(problemParameters,gca);
PlotSolution.SwitchFunctionAnalysis(solutionCG_AlignedUnconstrained)
PlotSolution.SwitchFunctionAnalysisAllEngines(solutionCG_AlignedUnconstrained)
figure; grid on; hold on; plot(solutionCG_AlignedUnconstrained.t,solutionCG_AlignedUnconstrained.thrustTranslationalFrame)
        

%% Comparison to 3DOF 
% comparison to 3dof problem
engineType = 3; % MONOPROPELLANT
% engineType = 4; % COLD GAS
[problemParameters3DOF, solverParameters3DOF] = ConstrainedApproachTestCondition(11,engineType);
problemParameters3DOF.transferTime = 48;
solverParameters3DOF.tSpan = [0, problemParameters3DOF.transferTime];
solverParameters3DOF.initialCostateGuess =  1e-5*[0.002793174850374   0.033568538261829   0.000290206223243  -0.002250790435555   1.337498833488286   0.002331243603019   0.000000148370925  -0.034666484751677   0.009702146188198 0.225697266167110  -0.284304178384577  -0.104361818552909   0.465951552533527]';
solverParameters3DOF.initialCostateGuess =  [-0.000061909304126   1.683061783958727  -0.000000000000181  -1.449486254206033  40.355191913670460  -0.000000000007311   0.000100989532481]';
solution3DOF = SolvePointingConstrainedControlProblem(problemParameters3DOF,solverParameters3DOF);
solution3DOF = RerunSolution(solution3DOF);

