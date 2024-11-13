function PlotsFor6DOF_ConstraintPaper
% This function contains results generated for 6DOF RPOD. 
parallelFSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
    'MaxIter',3e2,'TolFun',1e-12,'TolX',1e-9,...
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

UnconstrainedShortTimeRhoSweep = SweepSolutions(solutionCG_AlignedUnconstrained,'rho',1e-4,false);
solutionCG_AlignedUnconstrainedSmallRho = RerunSolution(UnconstrainedShortTimeRhoSweep(end));
PlotSolution.ThrustProfileAllEngines(solutionCG_AlignedUnconstrainedSmallRho)
PlotSolution.SixDOF_Traj(solutionCG_AlignedUnconstrainedSmallRho);
PlotSolution.InitialOrientation(problemParameters);

%% Unconstrained attitude only
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1100,engineType,THRUSTER_CONFIGURATION.RCS_12);
solutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
solutionAttitudeOnlyRhoSweep = SweepSolutions(solutionAttitudeOnly,'rho',1e-4,true);

PlotSolution.ThrustProfileAllEngines(solutionAttitudeOnlyRhoSweep(end))
PlotSolution.RotationalSummary(solutionAttitudeOnlyRhoSweep(end))

%% Unconstrained, attitude only, 180 degree rotation
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1101,engineType,THRUSTER_CONFIGURATION.RCS_12);
solutionAttitudeOnly = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
solutionAttitudeOnlyRhoSweep = SweepSolutions(solutionAttitudeOnly,'rho',1e-4,true);

PlotSolution.ThrustProfileAllEngines(solutionAttitudeOnlyRhoSweep(end))
PlotSolution.RotationalSummary(solutionAttitudeOnlyRhoSweep(end))

%% 6x CG Aligned with pointing constraint
% expect to find and fix bugs with constraint here
[problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
problemParametersCG_6_Constrained = UpdateSphereCircleRadius(problemParametersCG_6_Constrained,1/1000);
solutionCG_Aligned_Constrained = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);
solutionCG_AlignedConstrainedRad1_SmallRho = SweepSolutions(solutionCG_Aligned_Constrained,'rho',1e-4,false);
solutionCG_AlignedConstrainedRad1_SmallRhoSmallEpsilon = SweepSolutions(solutionCG_AlignedConstrainedRad1_SmallRho(end),'epsilon',1e-4,false);


[problemParametersCG_6_Constrained, solverParametersCG_6_Constrained] = ConstrainedApproachTestCondition(1013,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
solverParametersCG_6_Constrained.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
problemParametersCG_6_Constrained = UpdateSphereCircleRadius(problemParametersCG_6_Constrained,.2/1000);
solutionCG_Aligned_ConstrainedSmallRad = Solve6DOFPointingConstrainedControlProblem(problemParametersCG_6_Constrained,solverParametersCG_6_Constrained);

%% 6x CG aligned + a pair of additional thrustuers, what happens here? - experiment with this?
 % experimenting with convergence as number of thrusters increases. expect to find bugs in attitude dynamics

%% RCS_12 configuration - currentlyNotWorking
[problemParametersRCS12_Uncons, solverParametersRCS12_Uncons] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.RCS_12);
% try to use initial costate guess from RCS_6 to guess this sol. 
solverParametersRCS12_Uncons.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
solutionRCS_12Unconstrained = Solve6DOFPointingConstrainedControlProblem( ...
    problemParametersRCS12_Uncons,solverParametersRCS12_Uncons);
PlotSolution.InitialOrientation(problemParametersRCS12_Uncons)

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

end