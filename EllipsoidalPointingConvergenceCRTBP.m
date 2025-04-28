function EllipsoidalPointingConvergenceCRTBP()
%% CRTBP Dynamics
    parallelFSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
    'MaxIter',3e1,'TolFun',1e-12,'TolX',1e-11,...
    'UseParallel',false); % fsolve  

    engineType = 3; % MONOPROPELLANT
    % engineType = 4; % COLD GAS
    
    %% 6dof, CRTBP dynamics, no constraint, Easy convergence
    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(4020, ...
        engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6); 
    

    % for i=1:1:100
    %     a=-1;b=1;
    %     solverParameters.initialCostateGuess = [(a + (b-a).*rand) (a + (b-a).*rand) (a + (b-a).*rand) (a + (b-a).*rand) (a + (b-a).*rand) (a + (b-a).*rand) (a + (b-a).*rand)   (a + (b-a).*rand)*10   0.000000000000007  (a + (b-a).*rand)*10  (a + (b-a).*rand)   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    %     solutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblemCRTBP(problemParameters,solverParameters);
    %     mat = solverParameters.initialCostateGuess;
    %     disp(mat);
    % end
        
    solverParameters.initialCostateGuess = [-0.3630 0.0681 -0.8201 -0.7766 -0.7274 0.3573 -0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    solutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblemCRTBP(problemParameters,solverParameters);
    
    solutionCG_AlignedUnconstrainedrhoSweep = SweepSolutions(solutionCG_AlignedUnconstrained, ...
        'rho',3e-2);
    solutionCG_AlignedUnconstrained = RerunSolution(solutionCG_AlignedUnconstrainedrhoSweep(end));

    PlotSolution.PlumeAngleSixDOF(solutionCG_AlignedUnconstrained);
    PlotSolutionOLD.SixDOF_TrajCRTBP(solutionCG_AlignedUnconstrained);


    % Sweep unconstrained trajectory (rho)

    %% 6DOF, CRTBP dynamics, ellipsoidal constraint, no attitude dynamics
    [problemParametersCG_6_Constrained_Ellips, solverParametersCG_6_Constrained_Ellips] = ConstrainedApproachTestCondition(4022,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solverParametersCG_6_Constrained_Ellips.initialCostateGuess = solutionCG_AlignedUnconstrained.newCostateGuess;
    problemParametersCG_6_Constrained_Ellips.constraint.epsilon = 0.8;

    problemParametersCG_6_Constrained_Ellips.constraint.targetAxisx = 0.0001;
    problemParametersCG_6_Constrained_Ellips.constraint.targetAxisy = 0.0001;
    problemParametersCG_6_Constrained_Ellips.constraint.targetAxisz = 0.0001;
    
    solverParametersCG_6_Constrained_Ellips = Solve6DOFPointingConstrainedControlProblemCRTBP(problemParametersCG_6_Constrained_Ellips,solverParametersCG_6_Constrained_Ellips);
    solverParametersCG_6_Constrained_Ellips.initialCostateGuess = solverParametersCG_6_Constrained_Ellips.newCostateGuess;

    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained_Ellips);

    newSols = SweepSolutions(solverParametersCG_6_Constrained_Ellips, 'Ellipsoid2', [linspace(0.0001,0.0015,60)',linspace(0.0001,0.001,60)',linspace(0.0001,0.002,60)'], true);
    solutionCG_Aligned_Constrained_Ellips = RerunSolution(newSols(end));

    PlotSolutionOLD.SixDOF_TrajCRTBP(solutionCG_Aligned_Constrained_Ellips);

    % First step and plot
    newSols2 = SweepSolutions(solutionCG_Aligned_Constrained_Ellips,'rho',1e-1);
    newSols2a = SweepSolutions(newSols2(end),'epsilon',0.5);
    solutionCG_Aligned_Constrained_EllipsSmallRhoSmallEps = RerunSolution(newSols2a(end));
    PlotSolutionOLD.PlumeAngleSixDOF(solutionCG_Aligned_Constrained_EllipsSmallRhoSmallEps);
    PlotSolutionOLD.SixDOF_TrajCRTBP(solutionCG_Aligned_Constrained_EllipsSmallRhoSmallEps);
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained_EllipsSmallRhoSmallEps);

    %Second sweep eps and plot
    newSols2b = SweepSolutions(newSols2a(end),'epsilon',4e-1);
    solutionCG_Aligned_Constrained_EllipsSmallRhoSmallSmallEps = RerunSolution(newSols2b(end));
    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained_EllipsSmallRhoSmallSmallEps);
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained_EllipsSmallRhoSmallSmallEps);
    PlotSolutionOLD.SixDOF_TrajCRTBP(solutionCG_Aligned_Constrained_EllipsSmallRhoSmallSmallEps);

    newSols2d = SweepSolutions(newSols2c(end),'rho',1e-3);
    newSols2e = SweepSolutions(newSols2d(end),'epsilon',1e-3);
    solutionCG_Aligned_Constrained_Ellips2 = RerunSolution(newSols2e(end));

    PlotSolution.ThrustProfileAllEngines(solutionCG_Aligned_Constrained_Ellips2);
    PlotSolution.PlumeAngleSixDOF(solutionCG_Aligned_Constrained_Ellips2);
    PlotSolutionOLD.SixDOF_TrajCRTBP(solutionCG_Aligned_Constrained_Ellips2);

    solverParametersCG_6_Constrained_Ellips3.initialCostateGuess = solutionCG_Aligned_Constrained_Ellips2.newCostateGuess;
