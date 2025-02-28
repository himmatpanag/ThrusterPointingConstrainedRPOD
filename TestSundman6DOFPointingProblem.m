function TestSundman6DOFPointingProblem()
     % engineType = 4; % COLD GAS
     engineType = 3; % MONOPROPELLANT

    [problemParametersNormal, solverParametersNormal] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    [problemParametersSundman, solverParametersSundman] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
    solutionCG_AlignedUnconstrained.solutionFound = false; 
    solverParametersNormal.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    
    problemParametersSundman.dynamics.sundman.useTransform = true;
    problemParametersSundman.dynamics.sundman.useSimplified = true;
    problemParametersSundman.dynamics.sundman.theta=.9;
    solverParametersSundman.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000]';
    % solverParametersSundman.initialCostateGuess = [-0.152390378730075   2.390582055890830   0.000000000000007  -6.799259141355201  57.203929337461183   0.000000000000211   0.000143446214180  -0.000008931051000  -0.000001093674000   0.000003524540000  -0.000071361039000  -0.000010314917000   0.000008709836000 randn*1e-5]';
    % solverParametersSundman.initialCostateGuess = [rand(3,1)/1e3;rand(3,1);.01;rand(6,1)*1e-3;randn]*1e-3;

    SundmanSolution = Solve6DOFPointingConstrainedControlProblem(problemParametersSundman,solverParametersSundman);
    SundmanSweep = SweepSolutions(SundmanSolution,'rho',1e-4);

    OriginalSolution = Solve6DOFPointingConstrainedControlProblem(problemParametersNormal,solverParametersNormal);
    NormalSweep = SweepSolutions(OriginalSolution,'rho',1e-4);

    %% Q1 Do the converged solutions match?
        sol = SundmanSweep(end); PlotSolution.ThrustProfileAllEngines(sol); m1 = sol.x(end,7)
        sol = NormalSweep(end); PlotSolution.ThrustProfileAllEngines(sol); m2 = sol.x(end,7)
        % The solutions are practically the same and have the same cost. 
        format long; m1-m2
    %% Q2 Does the Sundman Sweep take fewer iterations for the above problem?
        numel(SundmanSweep) - numel(NormalSweep)
        % A1b) They take the same number of iterations
    %% Q3 Does the Sundman version give a good initial guess to the normal version for smaller values of rho?
        % part a) Start with initial costate guesses close to the actual value
        % part b) Start with random initial costate guesses 
        solverParametersSundman.fSolveOptions.MaxIterations = 300; solverParametersSundman.fSolveOptions.MaxFunctionEvaluations = 2000;
        solverParametersNormal.fSolveOptions.MaxIterations = 300; solverParametersNormal.fSolveOptions.MaxFunctionEvaluations = 2000;
        ii = 1;
        for rhoVals = [.4, .25, .2, .15, .1, .08, .05, .03]
            solverParametersSundman.rho=rhoVals; solverParametersNormal.rho=rhoVals;
            sundSolAttempt(ii) = Solve6DOFPointingConstrainedControlProblem(problemParametersSundman,solverParametersSundman);
            normalSolAttempt(ii) = Solve6DOFPointingConstrainedControlProblem(problemParametersNormal,solverParametersNormal);
            ii = ii+1;
        end
        
    SymbolicSolutionCG_AlignedUnconstrained = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    PlotSolution.ThrustProfileAllEngines(SymbolicSolutionCG_AlignedUnconstrained)
    
    solverParameters.rho=.25;
     problemParameters.dynamics.sundman.useTransform = false;
     
    SymbolicSolutionCG_AlignedUnconstrained.newCostateGuess - solutionCG_AlignedUnconstrained.newCostateGuess
    PlotSummary(solutionCG_AlignedUnconstrained);
    PlotSummary(SymbolicSolutionCG_AlignedUnconstrained);

end 