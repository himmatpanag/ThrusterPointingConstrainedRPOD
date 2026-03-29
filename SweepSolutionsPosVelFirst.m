function solsOut = SweepSolutionsPosVelFirst(solution,sweepType,values)

problemParametersOriginal = solution.problemParameters;
solverParametersOriginal = solution.solverParameters;

solution.problemParameters.dynamics.attitudeActuator = ATTITUDE_CONTROL_TYPE.NONE;
solsOut(1) = solution; iter = 1;
switch sweepType
    case 'rho'
        rhoPrev = solution.solverParameters.rho;
        rhoRate = .99;
        while rhoPrev > values
            solverParametersOriginal.rho = rhoRate*rhoPrev;
            fprintf('rho = %1.3d \t modifying position, velocity only\n', rhoRate*rhoPrev)
            sol1 = Solve6DOFPointingConstrainedControlProblem(solution.problemParameters,solverParametersOriginal);
            solverParametersOriginal.initialCostateGuess = sol1.newCostateGuess;
            fprintf('rho = %1.3d \t modifying all costates\n', rhoRate*rhoPrev)
            dsol2 = Solve6DOFPointingConstrainedControlProblem(problemParametersOriginal,solverParametersOriginal);
            if sol2.solutionFound
                fprintf('Solution found for rho = %1.3d\n', rhoRate*rhoPrev)
                rhoPrev = solverParametersOriginal.rho;
                rhoRate = max(.5,rhoRate-.05);
                iter = iter + 1;
                solsOut(iter) = sol2;
                solverParametersOriginal.initialCostateGuess = sol2.newCostateGuess;
            else
                rhoRate = min(.999,rhoRate + (1-rhoRate)/2);
                solverParametersOriginal.initialCostateGuess = solsOut(iter).newCostateGuess;
            end
        end 
end

end