function newSol = RerunSolution(solutions)
% newSol = solutions;
    for ii = 1:numel(solutions)
        solution = solutions(ii);
        solverParameters=solution.solverParameters;
        solverParameters.initialCostateGuess = solution.newCostateGuess;
        if numel(solution.newCostateGuess) > 10 % Then this includes attitude dynamics
            newSol(ii) = Solve6DOFPointingConstrainedControlProblem(solution.problemParameters,solverParameters);
        else
            newSol(ii) = SolvePointingConstrainedControlProblem(solution.problemParameters,solverParameters);
        end 
    end
end 