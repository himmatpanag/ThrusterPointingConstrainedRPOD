function newSol = RerunSolution(solutions)
% newSol = solutions;
    for ii = 1:numel(solutions)
        solution = solutions(ii);
        solverParameters=solution.solverParameters;
        solverParameters.initialCostateGuess = solution.newCostateGuess;
        newSol(ii) = SolvePointingConstrainedControlProblem(solution.problemParameters,solverParameters);
    end
end 