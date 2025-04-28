function newSol = RerunSolution(solutions)
% newSol = solutions;
    for ii = 1:numel(solutions)
        solution = solutions(ii);
        solverParameters=solution.solverParameters;
        solverParameters.initialCostateGuess = solution.newCostateGuess;
        %if numel(solution.newCostateGuess) > 10 % Then this includes attitude dynamics
        if strcmp(solution.problemParameters.dynamics.type, 'CRTBP')
            newSol(ii) = Solve6DOFPointingConstrainedControlProblemCRTBP(solution.problemParameters,solverParameters);
        else
            newSol(ii) = Solve6DOFPointingConstrainedControlProblem(solution.problemParameters,solverParameters);
        end
        %else
            %newSol(ii) = SolvePointingConstrainedControlProblem(solution.problemParameters,solverParameters);
        %end 
    end
end 