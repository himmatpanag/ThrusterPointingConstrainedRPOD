% SolvePointingConstrainedControlProblem is the core function. Requires two arguments which are automatically 
% saved to the solution structure. 
% Define the problem using problemParameters and solverParameters structure using
% ConstrainedApproachTestCondition.m Save new transfer conditions here.

[problemParameters, solverParameters] = ConstrainedApproachTestCondition(11);
newSolution = SolvePointingConstrainedControlProblem(problemParameters, solverParameters);

% SweepSolutions is a useful function to sweep rho, target radius,
% flightTIme, etc. It calls the function above
solutionsRhoSwept = SweepSolutions(newSolution,'rho',1e-4,true);

%% Plotting tools
% Useful plotting tool. Please add any tools here
PlotSolution.summary(newSolution);
PlotSolution.Sweep(solutionsRhoSwept);

PlotSolution.summary(solutionRhoLarge); 
PlotSolution.Sweep(solutions);
PlotSolution.SweepShort(solutions);

% 