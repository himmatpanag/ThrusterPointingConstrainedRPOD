[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011, ...
    3,THRUSTER_CONFIGURATION.RCS_12);
problemParameters.x0 = zeros(7,1);
PlotSolution.InitialOrientation(problemParameters)

[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011, ...
    engineType,THRUSTER_CONFIGURATION.RCS_12);
problemParameters.x0 = zeros(7,1);
PlotSolution.InitialOrientation(problemParameters)



[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011, ...
    engineType,THRUSTER_CONFIGURATION.RCS_14_CANTED);
problemParameters.x0 = zeros(7,1);
PlotSolution.InitialOrientation(problemParameters)


[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011, ...
    1,THRUSTER_CONFIGURATION.RCS_CANTED);
problemParameters.x0 = zeros(7,1);
PlotSolution.InitialOrientation(problemParameters)
