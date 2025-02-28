function solution = SixDOF_TestCases(caseNum)
if nargin < 1 
    caseNum = 1; 
    for ii = 1:5, SixDOF_TestCases(ii); end
end 

% Each one of these should cases converge with the costate guess provided
engineType = 3; % MONOPROPELLANT
solverParameters.initialCostateGuess = [rand(3,1)/1e3;rand(3,1);.01;rand(6,1)*1e-3]*1e-3;

switch caseNum
    case 1 % CG Aligned 6 thrusters transation
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType, ...
            THRUSTER_CONFIGURATION.CG_ALIGNED_6);
        solverParameters.initialCostateGuess = [-0.000089618015403   2.382117256137634   0.000000000000000  -2.076139519736225  57.115489612329533  -0.000000000000000   0.000142936265578  -0.000008931051246  -0.000001093674343 0.000003524540051  -0.000071361039315  -0.000010314917224   0.000008709836245]';
        solution = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
      
    case 2 % RCS_12, attitude only, 60 degree rotation 
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1100,engineType, ...
            THRUSTER_CONFIGURATION.RCS_12);
        solverParameters.initialCostateGuess = 1e-4.*[ 0.000000000000548   0.000000000009525  -0.000000000000206   0.000000000003950   0.000000000238142  -0.000000000017221   0.000194845229529 -0.037211240026859   0.000000000000000  -0.000000000000000  -0.893093076017197  -0.000000000000000  -0.000000000000002]';
        solution = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
        
    case 3 % RCS_12, attitude only, 180 degree rotation
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1101,engineType, ...
            THRUSTER_CONFIGURATION.RCS_12);
        solverParameters.initialCostateGuess = 1e-3.*[0.000000000000005  -0.000000000001046   0.000000000000030   0.000000000000953  -0.000000000025775   0.000000000000466   0.000691821455042  -0.022020484056926   0.000000000000000 0.000000000000000  -0.528505479191280   0.000000000000000   0.000000000000001]';
        solution = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
        
    case 4 % RCS_12, translation only
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType, ...
            THRUSTER_CONFIGURATION.RCS_12);
        solverParameters.initialCostateGuess = [-0.000089618015403   2.382117256137634   0.000000000000000  -2.076139519736225  57.115489612329533  -0.000000000000000   0.000142936265578  -0.000008931051246  -0.000001093674343 0.000003524540051  -0.000071361039315  -0.000010314917224   0.000008709836245]';
        solution = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);

    case 5 % As above with constraint
end 

PlotSolution.ThrustProfileAllEngines(solution)
PlotSolution.SixDOF_Traj(solution);

end 