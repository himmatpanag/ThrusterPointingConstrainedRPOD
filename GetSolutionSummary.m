function summary = GetSolutionSummary(solution)

switch solution.problemParameters.constraint.type
    case POINTING_CONSTRAINT_TYPE.NONE
        %summary.title = ['Unconstrained ',solution.problemParameters.dynamics.type, ' dynamics'];
        summary.title = 'Unconstrained';
        summary.constraintString = summary.title;

    case POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE
        summary.title = ['Origin Pointing Constant Angle'];        
        summary.constraintString = ['\alpha = ', num2str(round(solution.problemParameters.constraint.alpha0*180/pi)),' deg'];

    case POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE    
        if strcmp(solution.problemParameters.dynamics.type,'Linear')
            rad = solution.problemParameters.constraint.targetRadius*1e3;
        else
            rad = solution.problemParameters.constraint.targetRadius;
        end 
        summary.title = ['Spherical Target Radius ',num2str(rad)];        
        summary.constraintString = ['Target Radius ', num2str(rad)];

    case POINTING_CONSTRAINT_TYPE.CURVE_TWO_DIMENSIONAL
        summary.title = ['Circular Segment Radius ',num2str(solution.problemParameters.constraint.curve.parametricRadial(1))];        
        summary.constraintString = ['Mirror Radius ',num2str(solution.problemParameters.constraint.curve.parametricRadial(1))];
end 

end 