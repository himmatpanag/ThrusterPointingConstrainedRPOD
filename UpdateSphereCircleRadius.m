function problemParameters = UpdateSphereCircleRadius(problemParameters,R)
    problemParameters.constraint.angleFunc = @(rNorm) asin(R/rNorm);
    problemParameters.constraint.angleFuncDerivative = @(r) -r.*R./(r'*r*sqrt(r'*r-R^2));        
    problemParameters.constraint.targetRadius = R;

end
